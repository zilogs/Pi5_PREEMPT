/* main.c — standalone C MSCKF-VIO for the AirSim "arc" EuRoC dataset.
 * Mirrors F5_C4_airsim_triangulate_double.m main loop (v81 config, FEJ off).
 * Frontend: min-eigen detect + grid reseed + pyramidal KLT + 2-pt RANSAC + undistort.
 * Backend : IMU-prop + v70/v79c/v81 triggers + augment + gated update + eviction. */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <sched.h>     /* sched_setaffinity / sched_getcpu (needs _GNU_SOURCE, defined above) */

#include "vio.h"
#include "msckf_imu.h"
#include "msckf_augment.h"
#include "klt_tracker.h"
#include "png_load.h"
#include "vio_prof.h"

#define SZ MSCKF_MAX_STATE_SZ

/* ─────────────────────────── CPU pinning ───────────────────────────
 * Pin the whole process to ONE core. Call this first thing in main(): the affinity mask is
 * inherited by any thread created afterwards (pthread / OpenMP inside klt_tracker etc.),
 * so everything stays on that core. Core index is 0-based (core 3 = the 4th logical CPU).
 * Override at build time:  -DPIN_CORE=2   — or at run time: PIN_CORE=2 ./vio ... */
#ifndef PIN_CORE
#define PIN_CORE 3
#endif
static void pin_to_core(int core)
{
    const char *e = getenv("PIN_CORE");
    if (e && *e) core = atoi(e);
    cpu_set_t set; CPU_ZERO(&set); CPU_SET(core, &set);
    if (sched_setaffinity(0, sizeof set, &set) != 0) {   /* pid 0 = calling process/thread */
        fprintf(stderr, "warn: cannot pin to core %d (%s) — running unpinned\n", core, strerror(errno));
        return;
    }
    fprintf(stderr, "pinned to core %d (now running on cpu %d)\n", core, sched_getcpu());
}

/* ─────────────────────────── data storage ─────────────────────────── */
static long long *imu_t; static double (*imu_g)[3], (*imu_a)[3]; static int n_imu;
static long long *cam_t_ns; static char (*cam_files)[64]; static int n_cam;
static long long *gt_t_ns; static double (*gt_p)[3], (*gt_q)[4], (*gt_v)[3], (*gt_bg)[3], (*gt_ba)[3]; static int n_gt;

static char *rd_line(FILE *f, char *buf, int cap) { return fgets(buf, cap, f); }

#ifdef ZUPT_PROBE
/* [ZUPT PROBE] per-frame detector/gate log. Opt-in via env ZUPT_LOG=<path>.
 * Columns (Prof lock: detector stats, flag, NIS, accept/reject, v before/after):
 *   t_sec,gyro_std,accel_std,accel_norm,stationary,zupt_nis,gate_pass,applied,v_before,v_after
 *   gate_pass: -1 = gate not evaluated (not stationary / singular S), 0 = rejected, 1 = passed
 * Takeoff marks are NEVER read here — they are used only offline to score detector FP/FN. */
static FILE *zupt_log = NULL;
static void zupt_log_open(void)
{
    const char *p = getenv("ZUPT_LOG");
    if (!p || !*p) return;
    zupt_log = fopen(p, "w");
    if (zupt_log)
        fprintf(zupt_log, "t_sec,gyro_std,accel_std,accel_norm,stationary,"
                          "zupt_nis,gate_pass,applied,v_before,v_after\n");
}
#endif

/* PROBE2: the Kalibr radtan polynomial (k1=+0.127, k2=-0.169) is non-invertible
 * beyond distorted radius ~1.003 and the fixed-point undistort diverges there
 * (716px roundtrip at the far corner). Observations outside a safe distorted
 * radius carry no usable geometry — drop them at track-keep/reseed time. */
#define VIO_UNDIST_RD_MAX 0.90
static int undist_in_range(const cam_intrinsics_t *K, double u, double v)
{
    double x = (u - K->cx) / K->fx, y = (v - K->cy) / K->fy;
    return x*x + y*y <= VIO_UNDIST_RD_MAX * VIO_UNDIST_RD_MAX;
}

/* ─────────────────────────── yaml calib parse ─────────────────────── */
static int parse_yaml_array(const char *text, const char *key, double *out, int maxn)
{
    const char *p = text;
    size_t klen = strlen(key);
    while ((p = strstr(p, key)) != NULL) {
        const char *q = p + klen;
        while (*q==' '||*q=='\t') q++;
        if (*q != ':') { p += klen; continue; }
        q++;
        while (*q && *q!='[') { if (*q=='\n') break; q++; }
        if (*q != '[') { p += klen; continue; }
        q++;
        int n = 0;
        while (*q && *q != ']' && n < maxn) {
            char *end;
            double v = strtod(q, &end);
            if (end == q) { q++; continue; }
            out[n++] = v; q = end;
            while (*q==' '||*q==','||*q=='\t') q++;
        }
        return n;
    }
    return 0;
}

static int load_calib(const char *mav0, msckf_calib_t *calib, cam_intrinsics_t *K)
{
    char path[1024]; snprintf(path, sizeof path, "%s/cam0/sensor.yaml", mav0);
    FILE *f = fopen(path, "rb"); if (!f) { perror(path); return 0; }
    fseek(f,0,SEEK_END); long sz=ftell(f); fseek(f,0,SEEK_SET);
    if (sz <= 0) { fclose(f); fprintf(stderr,"bad sensor.yaml size\n"); return 0; }
    char *txt = malloc((size_t)sz+1); if(!txt){fclose(f);return 0;}
    size_t got = fread(txt,1,(size_t)sz,f); txt[got]=0; fclose(f);
    double T[16], intr[4], dist[8];
    int nT = parse_yaml_array(txt, "data", T, 16);
    int ni = parse_yaml_array(txt, "intrinsics", intr, 4);
    int nd = parse_yaml_array(txt, "distortion_coefficients", dist, 8);
    free(txt);
    if (nT < 16 || ni < 4) return 0;
    /* T_BS row-major 4x4: imu_R_camera = 3x3 top-left, imu_t_camera = col 4 rows 1-3 */
    calib->imu_R_camera[0]=T[0]; calib->imu_R_camera[1]=T[1]; calib->imu_R_camera[2]=T[2];
    calib->imu_R_camera[3]=T[4]; calib->imu_R_camera[4]=T[5]; calib->imu_R_camera[5]=T[6];
    calib->imu_R_camera[6]=T[8]; calib->imu_R_camera[7]=T[9]; calib->imu_R_camera[8]=T[10];
    calib->imu_t_camera[0]=T[3]; calib->imu_t_camera[1]=T[7]; calib->imu_t_camera[2]=T[11];
    K->fx=intr[0]; K->fy=intr[1]; K->cx=intr[2]; K->cy=intr[3];
    K->k1 = nd>0?dist[0]:0; K->k2 = nd>1?dist[1]:0; K->p1 = nd>2?dist[2]:0; K->p2 = nd>3?dist[3]:0;
    return 1;
}

/* alloc that fails loud (data-ingestion boundary; embedded OOM must not become a silent NULL-deref) */
static void *xrealloc(void *p, size_t sz){
    void *q = realloc(p, sz);
    if (!q) { fprintf(stderr, "fatal: out of memory (%zu bytes)\n", sz); exit(1); }
    return q;
}

/* read `n` comma fields into out[]; returns 0 (and leaves row to be skipped) on short/malformed row */
static int tok_doubles(double *out, int n){
    for (int k = 0; k < n; k++) {
        char *t = strtok(NULL, ",\r\n");
        if (!t) return 0;
        out[k] = atof(t);
    }
    return 1;
}

static void load_data(const char *mav0)
{
    char path[1024], line[4096];
    int n_bad = 0;
    /* IMU */
    snprintf(path,sizeof path,"%s/imu0/data.csv",mav0);
    FILE *f=fopen(path,"rb"); if(!f){perror(path);exit(1);}
    int cap=70000; imu_t=xrealloc(NULL,cap*sizeof(*imu_t)); imu_g=xrealloc(NULL,cap*sizeof(*imu_g)); imu_a=xrealloc(NULL,cap*sizeof(*imu_a));
    n_imu=0; rd_line(f,line,sizeof line);
    while(rd_line(f,line,sizeof line)){
        char*t=strtok(line,",\r\n"); if(!t)continue;
        imu_t[n_imu]=atoll(t);
        if(!tok_doubles(imu_g[n_imu],3) || !tok_doubles(imu_a[n_imu],3)){ n_bad++; continue; }
        if(++n_imu>=cap){ cap*=2; imu_t=xrealloc(imu_t,cap*sizeof(*imu_t)); imu_g=xrealloc(imu_g,cap*sizeof(*imu_g)); imu_a=xrealloc(imu_a,cap*sizeof(*imu_a)); }
    } fclose(f);
    /* cam */
    snprintf(path,sizeof path,"%s/cam0/data.csv",mav0);
    f=fopen(path,"rb"); if(!f){perror(path);exit(1);}
    cap=8000; cam_t_ns=xrealloc(NULL,cap*sizeof(*cam_t_ns)); cam_files=xrealloc(NULL,cap*sizeof(*cam_files));
    n_cam=0; rd_line(f,line,sizeof line);
    while(rd_line(f,line,sizeof line)){
        char*t=strtok(line,",\r\n"); if(!t)continue;
        cam_t_ns[n_cam]=atoll(t);
        char*fn=strtok(NULL,",\r\n"); if(!fn){n_bad++;continue;}
        snprintf(cam_files[n_cam],64,"%s",fn);
        if(++n_cam>=cap){ cap*=2; cam_t_ns=xrealloc(cam_t_ns,cap*sizeof(*cam_t_ns)); cam_files=xrealloc(cam_files,cap*sizeof(*cam_files)); }
    } fclose(f);
    /* GT */
    snprintf(path,sizeof path,"%s/state_groundtruth_estimate0/data.csv",mav0);
    f=fopen(path,"rb"); if(!f){perror(path);exit(1);}
    cap=70000; gt_t_ns=xrealloc(NULL,cap*sizeof(*gt_t_ns)); gt_p=xrealloc(NULL,cap*sizeof(*gt_p));
    gt_q=xrealloc(NULL,cap*sizeof(*gt_q)); gt_v=xrealloc(NULL,cap*sizeof(*gt_v)); gt_bg=xrealloc(NULL,cap*sizeof(*gt_bg)); gt_ba=xrealloc(NULL,cap*sizeof(*gt_ba));
    n_gt=0; rd_line(f,line,sizeof line);
    while(rd_line(f,line,sizeof line)){
        char*t=strtok(line,",\r\n"); if(!t)continue;
        gt_t_ns[n_gt]=atoll(t);
        if(!tok_doubles(gt_p[n_gt],3) || !tok_doubles(gt_q[n_gt],4) ||   /* q = w,x,y,z */
           !tok_doubles(gt_v[n_gt],3) || !tok_doubles(gt_bg[n_gt],3) || !tok_doubles(gt_ba[n_gt],3)){ n_bad++; continue; }
        if(++n_gt>=cap){ cap*=2; gt_t_ns=xrealloc(gt_t_ns,cap*sizeof(*gt_t_ns)); gt_p=xrealloc(gt_p,cap*sizeof(*gt_p));
            gt_q=xrealloc(gt_q,cap*sizeof(*gt_q)); gt_v=xrealloc(gt_v,cap*sizeof(*gt_v)); gt_bg=xrealloc(gt_bg,cap*sizeof(*gt_bg)); gt_ba=xrealloc(gt_ba,cap*sizeof(*gt_ba)); }
    } fclose(f);
    fprintf(stderr,"loaded: imu=%d cam=%d gt=%d%s\n",n_imu,n_cam,n_gt, n_bad?" (malformed rows skipped)":"");
    if(n_bad) fprintf(stderr,"warn: %d malformed CSV rows skipped\n",n_bad);
}

static int buf_end(long long tgt){ int lo=0,hi=n_imu-1,r=-1; while(lo<=hi){int m=(lo+hi)/2; if(imu_t[m]<=tgt){r=m;lo=m+1;}else hi=m-1;} return r; }
/* binary search on monotonic gt_t_ns, then pick nearer neighbour */
static int gt_nearest(long long ts){
    int lo=0, hi=n_gt-1;
    while(lo<hi){ int m=(lo+hi)/2; if(gt_t_ns[m]<ts) lo=m+1; else hi=m; }
    if(lo>0){ long long dl=ts-gt_t_ns[lo-1], dc=gt_t_ns[lo]-ts; if(dl<0)dl=-dl; if(dc<0)dc=-dc; if(dl<dc) return lo-1; }
    return lo;
}

/* ─────────────────────────── track hash map (chaining) ─────────────── */
typedef struct HNode { int32_t fid; int idx; struct HNode *next; } HNode;
static HNode **hbuckets; static int hmask;
static vio_track_t **pool; static int pool_n, pool_cap;
static int *freelist; static int free_n;

static void tm_init(void){
    hmask = (1<<16)-1;
    hbuckets = calloc(hmask+1, sizeof(HNode*));
    pool_cap = 4096; pool = malloc(pool_cap*sizeof(*pool));
    freelist = malloc(pool_cap*sizeof(int)); free_n=0; pool_n=0;
}
static int tm_find(int32_t fid){
    HNode *h=hbuckets[(uint32_t)fid & hmask];
    while(h){ if(h->fid==fid) return h->idx; h=h->next; }
    return -1;
}
static int tm_create(int32_t fid){
    int idx;
    if(free_n>0){ idx=freelist[--free_n]; }
    else {
        if(pool_n>=pool_cap){ pool_cap*=2; pool=realloc(pool,pool_cap*sizeof(*pool)); freelist=realloc(freelist,pool_cap*sizeof(int)); }
        idx=pool_n++; pool[idx]=malloc(sizeof(vio_track_t));
    }
    pool[idx]->fid=fid; pool[idx]->n=0;
    HNode *h=malloc(sizeof(HNode)); h->fid=fid; h->idx=idx; h->next=hbuckets[(uint32_t)fid & hmask]; hbuckets[(uint32_t)fid & hmask]=h;
    return idx;
}
static void tm_remove(int32_t fid){
    uint32_t b=(uint32_t)fid & hmask; HNode **pp=&hbuckets[b];
    while(*pp){ if((*pp)->fid==fid){ HNode*d=*pp; int idx=d->idx; *pp=d->next; free(d);
            freelist[free_n++]=idx; return; } pp=&(*pp)->next; }
}

/* ─────────────────────────── rotation helpers ──────────────────────── */
static void rotvec_to_R(const double rv[3], double R[9])
{
    double th = sqrt(rv[0]*rv[0]+rv[1]*rv[1]+rv[2]*rv[2]);
    if (th < 1e-12) { memset(R,0,9*sizeof(double)); R[0]=R[4]=R[8]=1.0; return; }
    double r[3]={rv[0]/th,rv[1]/th,rv[2]/th};
    double c=cos(th), s=sin(th), cs=1.0-c;
    double sk[9]={0,-r[2],r[1], r[2],0,-r[0], -r[1],r[0],0};
    for(int i=0;i<9;i++) R[i]=cs*(r[i/3]*r[i%3]) - s*sk[i];
    R[0]+=c; R[4]+=c; R[8]+=c;
}

/* ─────────────────────────── clone eviction ────────────────────────── */
static void remove_oldest_clone_full(msckf_state_t *st, vio_snap_t *snap)
{
    int N=st->clones.N; if(N<=0) return; int cur=15+6*N;
    for(int r=21;r<cur;r++) for(int c=0;c<cur;c++) st->P[(r-6)*SZ+c]=st->P[r*SZ+c];
    int nr=cur-6;
    for(int c=21;c<cur;c++) for(int r=0;r<nr;r++) st->P[r*SZ+(c-6)]=st->P[r*SZ+c];
    for(int k=1;k<N;k++){
        st->clones.id[k-1]=st->clones.id[k]; st->clones.t[k-1]=st->clones.t[k];
        for(int j=0;j<4;j++){ st->clones.q_CG[4*(k-1)+j]=st->clones.q_CG[4*k+j]; snap->q_CG[4*(k-1)+j]=snap->q_CG[4*k+j]; }
        for(int j=0;j<3;j++){ st->clones.t_GC[3*(k-1)+j]=st->clones.t_GC[3*k+j]; snap->t_GC[3*(k-1)+j]=snap->t_GC[3*k+j]; }
    }
    st->clones.N=N-1;
}

static void symmetrize(msckf_state_t*st,int cur){
    for(int r=0;r<cur;r++)for(int c=r+1;c<cur;c++){ double s=0.5*(st->P[r*SZ+c]+st->P[c*SZ+r]); st->P[r*SZ+c]=s; st->P[c*SZ+r]=s; }
}

#if defined(N3_BARO_Z) || defined(N3_DEPTH_GATE) || defined(ID1_BARO_DZ) || defined(ADAPT_C2_DEPTH_PRIOR) || defined(C3_DIAG) || defined(KF2_HOVER_GOV) || defined(KF3_TERM_GOV)
/* [N3 family] baro series loaded once; anchor h0 = mean over the static init window. */
static int     n3_n = 0, n3_i = 0, n3_applied = 0, n3_rej = 0;
static double *n3_t = NULL, *n3_h = NULL;
static double  n3_h0 = 0.0, n3_last = -1e18;
double         g_n3_hagl = -1.0;   /* consumed by vio_backend depth gate (ZG) */
#ifdef ADAPT_C2_DEPTH_PRIOR
double         g_c2_w = 0.0;       /* [C2] scheduled ramp weight for the depth prior */
#endif
#ifdef C3_DIAG
double         g_c3_now = 0.0;     /* [C3_DIAG] state time for the track side channel */
#endif
#ifdef KF3_P2B_DIAG
/* [P2B PREREG 29b1550] guarded logging-only globals — no estimator effect (proof: traj md5 identical) */
double         g_p2b_now = 0.0;    /* state time for backend track/update rows */
static int     g_p2b_nprop = 0;    /* per-frame propagation accumulators */
static double  g_p2b_sumdt = 0.0, g_p2b_fsum[3] = {0, 0, 0}, g_p2b_imut = 0.0;
#endif
#ifdef KF2_HOVER_GOV
int            g_kf2_freeze = 0;
double         g_kf2_rampf = 1.0;
#endif
#ifdef KF3_TERM_GOV
double         g_kf3_f = 1.0;      /* [KF3] terminal R factor (ramped 1..F_TERM) */
#endif
#ifdef KF3_COV_DIAG
extern const char *g_covdiag_consumer;   /* [ROUND2B] defined in msckf_ekf.c; callers tag before update */
#endif
static void n3_load(double t0_s)
{
    const char *pth = getenv("VIO_BARO_CSV");
    if(!pth){ fprintf(stderr,"[N3] VIO_BARO_CSV not set - baro update disabled\n"); return; }
    FILE *bf = fopen(pth,"r");
    if(!bf){ fprintf(stderr,"[N3] cannot open %s\n",pth); return; }
    int cap = 1<<15;
    n3_t = malloc(sizeof(double)*cap); n3_h = malloc(sizeof(double)*cap);
    char line[256]; double a,b;
    while(fgets(line,sizeof line,bf)){
        if(line[0]=='#') continue;
        if(sscanf(line,"%lf,%lf",&a,&b)==2){
            if(n3_n==cap){ cap*=2;
                n3_t=realloc(n3_t,sizeof(double)*cap); n3_h=realloc(n3_h,sizeof(double)*cap); }
            n3_t[n3_n]=a; n3_h[n3_n]=b; n3_n++;
        }
    }
    fclose(bf);
    double s=0; int c=0;
    for(int i=0;i<n3_n;i++)
        if(n3_t[i]>=t0_s && n3_t[i]<t0_s+VIO_BARO_ANCHOR_S){ s+=n3_h[i]; c++; }
    if(c<5){ fprintf(stderr,"[N3] anchor window has %d samples - disabled\n",c); n3_n=0; return; }
    n3_h0 = s/c;
    fprintf(stderr,"[N3] baro n=%d span=[%.3f,%.3f] h0=%.3f (anchor %d samples)\n",
            n3_n, n3_t[0], n3_t[n3_n-1], n3_h0, c);
}
#endif

int main(int argc, char **argv)
{
    pin_to_core(PIN_CORE);   /* before any allocation/thread so everything runs on this core */
    const char *mav0 = (argc>1)?argv[1]:
        "/mnt/c/datasets/euroc/machine_hall/vio_run_20260316_092725_euroc_v2_5_quatfix__heart_loop_moving_entry_arc/mav0";
    const char *out_path = (argc>2)?argv[2]:"est_trajectory.csv";
    long long start_ts = (argc>3)?atoll(argv[3]):3177264630954707712LL;
#ifdef ZUPT_PROBE
    zupt_log_open();
#endif
    int ransac_seed_base = 42;

    msckf_calib_t calib; cam_intrinsics_t K;
    if(!load_calib(mav0,&calib,&K)){ fprintf(stderr,"calib load failed\n"); return 1; }
    fprintf(stderr,"K: fx=%g fy=%g cx=%g cy=%g dist=[%g %g %g %g]\n",K.fx,K.fy,K.cx,K.cy,K.k1,K.k2,K.p1,K.p2);
    load_data(mav0);

    int start_idx=-1; for(int i=0;i<n_cam;i++) if(cam_t_ns[i]>=start_ts){ start_idx=i; break; }
    if(start_idx<0){ fprintf(stderr,"no cam frame >= start_ts\n"); return 1; }
    int gi = gt_nearest(start_ts);
    fprintf(stderr,"start_idx=%d cam_ts=%lld gt_idx=%d\n",start_idx,cam_t_ns[start_idx],gi);

#if defined(N3_BARO_Z) || defined(N3_DEPTH_GATE) || defined(ID1_BARO_DZ)
    n3_load((double)cam_t_ns[start_idx]*1e-9);
#endif

    /* init state from GT */
    msckf_state_t st; memset(&st,0,sizeof st);
    st.q_GI[0]=gt_q[gi][1]; st.q_GI[1]=gt_q[gi][2]; st.q_GI[2]=gt_q[gi][3]; st.q_GI[3]=gt_q[gi][0]; /* JPL x,y,z,w */
    for(int k=0;k<3;k++){ st.p_GI[k]=gt_p[gi][k]; st.v_GI[k]=gt_v[gi][k]; st.bg[k]=gt_bg[gi][k]; st.ba[k]=gt_ba[gi][k]; }
    st.t = (double)cam_t_ns[start_idx]*1e-9;
    st.clones.N=0;
    double idiag[15]={1e-5,1e-5,1e-5,1e-12,1e-12,1e-12,1e-4,1e-4,1e-4,1e-4,1e-4,1e-4,1e-4,1e-4,1e-4};
    for(int i=0;i<15;i++) st.P[i*SZ+i]=idiag[i];

#ifdef A1_IMU_SPEC
    /* [A1] measured post-LPF stochastic model (PREREG_A1 formulas; conservative-of-two-flights).
     * White down 0.13->0.052, bias-RW up 0.003->0.064 (x21 - matches N5's "too low ~20x"):
     * vibration quasi-DC wander moves from fake white noise into the estimable bias states. */
    msckf_noise_t noise={.sigma_g=0.000705,.sigma_a=0.051875,.sigma_bg=0.0002,.sigma_ba=0.064299};
#else
    msckf_noise_t noise={.sigma_g=0.002,.sigma_a=0.13 /* SCALE-PROBE: measured in-flight vibration, cruise worst-axis fdRMS 1.835/sqrt(197.6); was 0.03 */,.sigma_bg=0.0002,.sigma_ba=0.003 /* PROBE: real-IMU ballpark */};
#endif
    const real_t g[3]={0.0,0.0,-9.81};

    vio_snap_t snap; memset(&snap,0,sizeof snap);
    vio_stats_t stats; memset(&stats,0,sizeof stats);
    tm_init();

    /* trigger state */
    int trans_fired=0, baz_asc_fired=0, baz_sustain=0; double prev_vbx=0.0;

    /* frontend persistent state */
    int imu_ptr = 0;
    float *prev_img=NULL; int W=0,H=0;
    const float *klt_last_curr=NULL;  /* curr_img of the last klt_tracker_track call (buffer kept alive as prev_img) */
    double (*prev_pts)[2]=malloc(4096*sizeof(*prev_pts)); int n_prev=0;
    int32_t *feat_ids=malloc(4096*sizeof(int32_t));
    int32_t next_feat_id=0, clone_id=0;

    corner_t *corners = malloc(20000*sizeof(corner_t));
    int *keep_idx = malloc(20000*sizeof(int));

    double (*est_p)[3]=calloc(n_cam,sizeof(*est_p)); int *est_has=calloc(n_cam,sizeof(int));

    for(int cam_idx=start_idx; cam_idx<n_cam; cam_idx++){
        PROF_TIC(t_frame);
        double t_cam = (double)cam_t_ns[cam_idx]*1e-9;
        int imu_buf_end = buf_end(cam_t_ns[cam_idx]);
        if(imu_buf_end<0) continue;

        char ipath[1200]; snprintf(ipath,sizeof ipath,"%s/cam0/data/%s",mav0,cam_files[cam_idx]);
        PROF_TIC(t_png);
        int iw,ih; float *img = png_load_gray(ipath,&iw,&ih);
        PROF_TOC(PROF_PNG, t_png);
        if(!img){ fprintf(stderr,"img load fail %s\n",ipath); continue; }
        if(!W){ W=iw; H=ih; }

        if(cam_idx==start_idx){
            PROF_TIC(t_det0);
            int nc = vio_detect_min_eigen(img,W,H,VIO_DETECT_MIN_QUALITY,corners,20000);
            int nk = vio_select_grid(corners,nc,NULL,0,VIO_NUM_FEATURES,W,H,keep_idx);
            PROF_TOC(PROF_DETECT, t_det0);
            n_prev=0;
            for(int i=0;i<nk;i++){ prev_pts[n_prev][0]=corners[keep_idx[i]].u; prev_pts[n_prev][1]=corners[keep_idx[i]].v; feat_ids[n_prev]=next_feat_id++; n_prev++; }
            prev_img=img; if(imu_buf_end>imu_ptr) imu_ptr=imu_buf_end;
            PROF_TOC(PROF_FRAME, t_frame);
            vio_prof_frame_commit(cam_idx-start_idx, t_cam, n_prev);
            continue;
        }

        /* build imu rotation segment (imu_ptr..imu_buf_end-1) */
        PROF_TIC(t_imu);
        double imu2_R_imu1[9]={1,0,0, 0,1,0, 0,0,1};
        int seg_count=0;
        for(int ii=imu_ptr; ii<imu_buf_end; ii++){
            if(ii+1>=n_imu) break;
            double dt=(double)(imu_t[ii+1]-imu_t[ii])*1e-9; if(dt<=0||dt>0.1) continue;
            double rv[3]={imu_g[ii][0]*dt,imu_g[ii][1]*dt,imu_g[ii][2]*dt};
            double dR[9]; rotvec_to_R(rv,dR);
            double tmp[9]; mat3x3_mul(dR,imu2_R_imu1,tmp); memcpy(imu2_R_imu1,tmp,sizeof tmp);
            seg_count++;
        }

        /* IMU propagation */
#ifdef KF3_P2B_DIAG
        g_p2b_nprop = 0; g_p2b_sumdt = 0.0; g_p2b_fsum[0] = g_p2b_fsum[1] = g_p2b_fsum[2] = 0.0;
#endif
        for(int ii=imu_ptr; ii<imu_buf_end; ii++){
            if(ii+1>=n_imu) break;
            double dt=(double)(imu_t[ii+1]-imu_t[ii])*1e-9; if(dt<=0||dt>0.1) continue;
            msckf_imu_t im; for(int k=0;k<3;k++){im.gyro[k]=imu_g[ii][k];im.acc[k]=imu_a[ii][k];} im.dt=dt;
            msckf_step_imu(&st,&im,&noise,g);
#ifdef KF3_P2B_DIAG
            g_p2b_nprop++; g_p2b_sumdt += dt;
            for(int k=0;k<3;k++) g_p2b_fsum[k] += imu_a[ii][k] - (double)st.ba[k];
#endif
        }
        imu_ptr=imu_buf_end;
#ifdef KF3_P2B_DIAG
        if (imu_buf_end > 0 && imu_buf_end <= n_imu) g_p2b_imut = (double)imu_t[imu_buf_end-1]*1e-9;
#endif
        PROF_TOC(PROF_IMU, t_imu);

#ifdef ZUPT_PROBE
        /* ── [ZUPT PROBE] velocity-only zero-velocity update ───────────────────
         * Placed AFTER IMU propagation and BEFORE the trigger check (Prof lock):
         * on manual the transition trigger fires 29 s pre-takeoff off the filter's
         * own drifting velocity, so the brake must be applied before it is read.
         * Detector = the frozen MATLAB Stage-A rule: three conditions on the last
         * VIO_ZUPT_WINDOW_N IMU samples, all of which must hold.
         * NIS is computed BEFORE the update is applied (Prof lock) — for a velocity
         * observation H is [0 0 I 0 0 | 0...], so S = P[6:9,6:9] + sigma_v^2 I and no
         * matrix product is needed. Innovation reached 26-74 m/s in the MATLAB Phase-A
         * work when fired at an already-diverged state, hence the gate is mandatory. */
        {
            double gs = -1.0, as = -1.0, am = -1.0;
            int    stationary = 0, zupt_applied = 0, gate_pass = -1;
            double nis_z = -1.0, v_before = 0.0, v_after = 0.0;
            int    w0 = imu_buf_end - VIO_ZUPT_WINDOW_N;

            if (w0 >= 0 && imu_buf_end <= n_imu) {
                double gn[VIO_ZUPT_WINDOW_N], an[VIO_ZUPT_WINDOW_N], amean[3] = {0,0,0};
                double gsum = 0.0, asum = 0.0;
                for (int i = 0; i < VIO_ZUPT_WINDOW_N; i++) {
                    const double *gv = imu_g[w0+i], *av = imu_a[w0+i];
                    gn[i] = sqrt(gv[0]*gv[0]+gv[1]*gv[1]+gv[2]*gv[2]);
                    an[i] = sqrt(av[0]*av[0]+av[1]*av[1]+av[2]*av[2]);
                    gsum += gn[i]; asum += an[i];
                    for (int k3 = 0; k3 < 3; k3++) amean[k3] += av[k3];
                }
                double gm = gsum/VIO_ZUPT_WINDOW_N, amn = asum/VIO_ZUPT_WINDOW_N;
                double gv2 = 0.0, av2 = 0.0;
                for (int i = 0; i < VIO_ZUPT_WINDOW_N; i++) {
                    gv2 += (gn[i]-gm)*(gn[i]-gm);
                    av2 += (an[i]-amn)*(an[i]-amn);
                }
                /* sample std (N-1), matching MATLAB std() */
                gs = sqrt(gv2/(VIO_ZUPT_WINDOW_N-1));
                as = sqrt(av2/(VIO_ZUPT_WINDOW_N-1));
                for (int k3 = 0; k3 < 3; k3++) amean[k3] /= VIO_ZUPT_WINDOW_N;
                am = sqrt(amean[0]*amean[0]+amean[1]*amean[1]+amean[2]*amean[2]);

                stationary = (gs < VIO_ZUPT_GYRO_STD_MAX) &&
                             (as < VIO_ZUPT_ACCEL_STD_MAX) &&
                             (fabs(am - VIO_ZUPT_GRAVITY) < VIO_ZUPT_GRAVITY_TOL);
            }

            v_before = sqrt(st.v_GI[0]*st.v_GI[0]+st.v_GI[1]*st.v_GI[1]+st.v_GI[2]*st.v_GI[2]);
            v_after  = v_before;

            if (stationary) {
                int Nc = st.clones.N;
                int nst = MSCKF_IMU_SZ + Nc*MSCKF_CLONE_SZ;
                int Psz = MSCKF_MAX_STATE_SZ;
                double r3[3] = { -st.v_GI[0], -st.v_GI[1], -st.v_GI[2] };
                double rn = VIO_ZUPT_VEL_SIGMA*VIO_ZUPT_VEL_SIGMA;
                /* S = P[6:9,6:9] + sigma_v^2 I  (velocity error-state block) */
                double S3[9];
                for (int a3 = 0; a3 < 3; a3++)
                    for (int b3 = 0; b3 < 3; b3++)
                        S3[a3*3+b3] = st.P[(6+a3)*Psz + (6+b3)] + (a3==b3 ? rn : 0.0);
                /* explicit 3x3 inverse; NIS = r' S^-1 r */
                double d = S3[0]*(S3[4]*S3[8]-S3[5]*S3[7])
                         - S3[1]*(S3[3]*S3[8]-S3[5]*S3[6])
                         + S3[2]*(S3[3]*S3[7]-S3[4]*S3[6]);
                if (fabs(d) > 1e-300) {
                    double inv[9];
                    inv[0]= (S3[4]*S3[8]-S3[5]*S3[7])/d; inv[1]=-(S3[1]*S3[8]-S3[2]*S3[7])/d; inv[2]= (S3[1]*S3[5]-S3[2]*S3[4])/d;
                    inv[3]=-(S3[3]*S3[8]-S3[5]*S3[6])/d; inv[4]= (S3[0]*S3[8]-S3[2]*S3[6])/d; inv[5]=-(S3[0]*S3[5]-S3[2]*S3[3])/d;
                    inv[6]= (S3[3]*S3[7]-S3[4]*S3[6])/d; inv[7]=-(S3[0]*S3[7]-S3[1]*S3[6])/d; inv[8]= (S3[0]*S3[4]-S3[1]*S3[3])/d;
                    nis_z = 0.0;
                    for (int a3 = 0; a3 < 3; a3++)
                        for (int b3 = 0; b3 < 3; b3++)
                            nis_z += r3[a3]*inv[a3*3+b3]*r3[b3];
                    static double zupt_thr = -1.0;
                    if (zupt_thr < 0.0) zupt_thr = vio_chi2inv(VIO_ZUPT_CHI2_P, 3);
                    gate_pass = (nis_z <= zupt_thr);
                    if (gate_pass) {
                        static real_t Hz[3*MSCKF_MAX_STATE_SZ];
                        memset(Hz, 0, sizeof(real_t)*3*nst);
                        Hz[0*nst + 6] = 1.0; Hz[1*nst + 7] = 1.0; Hz[2*nst + 8] = 1.0;
                        real_t rz[3] = { (real_t)r3[0], (real_t)r3[1], (real_t)r3[2] };
                        real_t nis_out = 0;
#ifdef KF3_COV_DIAG
                        g_covdiag_consumer = "ZUPT";
#endif
                        if (msckf_update_ekf(&st, rz, Hz, (real_t)rn, 3, &nis_out)) zupt_applied = 1;
                        v_after = sqrt(st.v_GI[0]*st.v_GI[0]+st.v_GI[1]*st.v_GI[1]+st.v_GI[2]*st.v_GI[2]);
                    }
                }
            }

            if (zupt_log) {
                fprintf(zupt_log, "%.9f,%.6g,%.6g,%.6g,%d,%.6g,%d,%d,%.6g,%.6g\n",
                        st.t, gs, as, am, stationary, nis_z, gate_pass, zupt_applied,
                        v_before, v_after);
            }
        }
#endif

#if defined(N3_BARO_Z) || defined(N3_DEPTH_GATE) || defined(ID1_BARO_DZ) || defined(ADAPT_C2_DEPTH_PRIOR) || defined(C3_DIAG) || defined(KF2_HOVER_GOV) || defined(KF3_TERM_GOV)
        /* [N3 family] advance baro cursor; publish current AGL for the ZG depth gate. */
        if (n3_n) {
            while (n3_i+1 < n3_n && n3_t[n3_i+1] <= st.t) n3_i++;
            if (n3_t[n3_i] <= st.t) g_n3_hagl = n3_h[n3_i] - n3_h0;
#ifdef C3_DIAG
            g_c3_now = st.t;
#endif
#ifdef KF2_HOVER_GOV
            /* [KF2] hover state machine (PREREG_KF2A): ENTER |v_h|<V_IN sustained T_IN
             * AND |baro rate|<BR_MAX · EXIT |v_h|>V_OUT sustained T_OUT -> R-ramp.
             * baro absent => brate stays huge => never freezes (safe default). */
            {
                static double kf2_prev_h = 0.0, kf2_prev_t = -1.0, kf2_brate = 1e9;
                static double kf2_lo_since = -1.0, kf2_hi_since = -1.0, kf2_exit_t = -1e18;
                if (kf2_prev_t < 0.0) { kf2_prev_t = st.t; kf2_prev_h = g_n3_hagl; }
                if (st.t - kf2_prev_t >= 1.0) {
                    kf2_brate = (g_n3_hagl - kf2_prev_h) / (st.t - kf2_prev_t);
                    kf2_prev_t = st.t; kf2_prev_h = g_n3_hagl;
                }
                double kf2_vh = sqrt(st.v_GI[0]*st.v_GI[0] + st.v_GI[1]*st.v_GI[1]);
                if (!g_kf2_freeze) {
                    /* [KF2b] ENTER เฉพาะกลางอากาศ (AGL gate — กัน ground/takeoff) */
                    if (kf2_vh < KF2_V_IN && fabs(kf2_brate) < KF2_BR_MAX
                        && g_n3_hagl > KF2_AGL_MIN) {
                        if (kf2_lo_since < 0.0) kf2_lo_since = st.t;
                        if (st.t - kf2_lo_since >= KF2_T_IN) {
                            g_kf2_freeze = 1; kf2_hi_since = -1.0;
#ifdef KF2_DIAG
                            fprintf(stderr,"[KF2] ENTER hover t=%.3f vh=%.2f brate=%.2f agl=%.1f\n",st.t,kf2_vh,kf2_brate,g_n3_hagl);
#endif
                        }
                    } else kf2_lo_since = -1.0;
                } else {
                    /* [KF2b] ไต่/ลงแนวดิ่ง = ไม่ใช่ hover -> EXIT ทันที ไม่รอ sustain */
                    if (fabs(kf2_brate) > KF2_BR_EXIT) {
                        g_kf2_freeze = 0; kf2_lo_since = -1.0; kf2_exit_t = st.t;
#ifdef KF2_DIAG
                        fprintf(stderr,"[KF2] EXIT hover (baro-rate) t=%.3f brate=%.2f (suppressed=%ld)\n",st.t,kf2_brate,g_kf2_suppressed);
#endif
                    } else if (kf2_vh > KF2_V_OUT) {
                        if (kf2_hi_since < 0.0) kf2_hi_since = st.t;
                        if (st.t - kf2_hi_since >= KF2_T_OUT) {
                            g_kf2_freeze = 0; kf2_lo_since = -1.0; kf2_exit_t = st.t;
#ifdef KF2_DIAG
                            fprintf(stderr,"[KF2] EXIT hover t=%.3f (suppressed=%ld)\n",st.t,g_kf2_suppressed);
#endif
                        }
                    } else kf2_hi_since = -1.0;
                }
                double kf2_dtr = st.t - kf2_exit_t;
                g_kf2_rampf = (!g_kf2_freeze && kf2_dtr < KF2_RAMP_S)
                    ? (KF2_RAMP_F0 + (1.0 - KF2_RAMP_F0) * (kf2_dtr / KF2_RAMP_S)) : 1.0;
            }
#endif
#ifdef KF3_TERM_GOV
            /* [KF3] terminal state machine (PREREG_KF3): ENTER = AGL>20 + v_h low sustained;
             * descent STAYS terminal (no baro-rate exit); EXIT = v_h high sustained or AGL<=20.
             * Factor ramps linearly 1<->F_TERM both directions — never switches abruptly. */
            {
                static int kf3_term = 0;
                static double kf3_lo = -1.0, kf3_hi = -1.0, kf3_last = -1.0;
                static double kf3_ph = 0.0, kf3_pt = -1.0, kf3_brate = 0.0;
#if defined(KF3_GOV_FAULT_INJECT) || defined(BARO_STALE_SYSWIDE)
                /* [DIAG] governor-only baro view (gov_agl) + stale signal (gov_stale).
                 * Slice A fault-inject: freeze gov_agl in env window (ID1/N3 keep real n3_h[n3_i]).
                 * Slice B syswide: gov_stale from real current-sample age. */
                double gov_agl = g_n3_hagl; int gov_stale = 0;
#  ifdef KF3_GOV_FAULT_INJECT
                { static int fi = 0; static double fw0 = -1, fw1 = -1, pre_agl = 0;
                  if (!fi) { const char *e = getenv("KF3_GOV_FAULT_WIN"); if (e) sscanf(e, "%lf,%lf", &fw0, &fw1); fi = 1; }
                  if (fw0 >= 0 && st.t < fw0) pre_agl = g_n3_hagl;          /* last trusted AGL before fault */
                  if (fw0 >= 0 && st.t >= fw0 && st.t <= fw1) { gov_agl = pre_agl; gov_stale = 1; } }
#  endif
#  ifdef BARO_STALE_SYSWIDE
                if (n3_n == 0 || (st.t - n3_t[n3_i]) >= KF3_BARO_FRESH_MAX_S) gov_stale = 1;
#  endif
#  define KF3_GOV_AGL gov_agl
#else
#  define KF3_GOV_AGL g_n3_hagl
#endif
                if (kf3_pt < 0.0) { kf3_pt = st.t; kf3_ph = KF3_GOV_AGL; }
                if (st.t - kf3_pt >= 1.0) {
                    kf3_brate = (KF3_GOV_AGL - kf3_ph) / (st.t - kf3_pt);
                    kf3_pt = st.t; kf3_ph = KF3_GOV_AGL;
                }
                double kf3_vh = sqrt(st.v_GI[0]*st.v_GI[0] + st.v_GI[1]*st.v_GI[1]);
#ifdef KF3_BARO_FRESH_FAILSAFE
                /* [KF3d ROUND1] baro-freshness fail-safe: a stalled baro stream must not
                 * latch the terminal governor (age-blind AGL/brate = stuck g_kf3_f). */
                static double kf3_stale_since = -1.0;
                double kf3_baro_age = (n3_n > 0) ? (st.t - n3_t[n3_i]) : -1.0;
                int kf3_fresh_ok = (n3_n > 0) && (kf3_baro_age < KF3_BARO_FRESH_MAX_S);
                if (!kf3_fresh_ok) {
                    if (kf3_stale_since < 0.0) {   /* first stale frame of this episode */
                        kf3_stale_since = st.t;
                        fprintf(stderr,"[KF3_BARO] stale-detect t=%.3f age=%.3f agl=%.1f brate=%.2f vh=%.2f term=%d\n",
                                st.t, kf3_baro_age, g_n3_hagl, kf3_brate, kf3_vh, kf3_term);
                    }
                } else kf3_stale_since = -1.0;
#endif
                if (!kf3_term) {
                    int kf3_br_ok = 1;
#ifdef KF3_ENTER_BRGATE
                    kf3_br_ok = fabs(kf3_brate) < KF3_BR_ENTER;   /* [KF3c] hold only, not climb/descent */
#endif
                    if (
#ifdef KF3_BARO_FRESH_FAILSAFE
                        kf3_fresh_ok &&   /* [KF3d] stale baro must not ENTER terminal */
#endif
#if defined(KF3_STALE_POLICY) || defined(BARO_STALE_SYSWIDE)
                        !gov_stale &&     /* [DIAG A2/B2] stale governor view must not ENTER terminal */
#endif
                        KF3_GOV_AGL > KF3_AGL_MIN && kf3_vh < KF3_V_IN && kf3_br_ok) {
                        if (kf3_lo < 0.0) kf3_lo = st.t;
                        if (st.t - kf3_lo >= KF3_T_IN) {
                            kf3_term = 1; kf3_hi = -1.0;
#ifdef KF3_DIAG
                            fprintf(stderr,"[KF3] TERMINAL enter t=%.3f vh=%.2f brate=%.2f agl=%.1f\n",
                                    st.t, kf3_vh, kf3_brate, g_n3_hagl);
#endif
                        }
                    } else kf3_lo = -1.0;
                } else {
#ifdef KF3_BARO_FRESH_FAILSAFE
                    /* [KF3d] stalled baro while latched terminal: after GRACE, force EXIT so the
                     * existing ramp walks g_kf3_f back to 1 (no abrupt switch). */
                    if (kf3_stale_since >= 0.0 && st.t - kf3_stale_since >= KF3_BARO_GRACE_S) {
                        kf3_term = 0; kf3_lo = -1.0;
                        fprintf(stderr,"[KF3_BARO] stale-terminal-exit t=%.3f age=%.3f stale_for=%.2f agl=%.1f g_kf3_f=%.3f\n",
                                st.t, kf3_baro_age, st.t - kf3_stale_since, g_n3_hagl, g_kf3_f);
                    } else
#endif
#if defined(KF3_STALE_POLICY) || defined(BARO_STALE_SYSWIDE)
                    /* [DIAG A2/B2] never exit on a stale AGL/rate (Prof: no stale-driven state change). */
                    if (gov_stale) { /* hold last trusted state */ }
                    else
#endif
                    if (KF3_GOV_AGL <= KF3_AGL_MIN) {
                        kf3_term = 0; kf3_lo = -1.0;
#ifdef KF3_DIAG
                        fprintf(stderr,"[KF3] TERMINAL exit (AGL) t=%.3f agl=%.1f\n", st.t, g_n3_hagl);
#endif
                    }
#ifndef KF3_LATCH
                    /* [KF3a] v_h-exit — REMOVED in KF3b latch (state vh หลอกได้กลาง descent) */
                    else if (kf3_vh > KF3_V_OUT) {
                        if (kf3_hi < 0.0) kf3_hi = st.t;
                        if (st.t - kf3_hi >= KF3_T_OUT) {
                            kf3_term = 0; kf3_lo = -1.0;
#ifdef KF3_DIAG
                            fprintf(stderr,"[KF3] TERMINAL exit (speed) t=%.3f vh=%.2f\n", st.t, kf3_vh);
#endif
                        }
                    } else kf3_hi = -1.0;
#endif
                }
                double kf3_target = kf3_term ? KF3_F_TERM : 1.0;
                double kf3_dt = (kf3_last < 0.0) ? 0.0 : (st.t - kf3_last);
                kf3_last = st.t;
                double kf3_rate = (KF3_F_TERM - 1.0) / KF3_RAMP_S * kf3_dt;
#if defined(KF3_STALE_POLICY) || defined(BARO_STALE_SYSWIDE)
                /* [DIAG A2/B2] hold-last-trusted: g_hold = g_kf3_f at last GOOD frame (age<FRESH_MAX);
                 * while STALE hold it constant (no ramp). Conservative placeholder — stale-after-latch UNRESOLVED. */
                {
                    static double g_hold = 1.0;
                    if (!gov_stale) {
                        if (g_kf3_f < kf3_target) { g_kf3_f += kf3_rate; if (g_kf3_f > kf3_target) g_kf3_f = kf3_target; }
                        else if (g_kf3_f > kf3_target) { g_kf3_f -= kf3_rate; if (g_kf3_f < kf3_target) g_kf3_f = kf3_target; }
                        g_hold = g_kf3_f;
                    } else {
                        g_kf3_f = g_hold;
                    }
                }
#else
                if (g_kf3_f < kf3_target) { g_kf3_f += kf3_rate; if (g_kf3_f > kf3_target) g_kf3_f = kf3_target; }
                else if (g_kf3_f > kf3_target) { g_kf3_f -= kf3_rate; if (g_kf3_f < kf3_target) g_kf3_f = kf3_target; }
#endif
#undef KF3_GOV_AGL
            }
#endif
#ifdef KF3_BARO_AUDIT
            /* [DIAG audit] per-frame baro-consumer view (isolation proof: cursor/sample/age are
             * frame+baro driven -> identical across A0/A1/A2; g_kf3_f is the policy output). Needs KF3_TERM_GOV. */
            {
                static FILE *g_baudit = NULL;
                if (!g_baudit) { const char *p = getenv("VIO_BARO_AUDIT_CSV");
                    if (p) { g_baudit = fopen(p, "w");
                        if (g_baudit) fprintf(g_baudit, "t,cursor_idx,sample_t,sample_val,age,g_kf3_f,baro_health\n"); } }
                if (g_baudit) {
                    double aage = (n3_n > 0) ? (st.t - n3_t[n3_i]) : -1.0;
                    const char *hh = (n3_n > 0 && aage < KF3_BARO_FRESH_MAX_S) ? "GOOD" : "STALE";
                    fprintf(g_baudit, "%.6f,%d,%.6f,%.6f,%.4f,%.4f,%s\n",
                            st.t, n3_i, (n3_n ? n3_t[n3_i] : 0.0), (n3_n ? n3_h[n3_i] : 0.0), aage, g_kf3_f, hh);
                }
            }
#endif
#ifdef ADAPT_C2_DEPTH_PRIOR
            /* [C2] scheduling: gates (i) AGL + (ii) freshness -> (iii) linear ramp.
             * Any gate fails => weight 0 and the ramp restarts (prereg: below gates
             * the prior is fully off; hover/ground never touched). */
            {
                static double c2_ok_since = -1.0;
                int c2_ok = g_n3_hagl > C2_AGL_MIN_M &&
                            (st.t - n3_t[n3_i]) < C2_FRESH_MAX_S;
                if (c2_ok) {
                    if (c2_ok_since < 0.0) c2_ok_since = st.t;
                    double w = (st.t - c2_ok_since) / C2_RAMP_S;
                    g_c2_w = w > 1.0 ? 1.0 : w;
                } else { c2_ok_since = -1.0; g_c2_w = 0.0; }
            }
#endif
#ifdef ID1_BARO_DZ
            /* [ID1] baro delta vs oldest clone (bias-immune; see vio.h note). */
#ifdef BARO_STALE_SYSWIDE
            /* [B2] skip the ID1 dz update when the CURRENT baro sample is stale (no stale reuse). */
            if (!(n3_n == 0 || (st.t - n3_t[n3_i]) >= KF3_BARO_FRESH_MAX_S))
#endif
            if (st.clones.N >= 2 && n3_t[n3_i] <= st.t && st.t - n3_last >= ID1_DZ_MIN_DT) {
                int k0 = 0;
                for (int c2 = 1; c2 < st.clones.N; c2++)
                    if (st.clones.t[c2] < st.clones.t[k0]) k0 = c2;
                double tk = st.clones.t[k0];
                int lo = 0, hi = n3_n - 1;
                while (lo < hi) { int mid = (lo + hi + 1) / 2;
                    if (n3_t[mid] <= tk) lo = mid; else hi = mid - 1; }
                if (n3_t[lo] <= tk && tk - n3_t[lo] < 0.2) {
                    double dz_meas = n3_h[n3_i] - n3_h[lo];
                    double dz_est  = st.p_GI[2] - st.clones.t_GC[3*k0 + 2];
                    double innov_d = dz_meas - dz_est;
                    int Ncd = st.clones.N;
                    int nstd = MSCKF_IMU_SZ + Ncd*MSCKF_CLONE_SZ;
                    static real_t Hd[MSCKF_MAX_STATE_SZ];
                    memset(Hd, 0, sizeof(real_t)*nstd);
                    Hd[5] = 1.0;
                    Hd[MSCKF_IMU_SZ + k0*MSCKF_CLONE_SZ + 5] = -1.0;
                    real_t rd[1] = { (real_t)innov_d };
                    real_t nis_d = 0;
#ifdef KF3_COV_DIAG
                    g_covdiag_consumer = "ID1_DZ";
#endif
                    if (msckf_update_ekf(&st, rd, Hd,
                            (real_t)(ID1_DZ_SIGMA_M*ID1_DZ_SIGMA_M), 1, &nis_d)) {
                        n3_applied++; n3_last = st.t;
                    }
                }
            }
#endif
#ifdef N3_BARO_Z
            /* [N3 BARO-Z] 1-DoF metric-height update on position-z (error-state idx 5).
             * No chi2 gate by design (see vio.h note): anchor must act on a diverged state. */
            if (n3_t[n3_i] <= st.t && st.t - n3_last >= VIO_BARO_MIN_DT) {
                double innov = (n3_h[n3_i] - n3_h0) - st.p_GI[2];
#ifdef N3_BARO_V2
                /* bias LPF: innovation used = raw - b_hat; b_hat tracks the slow part */
                static double n3_bias = 0.0;
                double dt_b = (n3_last > -1e17) ? (st.t - n3_last) : VIO_BARO_MIN_DT;
                innov -= n3_bias;
#endif
                if (fabs(innov) <= VIO_BARO_SANITY_M) {
                    int Ncb = st.clones.N;
                    int nstb = MSCKF_IMU_SZ + Ncb*MSCKF_CLONE_SZ;
                    static real_t Hb[MSCKF_MAX_STATE_SZ];
                    memset(Hb, 0, sizeof(real_t)*nstb);
                    Hb[5] = 1.0;
                    real_t rb[1] = { (real_t)innov };
                    real_t nis_b = 0;
                    double Rb = VIO_BARO_SIGMA_M*VIO_BARO_SIGMA_M;
#ifdef N3_BARO_V2
                    /* Huber-style bounded influence: linear R inflation beyond k */
                    if (fabs(innov) > VIO_BARO_HUBER_K)
                        Rb *= fabs(innov)/VIO_BARO_HUBER_K;
#endif
#ifdef KF3_COV_DIAG
                    g_covdiag_consumer = "N3_BARO";
#endif
                    if (msckf_update_ekf(&st, rb, Hb, (real_t)Rb, 1, &nis_b)) {
                        n3_applied++; n3_last = st.t;
#ifdef N3_BARO_V2
                        n3_bias += (dt_b/VIO_BARO_BIAS_TAU) * innov;
#endif
                    }
                } else n3_rej++;
            }
#endif
        }
#endif

        /* triggers */
        int cur=15+6*st.clones.N;
        double R_GI[9]; jpl_quat_to_rotm(st.q_GI,R_GI);
        double vb[3]; mat3x3_vec3(R_GI,st.v_GI,vb);
        double vbx=vb[0], vz=st.v_GI[2];
        int would = (prev_vbx<VIO_TRANS_FWD_THRESH && vbx>=VIO_TRANS_FWD_THRESH && fabs(vz)>=VIO_TRANS_VZ_MIN);
        if(!trans_fired && would){
            /* SCOPE: baz inflation hardcoded always-on (v81 config: baz.enable=true).
             * MATLAB gates this behind ENABLE_BAZ_REGIME — add a flag before reusing this
             * driver for a non-v81 tune. Same applies to vertical-balance in vio_detect. */
#ifndef ABLATE_TRIGGERS
            st.P[8*SZ+8]+=VIO_TRANS_INFLATE_VAR;
            st.P[14*SZ+14]+=VIO_BAZ_INFLATE_VAR;
            symmetrize(&st,cur);
#endif
            /* [ABLATE 2026-07-23] Prof-authorised diagnostic probe, NOT a promote.
             * These are AirSim-v81 driver leftovers: MATLAB gates them behind
             * ENABLE_BAZ_REGIME, the C port fires them unconditionally (comment above).
             * trans_fired is still set in both arms so the trigger stays one-shot and
             * the control flow is otherwise identical. */
            fprintf(stderr, "[TRIG] trans/baz trigger at t=%.6f  vbx=%.4f vz=%.4f%s\n",
                    st.t, vbx, vz,
#ifdef ABLATE_TRIGGERS
                    "  (ABLATED: no P injection)"
#else
                    "  (injected +4.0 on v_z, +0.002 on ba_z)"
#endif
                    );
            trans_fired=1;
        }
        prev_vbx=vbx;
        if(!baz_asc_fired && !trans_fired){
            if(vz>=VIO_BAZ_ASC_VZ_THRESH) baz_sustain++; else baz_sustain=0;
            if(baz_sustain>=VIO_BAZ_ASC_SUSTAIN){
#ifndef ABLATE_TRIGGERS
                st.P[14*SZ+14]+=VIO_BAZ_ASC_INFLATE_VAR; symmetrize(&st,cur);
#endif
                fprintf(stderr, "[TRIG] baz-ascent trigger at t=%.6f  vz=%.4f%s\n", st.t, vz,
#ifdef ABLATE_TRIGGERS
                        "  (ABLATED)"
#else
                        "  (injected)"
#endif
                        );
                baz_asc_fired=1; }
        }

        /* KLT track */
        double (*tracked_pts)[2]=malloc((VIO_NUM_FEATURES*2+n_prev+16)*sizeof(*tracked_pts));
        int32_t *tracked_ids=malloc((VIO_NUM_FEATURES*2+n_prev+16)*sizeof(int32_t));
        int n_tr=0;
        if(n_prev<=0) klt_last_curr=NULL;  /* chain broken: buffer may be freed+reused, cached pyramid untrusted */
        if(n_prev>0){
            PROF_TIC(t_klt);
            klt_point_t *pp=malloc(n_prev*sizeof(klt_point_t));
            klt_point_t *np=malloc(n_prev*sizeof(klt_point_t));
            uint8_t *val=malloc(n_prev);
            for(int i=0;i<n_prev;i++){ pp[i].u=(float)prev_pts[i][0]; pp[i].v=(float)prev_pts[i][1]; }
            memset(val,0,n_prev); memset(np,0,n_prev*sizeof(klt_point_t));   /* failure degrades to "no tracks", never uninitialized reads */
            int klt_reuse = (klt_last_curr!=NULL && prev_img==klt_last_curr);
            if(!klt_tracker_track(prev_img,img,W,H,pp,np,val,n_prev,klt_reuse))
                fprintf(stderr,"warn: klt_tracker_track failed (W=%d H=%d > compiled max %dx%d?) — frame treated as all-lost\n",W,H,KLT_MAX_W,KLT_MAX_H);
            klt_last_curr = img;
            /* keep valid + in bounds */
            double (*pv)[2]=malloc(n_prev*sizeof(*pv));  /* prev_pts_valid */
            int nk=0;
            for(int i=0;i<n_prev;i++){
                double u=np[i].u, v=np[i].v;
                if(val[i] && u>10 && u<W-10 && v>10 && v<H-10 && undist_in_range(&K,u,v)){ /* PROBE2 */
                    tracked_pts[nk][0]=u; tracked_pts[nk][1]=v; tracked_ids[nk]=feat_ids[i];
                    pv[nk][0]=prev_pts[i][0]; pv[nk][1]=prev_pts[i][1];
                    nk++;
                }
            }
            n_tr=nk;
            PROF_TOC(PROF_KLT, t_klt);
            /* RANSAC */
            PROF_TIC(t_ransac);
            if(n_tr>=2 && seg_count>0){
                double (*prev_n)[2]=malloc(n_tr*sizeof(*prev_n));
                double (*cur_n)[2]=malloc(n_tr*sizeof(*cur_n));
                for(int i=0;i<n_tr;i++){ vio_undistort(&K,pv[i][0],pv[i][1],&prev_n[i][0],&prev_n[i][1]); vio_undistort(&K,tracked_pts[i][0],tracked_pts[i][1],&cur_n[i][0],&cur_n[i][1]); }
                /* cam2_R_cam1 = Rc' * imu2_R_imu1 * Rc */
                double tmp[9], cam2_R_cam1[9];
                mat3x3_mul(imu2_R_imu1,calib.imu_R_camera,tmp);
                mat3x3T_mul(calib.imu_R_camera,tmp,cam2_R_cam1);
                vio_rng_t rng; vio_rng_seed(&rng,(uint64_t)(ransac_seed_base+cam_idx));
                uint8_t *mask=malloc(n_tr);
                int nin=vio_two_pt_ransac((const double(*)[2])prev_n,(const double(*)[2])cur_n,n_tr,cam2_R_cam1,VIO_RANSAC_ITERS,VIO_RANSAC_THRESH,&rng,mask);
                if(nin>0){
                    int w=0;
                    for(int i=0;i<n_tr;i++) if(mask[i]){ tracked_pts[w][0]=tracked_pts[i][0]; tracked_pts[w][1]=tracked_pts[i][1]; tracked_ids[w]=tracked_ids[i]; w++; }
                    n_tr=w;
                }
                free(prev_n); free(cur_n); free(mask);
            }
            PROF_TOC(PROF_RANSAC, t_ransac);
            free(pp);free(np);free(val);free(pv);
        }

        /* reseed new features by grid */
        int n_need=VIO_NUM_FEATURES-n_tr;
        PROF_TIC(t_det);
        if(n_need>0){
            int nc=vio_detect_min_eigen(img,W,H,VIO_DETECT_MIN_QUALITY,corners,20000);
            if(nc>0){
                double (*existing)[2]=malloc((n_tr>0?n_tr:1)*sizeof(*existing));
                for(int i=0;i<n_tr;i++){ existing[i][0]=tracked_pts[i][0]; existing[i][1]=tracked_pts[i][1]; }
                int nk=vio_select_grid(corners,nc,(const double(*)[2])existing,n_tr,n_need,W,H,keep_idx);
                for(int i=0;i<nk;i++){
                    if(!undist_in_range(&K,corners[keep_idx[i]].u,corners[keep_idx[i]].v)) continue; /* PROBE2 */
                    tracked_pts[n_tr][0]=corners[keep_idx[i]].u; tracked_pts[n_tr][1]=corners[keep_idx[i]].v; tracked_ids[n_tr]=next_feat_id++; n_tr++;
                }
                free(existing);
            }
        }
        PROF_TOC(PROF_DETECT, t_det);

        /* undistort observations */
        double (*z_norm)[2]=malloc((n_tr>0?n_tr:1)*sizeof(*z_norm));
        for(int i=0;i<n_tr;i++) vio_undistort(&K,tracked_pts[i][0],tracked_pts[i][1],&z_norm[i][0],&z_norm[i][1]);
#ifdef TRACK_DUMP
        /* [TRACK_DUMP 2026-07-30] data-only side channel (PREREG_BATCH_BOUND): every
         * tracked feature's undistorted normalized obs, for the offline batch bound.
         * No effect on any decision — proof = traj md5 identical. */
        {
            static FILE *tdf = NULL;
            if (!tdf) { tdf = fopen("track_dump.csv", "w");
                        if (tdf) fprintf(tdf, "t_s,feat_id,u_norm,v_norm\n"); }
            if (tdf) for (int i = 0; i < n_tr; i++)
                fprintf(tdf, "%.6f,%d,%.8f,%.8f\n", t_cam, tracked_ids[i], z_norm[i][0], z_norm[i][1]);
            /* filter full state per frame — batch smoother initialization (data-only) */
            static FILE *sdf = NULL;
            if (!sdf) { sdf = fopen("state_dump.csv", "w");
                        if (sdf) fprintf(sdf, "t_s,qx,qy,qz,qw,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz\n"); }
            if (sdf) fprintf(sdf, "%.6f,%.8f,%.8f,%.8f,%.8f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                t_cam, st.q_GI[0], st.q_GI[1], st.q_GI[2], st.q_GI[3],
                st.v_GI[0], st.v_GI[1], st.v_GI[2],
                st.bg[0], st.bg[1], st.bg[2], st.ba[0], st.ba[1], st.ba[2]);
        }
#endif

        /* augment clone */
#ifdef KF1_CLONE_SPACING
        /* [KF1] keyframe-spaced clones: augment + obs storage only every KF1_K-th
         * processed frame. KLT/propagation/triggers/updates on other frames unchanged. */
        int kf1_is_kf = ((cam_idx - start_idx) % KF1_K) == 0;
        if (kf1_is_kf) {
#endif
        PROF_TIC(t_aug);
        clone_id++;
        msckf_augment_clone(&st,&calib,(int32_t)clone_id,t_cam);
        int ni=st.clones.N-1;  /* new clone slot */
        for(int j=0;j<4;j++) snap.q_CG[4*ni+j]=st.clones.q_CG[4*ni+j];
        for(int j=0;j<3;j++) snap.t_GC[3*ni+j]=st.clones.t_GC[3*ni+j];
        PROF_TOC(PROF_AUGMENT, t_aug);

        /* update track_map with this frame's observations */
        for(int i=0;i<n_tr;i++){
            int32_t fid=tracked_ids[i];
            int idx=tm_find(fid);
            if(idx<0) idx=tm_create(fid);
            vio_track_t *tr=pool[idx];
            if(tr->n < VIO_MAX_TRACK_LENGTH+4){
                tr->clone_ids[tr->n]=clone_id;
                tr->z[tr->n][0]=z_norm[i][0]; tr->z[tr->n][1]=z_norm[i][1]; tr->z[tr->n][2]=1.0;
                tr->n++;
            }
        }
#ifdef KF1_CLONE_SPACING
        }
#endif
#ifdef CADENCE_DIAG
        double cad_p0[3] = { st.p_GI[0], st.p_GI[1], st.p_GI[2] };
        int cad_n_upd = 0, cad_n_evict = 0;
#endif

        /* build active-set membership for current features (for is_active) */
        /* collect update ids: iterate pool */
        vio_track_t **ids_upd=malloc((pool_n)*sizeof(void*)); int n_upd=0;
        int32_t *to_remove=malloc((pool_n)*sizeof(int32_t)); int n_rm=0;
        for(int idx=0; idx<pool_n; idx++){
            /* skip freed slots: check hash consistency via fid lookup */
            vio_track_t *tr=pool[idx];
            /* determine if slot active: a freed slot won't be found by tm_find(fid)->idx==idx */
            if(tm_find(tr->fid)!=idx) continue;
            int32_t fid=tr->fid;
            int is_active=0;
            for(int i=0;i<n_tr;i++) if(tracked_ids[i]==fid){ is_active=1; break; }
            int is_mature = tr->n >= VIO_MAX_TRACK_LENGTH;
            if((!is_active || is_mature) && tr->n>=VIO_MIN_TRACK_LENGTH){ ids_upd[n_upd++]=tr; }
            else if(!is_active && tr->n<VIO_MIN_TRACK_LENGTH){ to_remove[n_rm++]=fid; }
        }
        for(int i=0;i<n_rm;i++) tm_remove(to_remove[i]);
#ifdef CADENCE_DIAG
        cad_n_upd = n_upd;
#endif

        if(n_upd>0){
            /* snapshot fids to remove after update */
            int32_t *upd_fids=malloc(n_upd*sizeof(int32_t));
            for(int i=0;i<n_upd;i++) upd_fids[i]=ids_upd[i]->fid;
            PROF_TIC(t_upd);
#ifdef KF3_P2B_DIAG
            g_p2b_now = st.t;
#endif
            vio_run_update(&st,&snap,ids_upd,n_upd,&stats);
            PROF_TOC(PROF_UPDATE, t_upd);
            for(int i=0;i<n_upd;i++) tm_remove(upd_fids[i]);
            free(upd_fids);
        }
        free(ids_upd); free(to_remove);
#ifdef KF1_DIAG
        /* [KF1_DIAG] data-only: per-frame update effectiveness (Prof request).
         * mean-parallax per track already in always-on probe3_tracks.csv. */
        {
            static FILE *kdf = NULL;
            if (!kdf) { kdf = fopen("kf1_diag.csv", "w");
                        if (kdf) fprintf(kdf, "t_s,clones_N,accepted_cum\n"); }
            if (kdf) fprintf(kdf, "%.6f,%d,%ld\n", t_cam, st.clones.N, stats.accepted);
        }
#endif

        /* eviction */
        if(st.clones.N >= VIO_MAX_CLONE_NUM){
            int32_t oldest=st.clones.id[0];
            vio_track_t **ids_old=malloc((pool_n)*sizeof(void*)); int n_old=0;
            for(int idx=0;idx<pool_n;idx++){
                vio_track_t *tr=pool[idx];
                if(tm_find(tr->fid)!=idx) continue;
                if(tr->n>0 && tr->clone_ids[0]==oldest && tr->n>=VIO_MIN_TRACK_LENGTH) ids_old[n_old++]=tr;
            }
#ifdef CADENCE_DIAG
            cad_n_evict = n_old;
#endif
            if(n_old>0){
                int32_t *of=malloc(n_old*sizeof(int32_t)); for(int i=0;i<n_old;i++) of[i]=ids_old[i]->fid;
                PROF_TIC(t_updo);
#ifdef KF3_P2B_DIAG
                g_p2b_now = st.t;
#endif
                vio_run_update(&st,&snap,ids_old,n_old,&stats);
                PROF_TOC(PROF_UPDATE, t_updo);
                for(int i=0;i<n_old;i++) tm_remove(of[i]);
                free(of);
            }
            free(ids_old);
            PROF_TIC(t_ev);
            remove_oldest_clone_full(&st,&snap);
            PROF_TOC(PROF_EVICT, t_ev);
        }
#ifdef CADENCE_DIAG
        /* [CADENCE_DIAG] data-only per-frame pipeline-event log (Prof 07-30): correlate
         * which event carries the 0.52 Hz ripple. Does NOT touch estimator output. */
        {
            static FILE *cdf = NULL;
            static long acc_p = 0, rej_p = 0, lp_p = 0;
            if (!cdf) { cdf = fopen("cadence_diag.csv", "w");
                        if (cdf) fprintf(cdf, "t_s,clones_N,n_upd,n_evict,n_lowpar_d,acc_d,rej_d,dp_h,dpx,dpy\n"); }
            long rej_tot = stats.rej_short+stats.rej_baseline+stats.rej_parallax+stats.rej_triangulation
                         + stats.rej_reprojection+stats.rej_depth+stats.rej_chi2;
            double dpx = st.p_GI[0]-cad_p0[0], dpy = st.p_GI[1]-cad_p0[1];   /* signed per-frame correction */
            if (cdf) fprintf(cdf, "%.6f,%d,%d,%d,%ld,%ld,%ld,%.5f,%.6f,%.6f\n",
                             t_cam, st.clones.N, cad_n_upd, cad_n_evict,
                             stats.n_lowpar-lp_p, stats.accepted-acc_p, rej_tot-rej_p,
                             sqrt(dpx*dpx+dpy*dpy), dpx, dpy);
            acc_p = stats.accepted; rej_p = rej_tot; lp_p = stats.n_lowpar;
        }
#endif

        est_p[cam_idx][0]=st.p_GI[0]; est_p[cam_idx][1]=st.p_GI[1]; est_p[cam_idx][2]=st.p_GI[2]; est_has[cam_idx]=1;
#ifdef KF3_P2B_DIAG
        /* [P2B] estimator-state per-frame row — logging-only, non-invasive (traj md5 identical). */
        {
            static FILE *g_p2bs = NULL;
            const int PS2 = MSCKF_MAX_STATE_SZ;
            if (!g_p2bs) { const char *p = getenv("VIO_P2B_STATE_CSV");
                if (p) { g_p2bs = fopen(p, "w");
                    if (g_p2bs) fprintf(g_p2bs, "t,imu_t,n_prop,sum_dt,bgx,bgy,bgz,bax,bay,baz,"
                                        "vx,vy,vz,fbx,fby,fbz,Pp00,Pp11,Pp22,Pv00,Pv11,Pv22,qx,qy,qz,qw\n"); } }
            if (g_p2bs) {
                double fb[3] = {0, 0, 0};
                if (g_p2b_nprop > 0) for (int k2 = 0; k2 < 3; k2++) fb[k2] = g_p2b_fsum[k2] / g_p2b_nprop;
                fprintf(g_p2bs, "%.9f,%.9f,%d,%.6f,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
                        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9f,%.9f,%.9f,%.9f\n",
                        st.t, g_p2b_imut, g_p2b_nprop, g_p2b_sumdt,
                        (double)st.bg[0], (double)st.bg[1], (double)st.bg[2],
                        (double)st.ba[0], (double)st.ba[1], (double)st.ba[2],
                        (double)st.v_GI[0], (double)st.v_GI[1], (double)st.v_GI[2],
                        fb[0], fb[1], fb[2],
                        (double)st.P[3*PS2+3], (double)st.P[4*PS2+4], (double)st.P[5*PS2+5],
                        (double)st.P[6*PS2+6], (double)st.P[7*PS2+7], (double)st.P[8*PS2+8],
                        (double)st.q_GI[0], (double)st.q_GI[1], (double)st.q_GI[2], (double)st.q_GI[3]);
            }
        }
#endif
#ifdef KF3_COV_DIAG
        /* [ROUND2 cov-consistency DIAG] logging-only, non-invasive (traj byte-identical). */
        {
            static FILE *g_cov = NULL, *g_pfull = NULL; static double last_pf = -1e18;
            const int PS = MSCKF_MAX_STATE_SZ;
            if (!g_cov) { const char *p = getenv("VIO_COV_CSV");
                if (p) { g_cov = fopen(p, "w");
                    if (g_cov) fprintf(g_cov, "t,px,py,pz,Pxx,Pxy,Pyy,P00,P01,P02,P11,P12,P22,q0,q1,q2,q3,g_kf3_f,accepted\n"); } }
            if (!g_pfull) { const char *p = getenv("VIO_PFULL_CSV"); if (p) g_pfull = fopen(p, "w"); }
            double gf = 1.0;
#ifdef KF3_TERM_GOV
            gf = g_kf3_f;
#endif
            if (g_cov)
                fprintf(g_cov, "%.9f,%.6g,%.6g,%.6g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.4f,%ld\n",
                        st.t, st.p_GI[0], st.p_GI[1], st.p_GI[2],
                        (double)st.P[3*PS+3], (double)st.P[3*PS+4], (double)st.P[4*PS+4],
                        (double)st.P[0*PS+0], (double)st.P[0*PS+1], (double)st.P[0*PS+2],
                        (double)st.P[1*PS+1], (double)st.P[1*PS+2], (double)st.P[2*PS+2],
                        (double)st.q_GI[0], (double)st.q_GI[1], (double)st.q_GI[2], (double)st.q_GI[3],
                        gf, (long)stats.accepted);
            if (g_pfull && st.t - last_pf >= 1.0) { last_pf = st.t;
                fprintf(g_pfull, "%.9f", st.t);
                for (int a = 0; a < MSCKF_IMU_SZ; a++)
                    for (int b = 0; b < MSCKF_IMU_SZ; b++) fprintf(g_pfull, ",%.9g", (double)st.P[a*PS+b]);
                fprintf(g_pfull, "\n");
            }
        }
#endif

        /* roll frontend state */
        if(prev_img && prev_img!=img) free(prev_img);
        prev_img=img;
        for(int i=0;i<n_tr;i++){ prev_pts[i][0]=tracked_pts[i][0]; prev_pts[i][1]=tracked_pts[i][1]; feat_ids[i]=tracked_ids[i]; }
        n_prev=n_tr;

        free(tracked_pts); free(tracked_ids); free(z_norm);

        PROF_TOC(PROF_FRAME, t_frame);
        vio_prof_frame_commit(cam_idx-start_idx, t_cam, n_tr);

        if((cam_idx-start_idx+1)%100==0)
            fprintf(stderr,"frame %d/%d  p=[%.2f %.2f %.2f] N=%d acc=%ld rej[s%ld b%ld p%ld t%ld r%ld d%ld c%ld]\n",
                cam_idx-start_idx+1,n_cam-start_idx,st.p_GI[0],st.p_GI[1],st.p_GI[2],st.clones.N,
                stats.accepted,stats.rej_short,stats.rej_baseline,stats.rej_parallax,stats.rej_triangulation,stats.rej_reprojection,stats.rej_depth,stats.rej_chi2);
    }

    /* write trajectory + compute ATE vs dataset GT (nearest ts, no alignment).
     * Unwritable output path must NOT crash the run — warn and still report ATE. */
    FILE *ot=fopen(out_path,"w");
    if(!ot) fprintf(stderr,"warn: cannot write %s (%s) — trajectory file skipped, ATE still computed\n",out_path,strerror(errno));
    char gpath[1200]; snprintf(gpath,sizeof gpath,"%s.gt.csv",out_path);
    FILE *gt=ot?fopen(gpath,"w"):NULL;
    if(ot && !gt) fprintf(stderr,"warn: cannot write %s\n",gpath);
#ifdef KF3_OUT_LP
    /* [KF3_OUT_LP] output governor (deployment smoothing) — PREREG_KF3_OUT_LP (Prof GO 1).
     * PLAIN causal EMA on published horizontal (x,y). (The prereg's linear lag-compensation
     * was found to be a mathematical no-op: full first-order compensation exactly recovers
     * the raw signal — proven by byte-identical output — so it is dropped; the inherent
     * EMA group delay is the smoothing-vs-lag tradeoff under test, STOP->(2) if it exceeds
     * acceptance.) NOT an estimator change: modifies only est_p[] before write; filter
     * state untouched. alpha=0.14 (~0.46 Hz cutoff) attenuates the measured ~0.5 Hz ripple. */
    {
        const double LP_ALPHA = 0.14;
        double sx=0.0, sy=0.0; int lp_have=0;
        for(int fr=start_idx; fr<n_cam; fr++){
            if(!est_has[fr]) continue;
            double x=est_p[fr][0], y=est_p[fr][1];
            if(!lp_have){ sx=x; sy=y; lp_have=1; }
            else { sx += LP_ALPHA*(x-sx); sy += LP_ALPHA*(y-sy); }
            est_p[fr][0]=sx; est_p[fr][1]=sy;
        }
    }
#endif
#ifdef KF3_OUT_RL
    /* [KF3_OUT_RL] output rate-limiter (deployment jump-guard, panel D): clamp published
     * horizontal per-frame increment to a physically-feasible speed V_RL (envelope 5 + margin 3
     * m/s; rule-first, NOT tuned) — kills prohibited-jump/P2 excursions at the output without
     * blurring normal motion. NOT an estimator change: modifies est_p[] before write only.
     * Composes after OUT_LP if both set. off=byte-identical. */
    {
        const double V_RL = 8.0;
        double px=0.0, py=0.0, tp=0.0; int have=0;
        for (int fr=start_idx; fr<n_cam; fr++){
            if(!est_has[fr]) continue;
            double x=est_p[fr][0], y=est_p[fr][1];
            double tt=(double)cam_t_ns[fr]*1e-9;
            if(!have){ px=x; py=y; tp=tt; have=1; continue; }
            double dt=tt-tp; if(dt<=0.0) dt=1e-3;
            double dx=x-px, dy=y-py, dd=sqrt(dx*dx+dy*dy), lim=V_RL*dt;
            if(dd>lim && dd>1e-9){ double s=lim/dd; dx*=s; dy*=s; }
            px+=dx; py+=dy; tp=tt;
            est_p[fr][0]=px; est_p[fr][1]=py;
        }
    }
#endif
    double sse=0; int nrec=0;
    for(int fr=start_idx; fr<n_cam; fr++){
        if(!est_has[fr]) continue;
        double ts=(double)cam_t_ns[fr]*1e-9;
        if(ot) fprintf(ot,"%.9f,%.9g,%.9g,%.9g\n",ts,est_p[fr][0],est_p[fr][1],est_p[fr][2]);
        int j=gt_nearest(cam_t_ns[fr]);
        if(gt) fprintf(gt,"%.9f,%.9g,%.9g,%.9g\n",ts,gt_p[j][0],gt_p[j][1],gt_p[j][2]);
        double dx=est_p[fr][0]-gt_p[j][0], dy=est_p[fr][1]-gt_p[j][1], dz=est_p[fr][2]-gt_p[j][2];
        sse += dx*dx+dy*dy+dz*dz; nrec++;
    }
    if(ot) fclose(ot);
    if(gt) fclose(gt);
    double ate = nrec? sqrt(sse/nrec) : NAN;
    fprintf(stderr,"\n=== DONE: %d traj rows -> %s ===\n",nrec,out_path);
    fprintf(stderr,"ATE (no-align, vs dataset GT) = %.4f m over %d frames\n",ate,nrec);
    fprintf(stderr,"accepted=%ld rej[short=%ld base=%ld par=%ld tri=%ld reproj=%ld depth=%ld chi2=%ld]\n",
        stats.accepted,stats.rej_short,stats.rej_baseline,stats.rej_parallax,stats.rej_triangulation,stats.rej_reprojection,stats.rej_depth,stats.rej_chi2);
#ifdef N3_BARO_Z
    fprintf(stderr,"[N3] baro updates applied=%d sanity_rejected=%d\n",n3_applied,n3_rej);
#endif
#ifdef ADAPT_C2_DEPTH_PRIOR
    fprintf(stderr,"[C2] depth-prior tracks=%ld (final ramp w=%.2f)\n",g_c2_n_prior,g_c2_w);
#endif
#ifdef KF2_HOVER_GOV
    fprintf(stderr,"[KF2] suppressed visual updates=%ld (final freeze=%d)\n",g_kf2_suppressed,g_kf2_freeze);
#endif
    printf("ATE %.4f m  frames %d  accepted %ld\n",ate,nrec,stats.accepted);
    vio_prof_write(out_path);
    return 0;
}
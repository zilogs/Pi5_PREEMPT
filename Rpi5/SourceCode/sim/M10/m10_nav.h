/*------------------------------------------------------------------------

 ------------------------------------------------------------------------*/
#include    "m10_func.h"
/*------------------------------------------------------------------------

 ------------------------------------------------------------------------*/
enum { CLASS_NAV = 0x01 };
/*------------------------------------------------------------------------

 ------------------------------------------------------------------------*/
enum {  CLASS_NAV_PVT = 0x07 , CLASS_NAV_STATUS = 0x03 , CLASS_NAV_ORB = 0x34 ,
        CLASS_NAV_SAT = 0x35 , CLASS_NAV_SIG = 0x43    , CLASS_NAV_CLOCK = 0x22 };
/*------------------------------------------------------------------------

 ------------------------------------------------------------------------*/
typedef UBYTE   GPS_FIX_REP     <read=gpsFix_report>;
typedef UBYTE   CARR_SOLN       <read=carr_solution_func>;
typedef UBYTE   PVT_PSM_STATE   <read=pvt_psm_state>;
typedef UBYTE   LAST_CORR_AGE   <read=last_correction_age>;

string gpsFix_report ( GPS_FIX_REP v )
{
    switch( v ) {
    case 0 : return "no fix";
    case 1 : return "dead reckon only";
    case 2 : return "2D-fix";
    case 3 : return "3D-fix";
    case 4 : return "GPS+dead rekon combined";
    case 5 : return "Time only fix";
    }
    return "Reserved";
}

string carr_solution_func( CARR_SOLN v )
{
    switch (v) {
    case 0 : return "No carrier phase range solution";
    case 1 : return "Carrier phase & floating ambiguities";
    case 2 : return "Carrier phase & fix ambiguities";
    }
    return "Reserved";
}

string pvt_psm_state( PVT_PSM_STATE v )
{
    switch(v) {
    case 0 : return "PSM is not active";
    case 1 : return "Enabled";
    case 2 : return "Acquisition";
    case 3 : return "Tracking";
    case 4 : return "Power Optimized Tracking";
    case 5 : return "Inactive";
    }
    return "Reserved";
}

string last_correction_age( LAST_CORR_AGE v )
{
    switch(v) {
    case 0 : return "Not avaiable";
    case 1 : return "Age 0-1 sec.";
    case 2 : return "Age 1-2 sec.";
    case 3 : return "Age 2-5 sec.";
    case 4 : return "Age 5-10 sec.";
    case 5 : return "Age 10-15 sec.";
    case 6 : return "Age 15-20 sec.";
    case 7 : return "Age 20-30 sec.";
    case 8 : return "Age 30-45 sec.";
    case 9 : return "Age 45-60 sec.";
    case 10 : return "Age 60-90 sec.";
    case 11 : return "Age 90-120 sec.";
    default : return "Age greater or equal than 120 sec.";
    }
    return "Reserved";
}

typedef struct {
    UINT32      GPS_TOW;
    UINT16      Year;
    UBYTE       Month;
    UBYTE       Day;
    UBYTE       Hour;
    UBYTE       minute;
    UBYTE       sec;
    // UBYTE       valid;
    struct  {
        UBYTE   validDate   : 1;
        UBYTE   validTime   : 1;
        UBYTE   fullyResolved : 1;
        UBYTE   validMag    : 1;
        UBYTE   _reserved   : 4;
    } valid;
    UINT32      tAcc_ns;
    INT32       nano;
    GPS_FIX_REP FixType;
    struct  {
        UBYTE   gnssFixOK       : 1;
        UBYTE   diffSoln        : 1;
        PVT_PSM_STATE psmState  : 3;
        UBYTE   headVehValid    : 1;
        CARR_SOLN carrSoln      : 2;
    } Flags;
    struct {
        UBYTE   _reserved     : 5 <format=binary>;
        UBYTE   confirmedAval : 1;
        UBYTE   confirmedData : 1;
        UBYTE   confirmedTime : 1;
    } Flags2;
    UBYTE       numSV;
    POSIT_SCALE lon;
    POSIT_SCALE lat;
    MET_SCALE   height;
    MET_SCALE   hMSL;
    MET_SCALE   hAcc;
    MET_SCALE   vAcc;
    MET_SEC_SCALE   velN;
    MET_SEC_SCALE   velE;
    MET_SEC_SCALE   velD;
    MET_SEC_SCALE   gSpeed;
    ANGLE_SCALE     headMot;
    MET_SEC_SCALE   sAcc;
    ANGLE_SCALE     headAcc;
    PDOP_SCALE      pDOP;
    // UINT16          flag3 <format=hex>;
    struct {
        UBYTE       invalidL1h  : 1;
        LAST_CORR_AGE lastCorrectionAge : 4 <format=hex>;
        UBYTE       _reserved   : 3;
        UBYTE       _reserved_h : 8;
    } flag3;
    UBYTE           spare[4];
    ANGLE_SCALE     headVeh;
    ANGLE_I16_SCALE magDec;
    ANGLE_U16_SCALE magAcc;
} UBX_NAV_PVT;

/*------------------------------------------------------------------------

 ------------------------------------------------------------------------*/
typedef struct {
    UBYTE           gpsFixOK    : 1;   // 1 : Valid
    UBYTE           diffSoln    : 1;   // 1 : DGPS/RTK
    UBYTE           wknSet      : 1;   // 1 : Valid Week Number
    UBYTE           towSet      : 1;   // 1 : Valid Time of Week
    UBYTE           _reserved   : 4 <format=binary>;   // Normal = 0
} UBX_NAV_STATUS_FLAGS;

typedef     UBYTE   MAP_MATCHING <read=map_matching>;

string map_matching ( MAP_MATCHING v )
{
    switch ( v ) {
    case 0 : return "None";
    case 1 : return "Valid but not used";
    case 2 : return "Valid and used";
    case 3 : return "Valid and used requires map lat/long";
    }
    return "Reserved";
}

typedef struct {
    UBYTE           diffCorr        : 1;
    UBYTE           carrSolnValid   : 1;
    UBYTE           _reserved       : 4 <format=binary>;
    MAP_MATCHING    mapMatching     : 2;
} UBX_NAV_STATUS_FIXSTAT;

typedef     UBYTE   PSM_STATE <read=psm_state_func>;
typedef     UBYTE   SPOOF_STATE <read=spoof_state_func>;

string psm_state_func( PSM_STATE v )
{
    switch (v) {
    case 0 : return "ACQUISITION";
    case 1 : return "TRACKING";
    case 2 : return "POWER OPTIMIZED TRACKING";
    case 3 : return "INACTIVE";
    }
    return "Reserved";
}

string spoof_state_func( SPOOF_STATE v )
{
    switch (v) {
    case 0 : return "Unknown or deactived";
    case 1 : return "No spoofing indicated";
    case 2 : return "Spoofing indicated";
    case 3 : return "Multiple spoofing indications";
    }
    return "Reserved";
}

typedef struct {
    PSM_STATE       psmState        : 2;    // Power Save mode
    UBYTE           _reserved1      : 1;
    SPOOF_STATE     spoofDetState   : 2;
    UBYTE           _reserved2      : 1;
    CARR_SOLN       carrSoln        : 2;
} UBX_NAV_STATUS_FLAGS2;

typedef struct {
    UINT32          iTOW;               // GPS time of week (ms.)
    GPS_FIX_REP     gpsFix;
    UBX_NAV_STATUS_FLAGS    flags;      // Navigation Status Flags
    UBX_NAV_STATUS_FIXSTAT  fixStat;    // Fix Status Information
    UBX_NAV_STATUS_FLAGS2   flags2;
    UINT32          ttff;               // Time to first fix (ms.)
    UINT32          msss;               // Milliseconds since Startup / Reset
} UBX_NAV_STATUS;

/*------------------------------------------------------------------------

 ------------------------------------------------------------------------*/
typedef struct {
    UINT8           gnssId;
    UINT8           svId;
    UINT8           svFlag;
    UINT8           eph;
    UINT8           alm;
    UINT8           otherOrb;
} UBX_NAV_ORB_GROUP;

typedef struct {
    UINT32          iTOW;
    UINT8           version;
    UINT8           numSv;
    UINT8           reserved0[2];
    if ( numSv )
        UBX_NAV_ORB_GROUP   orb_group[numSv];
} UBX_NAV_ORB;

/*------------------------------------------------------------------------

 ------------------------------------------------------------------------*/
typedef struct {
    UINT8           gnssId;
    UINT8           svId;
    UINT8           cno;
    INT8            elev;
    INT16           azim;
    INT16           prRes;
    UINT32          flags;
} UBX_NAV_SAT_GROUP;

typedef struct {
    UINT32          iTOW;
    UINT8           version;
    UINT8           numSv;
    UINT8           reserved0[2];
    if ( numSv )
        UBX_NAV_SAT_GROUP   orb_group[numSv];
} UBX_NAV_SAT;

/*------------------------------------------------------------------------

 ------------------------------------------------------------------------*/
typedef struct {
    UINT8           gnssId;
    UINT8           svId;
    UINT8           sigId;
    UINT8           freqId;
    INT16           prRes;
    UINT8           cno;
    UINT8           qualityInd;
    UINT8           corrSource;
    UINT8           ionoMode1;
    UINT16          sigFlags;
    UINT8           reserved1[4];
} UBX_NAV_SIG_GROUP;

typedef struct {
    UINT32          iTOW;
    UINT8           version;
    UINT8           numSv;
    UINT8           reserved0[2];
    if ( numSv )
        UBX_NAV_SIG_GROUP   sig_group[numSv];
} UBX_NAV_SIG;

/*------------------------------------------------------------------------

 ------------------------------------------------------------------------*/
typedef struct {
    UINT32          iTOW;
    INT32           clkB;
    INT32           clkD;
    UINT32          tAcc;
    UINT32          fAcc;
} UBX_NAV_CLOCK;


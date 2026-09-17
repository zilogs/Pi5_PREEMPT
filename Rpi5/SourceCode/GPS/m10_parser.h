#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>

#define PACKED __attribute__((packed))

enum { CLASS_NAV = 0x01 };
enum { CLASS_NAV_PVT = 0x07, CLASS_NAV_STATUS = 0x03, CLASS_NAV_CLOCK = 0x22 };

inline double scalePos(int32_t v)   { return static_cast<double>(v) * 1e-7; }
inline double scaleMeter(int32_t v) { return static_cast<double>(v) * 1e-3; }

struct PACKED UBX_NAV_STATUS {
    uint32_t iTOW; uint8_t gpsFix;
    struct { uint8_t gpsFixOK:1, diffSoln:1, wknSet:1, towSet:1, _res:4; } flags;
    struct { uint8_t diffCorr:1, carrSolnValid:1, _res:4, mapMatching:2; } fixStat;
    struct { uint8_t psmState:2, _res1:1, spoofDetState:2, _res2:1, carrSoln:2; } flags2;
    uint32_t ttff, msss;
};

struct PACKED UBX_NAV_CLOCK {
    uint32_t iTOW; int32_t clkB, clkD; uint32_t tAcc, fAcc;
};

template <typename T>
inline T get_val(const uint8_t* pay, size_t offset) {
    T val;
    std::memcpy(&val, pay + offset, sizeof(T));
    return val;
}

// รีเซ็ต state machine: ถ้าไบต์ล่าสุดคือ 0xB5 ให้เริ่มนับใหม่จาก 1 (ชนหัวติดกันพอดี) ไม่งั้นเริ่มจาก 0
inline int resync(uint8_t* p, uint8_t c) {
    if (c != 0xB5) return 0;
    p[0] = 0xB5;
    return 1;
}

// UBX 8-bit Fletcher checksum คำนวณจาก class..payload (ไม่รวม sync bytes และ checksum เอง)
inline bool ubxChecksumOk(const uint8_t* p, int len) {
    uint8_t ckA = 0, ckB = 0;
    for (int k = 2; k < len - 2; k++) {
        ckA = static_cast<uint8_t>(ckA + p[k]);
        ckB = static_cast<uint8_t>(ckB + ckA);
    }
    return ckA == p[len - 2] && ckB == p[len - 1];
}

inline void parse_ubx_byte(uint8_t c) {
    static uint8_t p[100];
    static int i = 0;
    static const uint8_t h[6] = {0xB5, 0x62, CLASS_NAV, CLASS_NAV_PVT, 0x5C, 0x00};
    static const char* COL[5] = {"\033[1;31m", "\033[1;33m", "\033[1;32m", "\033[1;36m", "\033[1;35m"};

    if (i == 0 && c == 0xB5) printf("\n\033[1;90m[UBX] \033[0m");
    printf("%s%02X \033[0m", COL[i < 2 ? 0 : i < 4 ? 1 : i < 6 ? 2 : i < 98 ? 3 : 4], c);
    fflush(stdout);

    p[i++] = c;

    if (i <= 6 && p[i - 1] != h[i - 1]) { i = resync(p, c); return; }
    if (i < 100) return;

    if (ubxChecksumOk(p, 100)) {
        printf("\n\033[1;32m[UBX] Checksum OK!\033[0m\n");
        uint8_t* pay = p + 6;

        // ดึงค่าพื้นฐาน
        uint32_t iTOW     = get_val<uint32_t>(pay, 0);
        uint16_t year     = get_val<uint16_t>(pay, 4);
        uint8_t  month    = get_val<uint8_t>(pay, 6);
        uint8_t  day      = get_val<uint8_t>(pay, 7);
        uint8_t  hour     = get_val<uint8_t>(pay, 8);
        uint8_t  min      = get_val<uint8_t>(pay, 9);
        uint8_t  sec      = get_val<uint8_t>(pay, 10);
        uint8_t  valid    = get_val<uint8_t>(pay, 11);
        uint32_t tAcc     = get_val<uint32_t>(pay, 12);
        int32_t  nano     = get_val<int32_t>(pay, 16);
        uint8_t  fixType  = get_val<uint8_t>(pay, 20);
        uint8_t  flags    = get_val<uint8_t>(pay, 21);
        uint8_t  numSV    = get_val<uint8_t>(pay, 23);
        
        int32_t  lon      = get_val<int32_t>(pay, 24);
        int32_t  lat      = get_val<int32_t>(pay, 28);
        int32_t  height   = get_val<int32_t>(pay, 32);
        int32_t  hMSL     = get_val<int32_t>(pay, 36);
        uint32_t hAcc     = get_val<uint32_t>(pay, 40);
        uint32_t vAcc     = get_val<uint32_t>(pay, 44);

        int32_t  velN     = get_val<int32_t>(pay, 48);
        int32_t  velE     = get_val<int32_t>(pay, 52);
        int32_t  velD     = get_val<int32_t>(pay, 56);
        int32_t  gSpeed   = get_val<int32_t>(pay, 60);
        int32_t  headMot  = get_val<int32_t>(pay, 64);
        uint32_t sAcc     = get_val<uint32_t>(pay, 68);
        uint32_t headAcc  = get_val<uint32_t>(pay, 72);
        uint16_t pDOP     = get_val<uint16_t>(pay, 76);

        // แยกบิตแฟล็กย่อย
        uint8_t validDate  = (valid & 0x01);
        uint8_t validTime  = (valid & 0x02) >> 1;
        uint8_t fixOK      = (flags & 0x01);
        uint8_t diffSoln   = (flags & 0x02) >> 1;
        uint8_t psmState   = (flags & 0x40) >> 6;     // bit 6 ของ flags (offset 21)
        uint8_t carrSoln   = (flags & 0x80) >> 7;     // bit 7 ของ flags (offset 21)

        if (FILE* log = fopen("gps.log", "a")) {
            fprintf(log, 
                "--------------------------------------------------\n"
                "[NAV-PVT] TOW: %u | Date: %04u-%02u-%02u %02u:%02u:%02u\n"
                "  ├─ Status  : Valid(Date:%u, Time:%u) | FixType: %u | FixOK: %u | DiffSoln: %u\n"
                "  ├─ System  : PSM: %u | CarrSoln: %u | numSV: %u | tAcc: %u ns | Nano: %d\n"
                "  ├─ Position: Lat: %.7f | Lon: %.7f\n"
                "  ├─ Altitude: Height: %.6fm | MSL: %.6fm | hAcc: %.6fm | vAcc: %.6fm\n"
                "  ├─ Velocity: VelN: %.3fm/s | VelE: %.3fm/s | VelD: %.3fm/s | gSpeed: %.3fm/s\n"
                "  └─ Dynamics: Heading: %.5f° | sAcc: %.3fm/s | HeadAcc: %.5f° | pDOP: %.2f\n",
                iTOW, year, month, day, hour, min, sec,
                validDate, validTime, fixType, fixOK, diffSoln,
                psmState, carrSoln, numSV, tAcc, nano,
                scalePos(lat), scalePos(lon),
                scaleMeter(height), scaleMeter(hMSL), scaleMeter(hAcc), scaleMeter(vAcc),
                scaleMeter(velN), scaleMeter(velE), scaleMeter(velD), scaleMeter(gSpeed),
                static_cast<double>(headMot) * 1e-5,
                scaleMeter(sAcc),
                static_cast<double>(headAcc) * 1e-5,
                static_cast<double>(pDOP) * 0.01
            );
            fclose(log);
        }
    } else {
        printf("\n\033[1;31m[UBX] Checksum mismatch\033[0m\n");
    }

    i = resync(p, c);
}
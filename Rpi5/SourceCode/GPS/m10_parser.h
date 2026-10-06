#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>

#define PACKED __attribute__((packed))
#define NAV_PVT_LEN 100  // 6 (header) + 92 (payload) + 2 (checksum)

enum { CLASS_NAV = 0x01 };
enum { CLASS_NAV_PVT = 0x07, CLASS_NAV_STATUS = 0x03, CLASS_NAV_CLOCK = 0x22 };

inline double scalePos(int32_t v)   { return static_cast<double>(v) * 1e-7; }
inline double scaleMeter(int32_t v) { return static_cast<double>(v) * 1e-3; }

template <typename T>
inline T get_val(const uint8_t* pay, size_t offset) {
    T val;
    std::memcpy(&val, pay + offset, sizeof(T));
    return val;
}

// UBX 8-bit Fletcher checksum
inline bool ubxChecksumOk(const uint8_t* p, int len) {
    uint8_t ckA = 0, ckB = 0;
    for (int k = 2; k < len - 2; k++) {
        ckA = static_cast<uint8_t>(ckA + p[k]);
        ckB = static_cast<uint8_t>(ckB + ckA);
    }
    return ckA == p[len - 2] && ckB == p[len - 1];
}

// ── ถอดรหัส NAV-PVT 1 ข้อความ (msg = pointer to 100-byte message) ──
inline void decode_nav_pvt(const uint8_t* msg, FILE* log) {
    const uint8_t* pay = msg + 6;

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

    uint8_t validDate  = (valid & 0x01);
    uint8_t validTime  = (valid & 0x02) >> 1;
    uint8_t fixOK      = (flags & 0x01);
    uint8_t diffSoln   = (flags & 0x02) >> 1;
    uint8_t psmState   = (flags & 0x40) >> 6;
    uint8_t carrSoln   = (flags & 0x80) >> 7;

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
    fflush(log);  // flush ทุก message → ไม่ค้างใน buffer (กันข้อมูลหายตอน kill)
}

// ── ประมวลผล buffer ทั้งก้อน: หาหัว B5 62 01 07 5C 00 ด้วย memmem (SIMD) ──
// คืนค่า: จำนวนข้อความที่ถอดรหัสได้
inline int parse_ubx_buffer(const uint8_t* buf, size_t len, FILE* log) {
    static const uint8_t h[6] = {0xB5, 0x62, 0x01, 0x07, 0x5C, 0x00};
    int count = 0;
    size_t pos = 0;

    while (pos + NAV_PVT_LEN <= len) {
        // memmem = SIMD-accelerated pattern search (เร็วมากสำหรับข้อมูลกาก)
        const uint8_t* found = static_cast<const uint8_t*>(
            memmem(buf + pos, len - pos, h, 6));
        if (!found) break;

        size_t offset = static_cast<size_t>(found - buf);

        // ต้องมีครบ 100 byte ถึงจะถอดได้
        if (offset + NAV_PVT_LEN > len) break;

        if (ubxChecksumOk(found, NAV_PVT_LEN)) {
            decode_nav_pvt(found, log);
            count++;
        }

        // ข้ามไปหลัง header นี้ (ไม่ skip ทั้ง message เพราะอาจมี message ซ้อน)
        pos = offset + 6;
    }

    return count;
}
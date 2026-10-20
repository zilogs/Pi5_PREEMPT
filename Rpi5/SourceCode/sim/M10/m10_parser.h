#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include "../rt_util.h"     // SpscRing

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

// ค่าดิบที่ถอดแล้ว ส่งผ่าน ring ไปให้ thread เบื้องหลังจัดรูปแบบและเขียนไฟล์
struct NavPvtLog {
    uint32_t iTOW, tAcc, hAcc, vAcc, sAcc, headAcc;
    int32_t  nano, lon, lat, height, hMSL, velN, velE, velD, gSpeed, headMot;
    uint16_t year, pDOP;
    uint8_t  month, day, hour, minute, second, valid, fixType, flags, numSV;
};
using NavPvtRing = SpscRing<NavPvtLog, 256>;

// UBX 8-bit Fletcher checksum
inline bool ubxChecksumOk(const uint8_t* p, int len) {
    uint8_t ckA = 0, ckB = 0;
    for (int k = 2; k < len - 2; k++) {
        ckA = static_cast<uint8_t>(ckA + p[k]);
        ckB = static_cast<uint8_t>(ckB + ckA);
    }
    return ckA == p[len - 2] && ckB == p[len - 1];
}

// ── จัดรูปแบบข้อความ (เรียกจาก thread เบื้องหลังเท่านั้น ไม่ใช่ RT thread) ──
inline void format_nav_pvt(FILE* log, const NavPvtLog& r) {
    uint8_t validDate  = (r.valid & 0x01);
    uint8_t validTime  = (r.valid & 0x02) >> 1;
    uint8_t fixOK      = (r.flags & 0x01);
    uint8_t diffSoln   = (r.flags & 0x02) >> 1;
    uint8_t psmState   = (r.flags & 0x40) >> 6;
    uint8_t carrSoln   = (r.flags & 0x80) >> 7;

    fprintf(log,
        "--------------------------------------------------\n"
        "[NAV-PVT] TOW: %u | Date: %04u-%02u-%02u %02u:%02u:%02u\n"
        "  ├─ Status  : Valid(Date:%u, Time:%u) | FixType: %u | FixOK: %u | DiffSoln: %u\n"
        "  ├─ System  : PSM: %u | CarrSoln: %u | numSV: %u | tAcc: %u ns | Nano: %d\n"
        "  ├─ Position: Lat: %.7f | Lon: %.7f\n"
        "  ├─ Altitude: Height: %.6fm | MSL: %.6fm | hAcc: %.6fm | vAcc: %.6fm\n"
        "  ├─ Velocity: VelN: %.3fm/s | VelE: %.3fm/s | VelD: %.3fm/s | gSpeed: %.3fm/s\n"
        "  └─ Dynamics: Heading: %.5f° | sAcc: %.3fm/s | HeadAcc: %.5f° | pDOP: %.2f\n",
        r.iTOW, r.year, r.month, r.day, r.hour, r.minute, r.second,
        validDate, validTime, r.fixType, fixOK, diffSoln,
        psmState, carrSoln, r.numSV, r.tAcc, r.nano,
        scalePos(r.lat), scalePos(r.lon),
        scaleMeter(r.height), scaleMeter(r.hMSL), scaleMeter(r.hAcc), scaleMeter(r.vAcc),
        scaleMeter(r.velN), scaleMeter(r.velE), scaleMeter(r.velD), scaleMeter(r.gSpeed),
        static_cast<double>(r.headMot) * 1e-5,
        scaleMeter(r.sAcc),
        static_cast<double>(r.headAcc) * 1e-5,
        static_cast<double>(r.pDOP) * 0.01
    );
}

// ── ถอดรหัส NAV-PVT 1 ข้อความ (msg = pointer to 100-byte message) ──
// แค่ถอดค่าดิบใส่ out — ไม่มี I/O ไม่ push ring ไม่เขียน shm (ฝั่ง caller ตัดสินใจ)
inline void decode_nav_pvt(const uint8_t* msg, NavPvtLog& out) {
    const uint8_t* pay = msg + 6;

    out.iTOW    = get_val<uint32_t>(pay, 0);
    out.year    = get_val<uint16_t>(pay, 4);
    out.month   = get_val<uint8_t>(pay, 6);
    out.day     = get_val<uint8_t>(pay, 7);
    out.hour    = get_val<uint8_t>(pay, 8);
    out.minute  = get_val<uint8_t>(pay, 9);
    out.second  = get_val<uint8_t>(pay, 10);
    out.valid   = get_val<uint8_t>(pay, 11);
    out.tAcc    = get_val<uint32_t>(pay, 12);
    out.nano    = get_val<int32_t>(pay, 16);
    out.fixType = get_val<uint8_t>(pay, 20);
    out.flags   = get_val<uint8_t>(pay, 21);
    out.numSV   = get_val<uint8_t>(pay, 23);

    out.lon     = get_val<int32_t>(pay, 24);
    out.lat     = get_val<int32_t>(pay, 28);
    out.height  = get_val<int32_t>(pay, 32);
    out.hMSL    = get_val<int32_t>(pay, 36);
    out.hAcc    = get_val<uint32_t>(pay, 40);
    out.vAcc    = get_val<uint32_t>(pay, 44);

    out.velN    = get_val<int32_t>(pay, 48);
    out.velE    = get_val<int32_t>(pay, 52);
    out.velD    = get_val<int32_t>(pay, 56);
    out.gSpeed  = get_val<int32_t>(pay, 60);
    out.headMot = get_val<int32_t>(pay, 64);
    out.sAcc    = get_val<uint32_t>(pay, 68);
    out.headAcc = get_val<uint32_t>(pay, 72);
    out.pDOP    = get_val<uint16_t>(pay, 76);
}

// ── ประมวลผล buffer ทั้งก้อน: หาหัว B5 62 01 07 5C 00 ด้วย memmem (SIMD) ──
// เรียก onMsg(r) ทีละข้อความที่ checksum ผ่าน (ตัวนี้ไม่มี I/O — ฝั่ง caller ตัดสินใจ)
// คืนค่าจำนวนข้อความที่ checksum ผ่าน
template <typename OnMsg>
inline int parse_ubx_buffer(const uint8_t* buf, size_t len, OnMsg&& onMsg) {
    static const uint8_t h[6] = {0xB5, 0x62, 0x01, 0x07, 0x5C, 0x00};
    int count = 0;
    size_t pos = 0;

    while (pos + NAV_PVT_LEN <= len) {
        const uint8_t* found = static_cast<const uint8_t*>(
            memmem(buf + pos, len - pos, h, 6));
        if (!found) break;

        size_t offset = static_cast<size_t>(found - buf);

        // ต้องมีครบ 100 byte ถึงจะถอดได้
        if (offset + NAV_PVT_LEN > len) break;

        if (ubxChecksumOk(found, NAV_PVT_LEN)) {
            NavPvtLog r;
            decode_nav_pvt(found, r);
            onMsg(r);
            count++;
        }

        // ข้ามไปหลัง header นี้ (ไม่ skip ทั้ง message เพราะอาจมี message ซ้อน)
        pos = offset + 6;
    }

    return count;
}
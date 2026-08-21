// shm_common.h
#pragma once
#include <atomic>
#include <cstdint>

// ==================== Shared Memory Name ====================
static const char* SHM_NAME = "/writer_reader_shm";

// ==================== Bit Layout ของ ranwriter (รวม 64 bit) ====================
// [63:48] counter       16 bit  -> จำนวนรอบที่เขียน (wrap ที่ 65536)
// [47:34] value*100     14 bit  -> ค่า value ช่วง 0.00 - 100.00 (ทศนิยม 2 ตำแหน่ง)
// [33:0 ] ts_ms_of_day  34 bit  -> millisecond นับจากเที่ยงคืนของวันนั้น
constexpr int COUNTER_BITS = 16;
constexpr int VALUE_BITS   = 14;
constexpr int TS_BITS      = 34;

// ตำแหน่ง shift ของแต่ละฟิลด์ (นับจาก LSB)
constexpr int COUNTER_SHIFT = VALUE_BITS + TS_BITS; // 48
constexpr int VALUE_SHIFT   = TS_BITS;              // 34

// bit mask สำหรับตัด field แต่ละส่วนหลัง shift
constexpr uint64_t COUNTER_MASK = (1ULL << COUNTER_BITS) - 1;
constexpr uint64_t VALUE_MASK   = (1ULL << VALUE_BITS) - 1;
constexpr uint64_t TS_MASK      = (1ULL << TS_BITS) - 1;

// ==================== Shared Data ====================
// เหลือ field เดียว: ranwriter เก็บ counter+value+timestamp แบบ pack
// เข้าถึงตรงๆ ผ่าน atomic ไม่มีกลไกบังคับจังหวะสลับ อ่าน-เขียน
struct SharedData {
    std::atomic<uint64_t> ranwriter;
};
#ifndef SHM_LAYOUT_H
#define SHM_LAYOUT_H

#include <stdint.h>

#define SHM_CAM_NAME "/navball_cam"
#define SHM_IMU_NAME "/navball_imu"

#define CAM_WIDTH  1280
#define CAM_HEIGHT 720
#define CAM_FRAME_SIZE (CAM_WIDTH * CAM_HEIGHT * 3)

#pragma pack(push, 1)

typedef struct {
    uint64_t frame_seq;
    uint64_t timestamp_ns;
    uint32_t width;
    uint32_t height;
    uint8_t  ready;
    uint8_t  data[CAM_FRAME_SIZE];
} shm_cam_t;

typedef struct {
    uint64_t update_seq;
    uint64_t timestamp_ns;
    float    pitch_deg;
    float    roll_deg;
    float    yaw_deg;
    uint8_t  sensor_ok;
} shm_imu_t;

#pragma pack(pop)

/* Fixed byte offsets - both Go and C sides must use these, not sizeof(),
   to avoid struct padding mismatches between languages/compilers. */
#define CAM_OFF_FRAME_SEQ  0
#define CAM_OFF_TIMESTAMP  8
#define CAM_OFF_WIDTH      16
#define CAM_OFF_HEIGHT     20
#define CAM_OFF_READY      24
#define CAM_OFF_DATA       25
#define SHM_CAM_TOTAL_SIZE (CAM_OFF_DATA + CAM_FRAME_SIZE)

#define IMU_OFF_UPDATE_SEQ 0
#define IMU_OFF_TIMESTAMP  8
#define IMU_OFF_PITCH      16
#define IMU_OFF_ROLL       20
#define IMU_OFF_YAW        24
#define IMU_OFF_SENSOR_OK  28
#define SHM_IMU_TOTAL_SIZE 29

#endif

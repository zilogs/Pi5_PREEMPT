#include "shm_common.h"
#include <cstddef>
#include <cstdio>

int main() {
    printf("sizeof(IMUData)   = %zu\n", sizeof(IMUData));
    printf("sizeof(GPSData)   = %zu\n", sizeof(GPSData));
    printf("sizeof(CamData)   = %zu\n", sizeof(CamData));
    printf("sizeof(RB<IMU>)   = %zu\n", sizeof(RingBuffer<IMUData, RB_SIZE>));
    printf("sizeof(RB<GPS>)   = %zu\n", sizeof(RingBuffer<GPSData, RB_SIZE>));
    printf("sizeof(RB<CAM>)   = %zu\n", sizeof(RingBuffer<CamData, RB_SIZE>));
    printf("sizeof(SharedData)= %zu\n", sizeof(SharedData));
    printf("---\n");
    printf("imu_packet  @ %zu\n", offsetof(SharedData, imu_packet));
    printf("gps_packet  @ %zu\n", offsetof(SharedData, gps_packet));
    printf("cam_packet  @ %zu\n", offsetof(SharedData, cam_packet));
    printf("--- cam_packet internals ---\n");
    printf("cam.head    @ %zu\n", offsetof(SharedData, cam_packet.head));
    printf("cam.tail    @ %zu\n", offsetof(SharedData, cam_packet.tail));
    printf("cam.buffer  @ %zu\n", offsetof(SharedData, cam_packet.buffer));
    printf("cam.buffer[0].ts_ns   @ %zu\n", offsetof(SharedData, cam_packet.buffer[0].ts_ns));
    printf("cam.buffer[0].count   @ %zu\n", offsetof(SharedData, cam_packet.buffer[0].count));
    printf("cam.buffer[1].ts_ns   @ %zu\n", offsetof(SharedData, cam_packet.buffer[1].ts_ns));
    return 0;
}

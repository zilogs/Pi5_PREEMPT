/* mpu6050_rt200_compact.c — 200 Hz RT MPU-6050 Reader with Scope Pin */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <time.h>
#include <gpiod.h>

#define MPU6050_ADDR 0x68
#define REG_ACCEL_XOUT_H 0x3B
#define PERIOD_NS 5000000L // 200 Hz = 5 ms
#define DT 0.005f
#define ALPHA 0.98f

static int i2c_io(int fd, uint8_t reg, uint8_t *buf, int len, int is_read) {
    struct i2c_msg msgs[2] = {
        { .addr = MPU6050_ADDR, .flags = 0, .len = 1, .buf = &reg },
        { .addr = MPU6050_ADDR, .flags = is_read ? I2C_M_RD : 0, .len = len, .buf = buf }
    };
    struct i2c_rdwr_ioctl_data d = { .msgs = msgs, .nmsgs = is_read ? 2 : 1 };
    return ioctl(fd, I2C_RDWR, &d);
}

int main(void) {
// 1. RT Environment Setup (Memlock, CPU Affinity, SCHED_FIFO)
    /*ล็อคระบบให้ทำงานแบบห้ามดีเลย์ (Real-Time Setup) สั่งล็อคแรม (mlockall) และย้ายโปรแกรมไปวิ่งที่ CPU Core 3 แยกต่างหาก (sched_setaffinity) พร้อมให้สิทธิ์ความสำคัญสูงสุด เพื่อให้บอร์ดประมวลผลคำสั่งนี้อย่างต่อเนื่อง ไม่มีอาการกระตุกหรือโดนโปรแกรมอื่นแย่งงาน*/
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) perror("mlockall");
    volatile char stack_touch[65536]; memset((void *)stack_touch, 0, sizeof stack_touch);

    cpu_set_t cpuset; CPU_ZERO(&cpuset); CPU_SET(3, &cpuset); // Isolated Core 3
    if (sched_setaffinity(0, sizeof cpuset, &cpuset) != 0) return perror("affinity"), 1;

    struct sched_param sp = { .sched_priority = 99 };
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) return perror("scheduler"), 1;

// 2. GPIO Setup (libgpiod v2) -> GPIO 17
    /*เปิด-ปิดไฟที่ขาขั้วสัญญาณเพื่อเช็คความเร็ว (Scope Pin) ควบคุมขา GPIO 17 ให้ปล่อยไฟ 5V (HIGH) ตอนเริ่มอ่านค่าเซนเซอร์ และดับไฟ (LOW) ทันทีที่คำนวณเสร็จ เพื่อให้เราเอาเครื่องวัด (Oscilloscope) มาคีบดูได้ว่าโปรแกรมทำงานได้แม่นยำทุกๆ 5 มิลลิวินาที (200 Hz) จริงไหม*/
    struct gpiod_chip *chip = gpiod_chip_open("/dev/gpiochip0");
    if (!chip) return perror("gpio chip"), 1;
    
    struct gpiod_line_config *l_cfg = gpiod_line_config_new();
    struct gpiod_line_settings *l_set = gpiod_line_settings_new();
    gpiod_line_settings_set_direction(l_set, GPIOD_LINE_DIRECTION_OUTPUT);
    gpiod_line_settings_set_output_value(l_set, GPIOD_LINE_VALUE_INACTIVE);
    unsigned int offsets[1] = { 17 };
    gpiod_line_config_add_line_settings(l_cfg, offsets, 1, l_set);
    gpiod_line_settings_free(l_set);

    struct gpiod_request_config *r_cfg = gpiod_request_config_new();
    gpiod_request_config_set_consumer(r_cfg, "scope");
    struct gpiod_line_request *l_req = gpiod_chip_request_lines(chip, r_cfg, l_cfg);
    gpiod_request_config_free(r_cfg); gpiod_line_config_free(l_cfg);
    if (!l_req) return perror("gpio req"), 1;

// 3. MPU-6050 Initialization
    /*อ่านค่าแล้วกรองสัญญาณรบกวน (Sensor Fusion)ดึงค่าความเร่งและมุมหมุนจากชิป MPU-6050 ผ่าน I2C นำมาเข้าสูตรตัวกรอง Complementary Filter (Alpha 0.98) เพื่อหักล้างสัญญาณรบกวน ผลลัพธ์ที่ได้คือค่ามุมองศา (Pitch, Roll, Yaw) ที่นิ่ง แม่นยำ และไม่สั่นไหวครับ*/
    int fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) return perror("i2c open"), 1;
    
    uint8_t init_cmds[][2] = {{0x6B, 0x00}, {0x19, 4}, {0x1A, 0x01}, {0x1B, 0x00}, {0x1C, 0x00}};
    for (int i = 0; i < 5; i++) {
        if (i2c_io(fd, init_cmds[i][0], &init_cmds[i][1], 1, 0) < 0) return perror("mpu init"), 1;
    }

    float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
    struct timespec deadline; clock_gettime(CLOCK_MONOTONIC, &deadline);

// 4. Main Loopmpu_sim
    while (1) {
        deadline.tv_nsec += PERIOD_NS;
        if (deadline.tv_nsec >= 1000000000L) { deadline.tv_sec++; deadline.tv_nsec -= 1000000000L; }

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, NULL);
        gpiod_line_request_set_value(l_req, 17, GPIOD_LINE_VALUE_ACTIVE); // SCOPE HIGH

        uint8_t raw[14];
        if (i2c_io(fd, REG_ACCEL_XOUT_H, raw, 14, 1) < 0) {
            gpiod_line_request_set_value(l_req, 17, GPIOD_LINE_VALUE_INACTIVE);
            continue;
        }

        // Decode & Scale Data = แปลงข้อมูลดิบระดับฮาร์ดแวร์
        float ax = (int16_t)((raw[0] << 8) | raw[1]) / 16384.0f;
        float ay = (int16_t)((raw[2] << 8) | raw[3]) / 16384.0f;
        float az = (int16_t)((raw[4] << 8) | raw[5]) / 16384.0f;
        float gx = (int16_t)((raw[8] << 8) | raw[9]) / 131.0f;
        float gy = (int16_t)((raw[10] << 8) | raw[11]) / 131.0f;
        float gz = (int16_t)((raw[12] << 8) | raw[13]) / 131.0f;

        // Complementary Filter
        float p_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.29578f;
        float r_acc = atan2f(ay, az) * 57.29578f;

        pitch = ALPHA * (pitch + gx * DT) + (1.0f - ALPHA) * p_acc;
        roll  = ALPHA * (roll + gy * DT) + (1.0f - ALPHA) * r_acc;
        yaw  += gz * DT;

        gpiod_line_request_set_value(l_req, 17, GPIOD_LINE_VALUE_INACTIVE); // SCOPE LOW
        (void)pitch; (void)roll; (void)yaw;
    }

    close(fd); gpiod_line_request_release(l_req); gpiod_chip_close(chip);
    return 0;
}
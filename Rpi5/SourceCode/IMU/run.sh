#!/bin/bash

echo "Compiling mpu_2.c..."
gcc -O2 -o mpu_2 mpu_2.c -lm -lgpiod

echo "Running mpu_2..."
sudo ./mpu_2

echo "Compiling mpu6050.c..."
gcc -O2 -o mpu6050 mpu6050.c -lm -lgpiod

echo "Running mpu6050..."
sudo ./mpu6050
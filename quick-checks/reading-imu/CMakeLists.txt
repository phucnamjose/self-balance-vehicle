# Standalone quick-check: read the MPU6050 at 500 Hz and report the achieved rate.
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(reading_imu)

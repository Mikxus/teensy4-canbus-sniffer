cmake_minimum_required(VERSION 3.20.0)
set(BOARD teensy41)

find_package(Zephyr)
project(teensy4-canbus-sniffer)

target_sources(app PRIVATE src/main.c)
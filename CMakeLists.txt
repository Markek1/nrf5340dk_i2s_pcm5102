cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nrf5340dk_i2s_pcm5102)

target_sources(app PRIVATE src/main.c)

# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(spiram_test)

target_sources(app PRIVATE src/main.c)
zephyr_linker_sources(DATA_SECTIONS spiram_usage.ld)

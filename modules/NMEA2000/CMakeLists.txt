# Copyright (c) 2025 Kim Bøndergaard <kim@fam-boendergaard.dk>
#
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_NMEA2000)

set(ZEPHYR_CURRENT_LIBRARY NMEA2000)
set(NMEA2000_DIR ${ZEPHYR_NMEA2000_MODULE_DIR})

zephyr_interface_library_named(NMEA2000)
zephyr_library()
zephyr_include_directories(
    ${NMEA2000_DIR}/src/
    .
)
zephyr_compile_options("-Wno-unused-function")

zephyr_library_sources(
    ${NMEA2000_DIR}/src/N2kGroupFunction.cpp
    ${NMEA2000_DIR}/src/N2kGroupFunctionDefaultHandlers.cpp
    ${NMEA2000_DIR}/src/N2kMsg.cpp
    ${NMEA2000_DIR}/src/N2kMessages.cpp
    ${NMEA2000_DIR}/src/N2kStream.cpp
    ${NMEA2000_DIR}/src/N2kTimer.cpp
    ${NMEA2000_DIR}/src/NMEA2000.cpp
    NMEA2000_zephyr.cpp
)

zephyr_library_link_libraries(NMEA2000)
target_link_libraries(NMEA2000 INTERFACE zephyr_interface)

endif(CONFIG_NMEA2000)

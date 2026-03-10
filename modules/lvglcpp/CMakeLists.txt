# Copyright (c) 2024 Kim Bøndergaard <kim@fam-boendergaard.dk>
#
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_LVGLCPP)

set(ZEPHYR_CURRENT_LIBRARY lvglcpp)
set(LVGLCPP_DIR ${ZEPHYR_LVGLCPP_MODULE_DIR})

zephyr_interface_library_named(LVGLCPP)
#zephyr_library()
zephyr_include_directories(
	${LVGLCPP_DIR}/include/
)

#zephyr_library_link_libraries(LVGLPP)
#target_link_libraries(LVGLPP INTERFACE zephyr_interface)

endif(CONFIG_LVGLCPP)

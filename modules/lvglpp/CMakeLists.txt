# Copyright (c) 2024 Kim Bøndergaard <kim@fam-boendergaard.dk>
#
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_LVGLPP)

set(ZEPHYR_CURRENT_LIBRARY lvglpp)
set(LVGLPP_DIR ${ZEPHYR_LVGLPP_MODULE_DIR})

zephyr_interface_library_named(LVGLPP)
zephyr_library()
zephyr_include_directories(${LVGLPP_DIR}/src/)
zephyr_include_directories(include)
zephyr_compile_options("-Wno-unused-function")

zephyr_library_sources(
    lvgl_zinput.cpp
    ${LVGLPP_DIR}/src/lvglpp/font/font.cpp

    ${LVGLPP_DIR}/src/lvglpp/core/event.cpp
    ${LVGLPP_DIR}/src/lvglpp/core/group.cpp
    ${LVGLPP_DIR}/src/lvglpp/core/theme.cpp
    ${LVGLPP_DIR}/src/lvglpp/core/indev.cpp
    ${LVGLPP_DIR}/src/lvglpp/core/display.cpp
    ${LVGLPP_DIR}/src/lvglpp/core/object.cpp

    ${LVGLPP_DIR}/src/lvglpp/draw/image.cpp
    ${LVGLPP_DIR}/src/lvglpp/draw/mask.cpp
    ${LVGLPP_DIR}/src/lvglpp/draw/desc.cpp

    ${LVGLPP_DIR}/src/lvglpp/misc/timer.cpp
    ${LVGLPP_DIR}/src/lvglpp/misc/anim.cpp
    ${LVGLPP_DIR}/src/lvglpp/misc/area.cpp
    ${LVGLPP_DIR}/src/lvglpp/misc/style.cpp
    ${LVGLPP_DIR}/src/lvglpp/misc/color.cpp
    ${LVGLPP_DIR}/src/lvglpp/misc/fs.cpp

    ${LVGLPP_DIR}/src/lvglpp/widgets/slider/slider.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/spinner/spinner.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/menu/menu.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/btnmatrix/btnmatrix.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/msgbox/msgbox.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/switch/switch.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/colorwheel/colorwheel.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/label/label.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/checkbox/checkbox.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/win/win.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/image/image.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/canvas/canvas.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/textarea/textarea.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/imgbtn/imgbtn.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/line/line.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/roller/roller.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/span/span.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/chart/chart.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/table/table.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/led/led.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/list/list.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/animimg/animimg.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/tileview/tileview.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/keyboard/keyboard.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/dropdown/dropdown.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/calendar/calendar.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/tabview/tabview.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/meter/meter.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/bar/bar.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/arc/arc.cpp
    ${LVGLPP_DIR}/src/lvglpp/widgets/spinbox/spinbox.cpp
)

zephyr_library_link_libraries(LVGLPP)
target_link_libraries(LVGLPP INTERFACE zephyr_interface)

endif(CONFIG_LVGLPP)

.. _ble_bmsim:

Bluetooth: BMS Simulator
########################

Overview
********

Application providing a simple BMS simulator able to produce the messages
necessary to keep the 'Overkill Solar' Android App happy when it comes to
showing data at the front pagge

Currently all data are static. I.e. there will be no emulation of different
cell voltage or different current consumption

Requirements
************

* nrf52840dongle_nrf52840


Building and Running
********************

To build run:

```bash
west build  -b nrf52840dongle_nrf52840 zephyr/samples/bluetooth/bmsim/
```

To flash, nrfutil must be installed. See nrf homepage or Zephyr doc

```bash
nrfutil pkg generate --hw-version 52 --sd-req=0x00 \
    --application build/zephyr/zephyr.hex --application-version 1 bmsim.zip

nrfutil dfu usb-serial -pkg bmsim.zip  -p /dev/ttyACM0
```


Connect to 'Kims BMS Simulator' using the 'Overkill Solar' app

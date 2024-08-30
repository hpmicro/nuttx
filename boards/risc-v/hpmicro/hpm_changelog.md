# PX4 Nuttx With HPM_SDK Change Log

## [v0.1.0] - 20240710

### Version
- PX4 Nuttx: release v10.3.0+
- PX4 Apps: release v10.3.0+
- Hpm_sdk: release v1.4.0

### Added:
- hpm6750evk2-sdk
    - configs: add adc
    - configs: add can
    - configs: add cansock
    - configs: add gpio
    - configs: add i2c_tools
    - configs: add lvgl_lcd
    - configs: add mmcsd
    - configs: add nsh
    - configs: add nx_lcd_demo
    - configs: add pwm
    - configs: add random
    - configs: add tcpecho
    - configs: add timers
    - configs: add usb_dev_cdcacm
    - configs: add usb_dev_msc
    - configs: add usb_host
    - configs: add userled
    - configs: add fpu

- hpm6300evk-sdk
    - configs: add gpio
    - configs: add nsh
    - configs: add fpu
    
- hpm6200evk-sdk
    - configs: add gpio
    - configs: add nsh
    - configs: add fpu

- hpm5300evk-sdk
    - configs: add gpio
    - configs: add nsh
    - configs: add fpu

## [v0.2.0] - 20240821

### Version
- PX4 Nuttx: release v10.3.0+
- PX4 Apps: release v10.3.0+
- Hpm_sdk: release v1.4.0

### Changed
- fix px4 linker problem as HEAD_ASRC is not in libarch.a
- separate non-os heap and os heap
- hpm6750evk2: increase freq to 816MHz and bump up DCDC voltage to 1250mv
- set EXCEPTION_SECTION to .isr_vector section for improve performance
- fix sdmmc several bugs
- fix enet function error

## [v0.2.1] - 20240830

### Version
- PX4 Nuttx: release v10.3.0+
- PX4 Apps: release v10.3.0+
- Hpm_sdk: release v1.4.0

### Changed
- fix enet some errors

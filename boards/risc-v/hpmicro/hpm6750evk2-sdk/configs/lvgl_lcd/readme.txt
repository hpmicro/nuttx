lvgl_lcd
=======
The demo use spi drive st7789 lcd display, st7789 lcd resolution is 240*320,the lcd will display littevgl demo.



You can open terminal check the device,use the cmd: ls /dev.

NuttShell (NSH)
nsh> ls /dev
/dev:
 console
 lcd0
 null
 spi2

st7789 lcd connector             board interface              pin
    GND                              J11[1]                   GND
    VCC                              J11[2]                   3.3V
    DC                               J11[6]                   PZ08
    CS                               J11[7]                   PE31
    CLK                              J11[8]                   PE27
    SDA                              J11[10]                  PE30
    RES                              J11[5]                   PE09
    BLK                              J11[4]                   PE10
Notice:you can access your lcd module(st7789) at the info pin above

Configure NuttX
keep defconfig

Build NuttX
$ make distclean
$ ./tools/configure.sh -l hpm6750evk2-sdk:lvgl_lcd
$ make -j8

Flash the nuttx to the board and run

Open the terminal for interaction
The LCD module will display white screen after power-on, if display error, please check the pin connect.

Example Usage:

1.check spi and lcd bus
NuttShell (NSH)
nsh> ls /dev
/dev:
 console
 lcd0
 null
 spi2

2.run the lvgl demo,and lcd begin display.
nsh> lvgldemo
nsh>

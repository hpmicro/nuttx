nx_lcd_demo
=======
The demo use spi drive st7789 lcd display, st7789 lcd resolution is 240*320,the lcd will display nuttx nx gui demo.

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
$ ./tools/configure.sh -l hpm6750evk2-sdk:nx_lcd_demo
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


2.check builtin apps

nsh> ?
help usage:  help [-v] [<cmd>]

  ?       cd      exec    kill    mount   pwd     uname
  cat     echo    help    ls      printf  sleep   usleep
Builtin Apps:
  nsh      nx       nxhello  sh

3.run the nx demo,and lcd begin display.

nsh> nx
nx_main: NX handle=0x80160
nx_main: Set background color=31581
nx_main: Create window #1
nx_main: hwnd1=0x848e0
nx_listenerthread: Connected
nxeg_position1: hwnd=0x848e0 size=(0,0) pos=(0,4) bounds={(0,0),(239,319)}
nx_main: Screen resolution (239,319)
nx_main: Set window #1 size to (119,159)
nxeg_position2: Have xres=239 yres=319
nx_main: Sleeping

nxeg_position1: hwnd=0x848e0 size=(119,159) pos=(4,4) bounds={(0,0),(239,319)}
nxeg_redraw1: hwnd=0x848e0 rect={(0,0),(118,158)} more=false
nx_main: Set window #1 position to (29,39)
nx_main: Sleeping

nxeg_position1: hwnd=0x848e0 size=(119,159) pos=(29,39) bounds={(0,0),(239,319)}
nxeg_redraw1: hwnd=0x848e0 rect={(0,0),(118,158)} more=false
nx_main: Add toolbar to window #1
nx_main: Sleeping

nxeg_redraw1: hwnd=0x848e0 rect={(0,0),(118,142)} more=false
nxeg_tbredraw1: hwnd=0x848e0 rect={(0,0),(118,15)} more=false
nx_main: Create window #2
nx_main: hwnd2=0x84940
nx_main: Sleeping

nxeg_position2: hwnd=0x84940 size=(0,0) pos=(0,4) bounds={(0,0),(239,319)}
nx_main: Set hwnd2 size to (119,159)
nx_main: Sleeping

nxeg_position2: hwnd=0x84940 size=(119,159) pos=(4,4) bounds={(0,0),(239,319)}
nxeg_redraw2: hwnd=0x84940 rect={(0,0),(118,158)} more=false
nx_main: Set hwnd2 position to (91,121)
nx_main: Sleeping

nxeg_position2: hwnd=0x84940 size=(119,159) pos=(91,121) bounds={(0,0),(239,319)}
nxeg_redraw2: hwnd=0x84940 rect={(0,0),(118,158)} more=false
nxeg_redraw1: hwnd=0x848e0 rect={(0,0),(118,61)} more=false
nxeg_tbredraw1: hwnd=0x848e0 rect={(0,0),(118,15)} more=false
nxeg_redraw1: hwnd=0x848e0 rect={(0,62),(57,142)} more=false
nx_main: Add toolbar to window #2
nx_main: Sleeping

nxeg_redraw2: hwnd=0x84940 rect={(0,0),(118,142)} more=false
nxeg_tbredraw2: hwnd=0x84940 rect={(0,0),(118,15)} more=false
nx_main: Lower window #2
nx_main: Sleeping

nxeg_redraw1: hwnd=0x848e0 rect={(58,62),(118,142)} more=false
nx_main: Raise window #2
nx_main: Sleeping

nxeg_redraw2: hwnd=0x84940 rect={(0,0),(118,142)} more=false
nxeg_tbredraw2: hwnd=0x84940 rect={(0,0),(118,15)} more=false
nx_main: Close window #2
nx_main: Close window #1
nx_main: Disconnect from the server

4.run the nxhello demo,and lcd begin display.
nsh> nxhello
nxhello_main: NX handle=0x84010
nxhello_main: Set background color=31581
nxhello_listener: Connected
nxhello_main: Screen resolution (240,320)
nxhello_hello: Position (87,156)
nxhello_main: Disconnect from the server
nsh>

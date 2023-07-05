userled
=======
This sample shows userled flash function

You can open terminal check the device,use the cmd: ls /dev.

NuttShell (NSH)
nsh> ls /dev
/dev:
 console
 null
 userleds


Configure NuttX
keep defconfig

Build NuttX
$ make distclean
$ ./tools/configure.sh -l hpm6750evk2-sdk:userled
$ make -j8

Flash the nuttx to the board and run

Open the terminal for interaction

Example Usage:

1.Check userled bus

NuttShell (NSH)
nsh> ls /dev
/dev:
 console
 null
 userleds


2.Check builtin apps

nsh> ?
help usage:  help [-v] [<cmd>]

  ?       cd      exec    kill    mount   pwd     uname
  cat     echo    help    ls      printf  sleep   usleep
Builtin Apps:
  leds  nsh   sh


3.Run the userled demo, you can see the board RGB is flash

nsh> leds
leds_main: Starting the led_daemon
leds_main: led_daemon started

led_daemon (pid# 4): Running
led_daemon: Opening /dev/userleds
led_daemon: Supported LEDs 0x07
nsh> led_daemon: LED set 0x01
led_daemon: LED set 0x02
led_daemon: LED set 0x03
led_daemon: LED set 0x04
led_daemon: LED set 0x05
led_daemon: LED set 0x06
led_daemon: LED set 0x07
led_daemon: LED set 0x06
led_daemon: LED set 0x05
led_daemon: LED set 0x04
led_daemon: LED set 0x03
led_daemon: LED set 0x02
led_daemon: LED set 0x01
led_daemon: LED set 0x00
led_daemon: LED set 0x01
led_daemon: LED set 0x02
led_daemon: LED set 0x03
led_daemon: LED set 0x04
led_daemon: LED set 0x05
led_daemon: LED set 0x06


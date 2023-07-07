TCP Echo

Overview
==========================================
This sample shows PC sends TCP data frames to MCU and then MCU sends the data frames back to PC.

HW Connections
==========================================
- Connect a USB port on PC to the PWR DEBUG port on the development board with a USB Type-C cable
- Connect an Ethernet port on PC to a RGMII port or a RMII port on the development board with an Ethernet cable

Configuration & Build NuttX
==========================================
$ make distclean
$ ./tools/configure.sh -l hpm6750evk2-sdk:tcpecho
$ make -j8

Programming
==========================================
Use a programmer to download the executable file(nuttx)


Running
==========================================
NuttShell (NSH)
nsh> tcpecho
Start echo server

Note:
You could use any TCP test tool which supports TCP protocol on PC.





USB MSC DEVICE

Overview
==========================================
This sample shows USB MSC device functions.


HW Connections
==========================================
- Connect a USB port on PC to the PWR DEBUG port on the development board with a USB Type-C cable
- Connect a USB port on PC to USB0 port on the development board with a USB Type-C cable


Configuration & Build NuttX
==========================================
$ make distclean
$ ./tools/configure.sh -l hpm6750evk2-sdk:usb_dev_msc
$ make -j


Programming
==========================================
Use a programmer to download the executable file(nuttx)


Running
==========================================

  NuttShell (NSH)
  nsh>
  nsh>
  nsh>
  nsh> help
  help usage:  help [-v] [<cmd>]

    ?        echo     free     mkfatfs  printf   rmdir    usleep
    cat      env      help     mkrd     ps       sleep
    cd       exec     kill     mount    pwd      uname
    cp       exit     ls       mv       rm       umount

  Builtin Apps:
    msconn  msdis   nsh     sh


When NSH first comes up, you must manually create the RAM disk before exporting it:

a) Create a 64Kb RAM disk at /dev/ram1:
  nsh> mkrd -m 1 -s 512 128

b) Put a FAT file system on the RAM disk:
  nsh> mkfatfs /dev/ram1

c) Connect a USB cable to hpm6750evk2 board USB0 and run:
  nsh> msconn
  mcsonn_main: Creating block drivers
  mcsonn_main: Configuring with NLUNS=1
  mcsonn_main: handle=0x95220
  mcsonn_main: Bind LUN=0 to /dev/ram1
  mcsonn_main: Connected

d) If you saved some file on this small disk you can now run disconnect command:
  nsh> msdis
  msdis: Disconnected

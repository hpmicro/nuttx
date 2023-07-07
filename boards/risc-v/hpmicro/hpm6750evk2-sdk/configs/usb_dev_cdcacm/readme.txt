USB CDCACM DEVICE

Overview
==========================================
This sample shows USB CDCACM device functions.


HW Connections
==========================================
- Connect a USB port on PC to the PWR DEBUG port on the development board with a USB Type-C cable
- Connect a USB port on PC to USB0 port on the development board with a USB Type-C cable


Configuration & Build NuttX
==========================================
$ make distclean
$ ./tools/configure.sh -l hpm6750evk2-sdk:usb_dev_cdcamc
$ make -j


Programming
==========================================
Use a programmer to download the executable file(nuttx)


Running
==========================================

  NuttShell (NSH)
  nsh> help
  help usage:  help [-v] [<cmd>]

    ?       cp      exec    help    mount   ps      rmdir   umount
    cat     echo    exit    kill    mv      pwd     sleep   usleep
    cd      env     free    ls      printf  rm      uname

  Builtin Apps:
    nsh     sercon  serdis  sh
  nsh>

[1] Connect a USB cable to hpm6750evk2 board USB0 and execute `sercon` commands, PC will enumerate a device with a com port.

  nsh> sercon
  sercon: Registering CDC/ACM serial driver
  sercon: Successfully registered the CDC/ACM serial driver

[2] You can detach the serial device with the command 'serdis'.

  nsh> serdis
  serdis: Disconnected





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

  Beacause board does not provide circuitry for control of the "soft connect" USB pullup. As a result, the host PC does not know the USB has been logically connected or disconnected. You have to follow these steps to use USB:

[1] Attach the serial device with the command 'sercon'. 

  1) Start NSH with USB disconnected

  2) enter to 'sercon' command to start the CDC/ACM device
      nsh> sercon
      sercon: Registering CDC/ACM serial driver
      sercon: Successfully registered the CDC/ACM serial driver

  3) Connect the USB device to the host, PC will enumerate a device with a com port.

[2] Detach the serial device with the command 'serdis'. 

  1) Disconnect the USB device from the host

  2) Enter the 'serdis' command
      nsh> serdis
      serdis: Disconnected

Note: Default USB port is USB0.

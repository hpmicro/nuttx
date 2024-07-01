USB HOST

Overview
==========================================
This sample shows USB host functions.


HW Connections
==========================================
- Connect a USB port on PC to the PWR DEBUG port on the development board with a USB Type-C cable
- Connect USB0 to a USB flash disk, keyboard or mouse with a USB Type-C convert Type-A cable


Configuration & Build NuttX
==========================================
$ make distclean
$ ./tools/configure.sh -l hpm6750evk2-sdk:usb_host
$ make -j


Programming
==========================================
Use a programmer to download the executable file(nuttx)


Running
==========================================
[1] Mass Storage Device
-------------------------

If a FAT format USB flash disk is connected, a /dev/sda device will appear.

  nsh> ls /dev
  /dev:
   console
   null
   sda

  nsh> mount -t vfat /dev/sda /mnt/sda
  nsh> ls /mnt/sda
  /mnt/sda:
   System Volume Information/
   20221010_085856.png
   20221010_101553.png
   LOST.DIR/
   20221115_153342.png
   20221115_160823.png
   20221115_160921.png

  nsh> echo "hello hpmicro nuttx!" > /mnt/sda/nuttx.txt
  nsh> cat /mnt/sda/nuttx.txt
  hello hpmicro nuttx!

To prevent data loss, don't forget to un-mount the FLASH drive before removing it:

  nsh> umount /mnt/sda

[2] Keyboard Device
-------------------------

If a (supported) USB keyboard is connected, a /dev/kbda device will appear.

  nsh> ls /dev
  /dev:
   console
   kbda
   null

   nsh> help
   help usage:  help [-v] [<cmd>]
   
     ?       cp      exec    help    mount   ps      rmdir   umount
     cat     echo    exit    kill    mv      pwd     sleep   usleep
     cd      env     free    ls      printf  rm      uname
   
   Builtin Apps:
     hidkbd  nsh     sh
   nsh>
   nsh> hidkbd
   Opening device /dev/kbda
   Device /dev/kbda opened

When Press keyborad, nsh will echo key:

   Normal Press:    r [72]
   Normal Press:    t [74]
   Normal Press:    r [72]
   Normal Press:    t [74]
   Normal Press:    r [72]
   Normal Press:    t [74]
   Normal Press:    t [74]
   Normal Press:    r [72]
   Normal Press:    r [72]
   Normal Press:    s [73]
   Normal Press:    d [64]
   Normal Press:    f [66]

[3] Mouse Device
-------------------------

If a (supported) USB mouse is connected, a /dev/mouse0 device will appear.

  nsh> ls /dev
  /dev:
   console
   mouse0
   null

/dev/mouse0 is a read-only serial device.

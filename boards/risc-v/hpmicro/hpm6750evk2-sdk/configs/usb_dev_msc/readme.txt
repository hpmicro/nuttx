 When NSH first comes up, you must manually create the RAM disk before exporting it:

a) Create a 64Kb RAM disk at /dev/ram1:
  nsh> mkrd -m 1 -s 512 128

b) Put a FAT file system on the RAM disk:
  nsh> mkfatfs /dev/ram1

c) Now the 'msconn' command will connect to the host and
   export /dev/ram0 as the USB logical unit:
  nsh> msconn
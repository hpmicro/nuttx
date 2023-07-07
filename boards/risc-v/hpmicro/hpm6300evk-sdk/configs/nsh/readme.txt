NUTTX NSH HELLO

Overview
==========================================
This sample shows USB NSH hello sample.


HW Connections
==========================================
- Connect a USB port on PC to the PWR DEBUG port on the development board with a USB Type-C cable


Configuration & Build NuttX
==========================================
$ make distclean
$ ./tools/configure.sh -l hpm6300evk-sdk:nsh
$ make -j


Programming
==========================================
Use a programmer to download the executable file(nuttx)


Running
==========================================

  NuttShell (NSH)
  nsh>
  nsh> help
  help usage:  help [-v] [<cmd>]

    ?       cd      env     free    kill    mount   ps      sleep   usleep
    cat     echo    exec    help    ls      printf  pwd     uname

  Builtin Apps:
    hello  nsh    sh

  nsh> hello
  Hello, World!!








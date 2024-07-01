random
=======
This sample shows the random number generation

You can open terminal check the device,use the cmd: ls /dev.

NuttShell (NSH)
nsh> ls /dev
/dev:
 console
 null
 random

Configure NuttX
keep defconfig

Build NuttX
$ make distclean
$ ./tools/configure.sh -l hpm6750evk2-sdk:random
$ make -j8

Flash the nuttx to the board and run

Open the terminal for interaction

Example Usage:

1.Check rand bus

NuttShell (NSH)
nsh> ls /dev
/dev:
 console
 null
 random


2.Check builtin apps

nsh> ?
help usage:  help [-v] [<cmd>]

  ?       cd      env     free    kill    mount   ps      sleep   usleep
  cat     echo    exec    help    ls      printf  pwd     uname
Builtin Apps:
  nsh   rand  sh


3.Run the rand demo,8 random numbers of 32bits will be generated each time and printed to the terminal

nsh> rand
ReRandom values (0x8377c):
0000  df d9 4c ce 23 bf b1 94 1c 69 3a 43 71 cf 62 62  ..L.#....i:Cq.bb
0010  97 2c 49 f2 32 a9 fd 75 13 2d 0d c4 3c ad 96 3d  .,I.2..u.-..<..=
ading 8 random numbers
nsh> rand
ReRandom values (0x8377c):
0000  38 73 80 50 d1 98 a9 0d c2 19 b7 a9 c9 23 e6 b6  8s.P.........#..
0010  23 3c bd 11 73 cf 2e ff b3 12 72 8f 6f 0a d1 c0  #<..s.....r.o...
ading 8 random numbers
nsh> rand
ReRandom values (0x8377c):
0000  67 f6 57 77 c9 38 a4 8e 8b 6c 97 d7 d5 3e 9e ef  g.Ww.8...l...>..
0010  ba 95 67 3e af 9c c5 35 c5 37 d7 74 aa 5f 15 5e  ..g>...5.7.t._.^
ading 8 random numbers
nsh>

NUTTX NSH FPU TEST

Overview
==========================================
This sample shows FPU feature.
This sample will call apps/examples/fpu_test/fpu_test_main.c,
On fpu_test_main.c, main_task will calcaute 'pi' and wait a random delay,
and also will create 2 while_thread and the thread will calaute the 'pi' on while function.

Compile Version
==========================================
flash_xip(ram.ld is small so not support ram version)


Configuration & Build NuttX
==========================================
$ make distclean
$ ./tools/configure.sh -l hpm6300evk-sdk:fpu
$ make -j


Programming
==========================================
Use a programmer to download the executable file(nuttx)


Running
==========================================

  NuttShell (NSH)
  nsh>fpu_test
  fpu_test [2:105]
  ENTER FPU Test!!
  Create 2 while task!!
  while task1 create successfully
  while task2 create successfully
  main_task :count = 1,pi = 3.1415916536
  Sleep time: 0 ms
  main_task :count = 2,pi = 3.1415916536
  Sleep time: 2351 ms
  Enter while task1!
  While task1! count : 1,pi = 3.1415916536
  Enter while task2!
  While task2! count : 1,pi = 3.1415916536
  While task1! count : 2,pi = 3.1415916536
  ...









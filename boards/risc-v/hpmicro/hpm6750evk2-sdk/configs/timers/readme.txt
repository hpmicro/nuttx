timers
=======
This sample shows timing function of the timer

You can open terminal check the device,use the cmd: ls /dev.hpm6750evk2 have 8 timer number.

NuttShell (NSH)
nsh> ls /dev
/dev:
 console
 null
 timer0
 timer1
 timer2
 timer3
 timer4
 timer5
 timer6
 timer7

Configure NuttX
keep defconfig
default CONFIG_EXAMPLES_TIMER_DELAY value is 1000000.ti's unit is us

Build NuttX
$ make distclean
$ ./tools/configure.sh -l hpm6750evk2-sdk:timers
$ make -j8

Flash the nuttx to the board and run

Open the terminal for interaction

Example Usage:

1.Check timer bus

NuttShell (NSH)
nsh> ls /dev
/dev:
 console
 null
 timer0
 timer1
 timer2
 timer3
 timer4
 timer5
 timer6
 timer7


2.Check builtin apps

nsh> ?
help usage:  help [-v] [<cmd>]

  ?       cd      exec    kill    mount   pwd     uname
  cat     echo    help    ls      printf  sleep   usleep
Builtin Apps:
  nsh    sh     timer



3.You can get help by typing the timer command

nsh> timer -h
Usage: timer [-d /dev/timerx]
nsh>


4.Run the timer demo, it's will print the timer timing in the specified number of times(CONFIG_EXAMPLES_TIMER_NSAMPLES)
Notice: default timer is /dev/timer0

nsh> timer
Open /dev/timer0
  flags: 00000000 timeout: 0 timeleft: 0 nsignals: 0
Set timer interval to 1000000
  flags: 00000000 timeout: 1000000 timeleft: 1000000 nsignals: 0
Attach timer handler
  flags: 00000002 timeout: 1000000 timeleft: 1000000 nsignals: 0
Start the timer
  flags: 00000003 timeout: 1000000 timeleft: 999864 nsignals: 1
  flags: 00000003 timeout: 1000000 timeleft: 999928 nsignals: 2
  flags: 00000003 timeout: 1000000 timeleft: 999957 nsignals: 3
  flags: 00000003 timeout: 1000000 timeleft: 999956 nsignals: 4
  flags: 00000003 timeout: 1000000 timeleft: 999963 nsignals: 5
  flags: 00000003 timeout: 1000000 timeleft: 999962 nsignals: 6
  flags: 00000003 timeout: 1000000 timeleft: 999954 nsignals: 7
  flags: 00000003 timeout: 1000000 timeleft: 999962 nsignals: 8
  flags: 00000003 timeout: 1000000 timeleft: 999965 nsignals: 9
  flags: 00000003 timeout: 1000000 timeleft: 999966 nsignals: 10
  flags: 00000003 timeout: 1000000 timeleft: 999964 nsignals: 11
  flags: 00000003 timeout: 1000000 timeleft: 999965 nsignals: 12
  flags: 00000003 timeout: 1000000 timeleft: 999968 nsignals: 13
  flags: 00000003 timeout: 1000000 timeleft: 999967 nsignals: 14
  flags: 00000003 timeout: 1000000 timeleft: 999968 nsignals: 15
  flags: 00000003 timeout: 1000000 timeleft: 999969 nsignals: 16
  flags: 00000003 timeout: 1000000 timeleft: 999966 nsignals: 17
  flags: 00000003 timeout: 1000000 timeleft: 999967 nsignals: 18
  flags: 00000003 timeout: 1000000 timeleft: 999969 nsignals: 19
  flags: 00000003 timeout: 1000000 timeleft: 999965 nsignals: 20
Stop the timer
  flags: 00000000 timeout: 1000000 timeleft: 999856 nsignals: 20
Finished


5. Use specified timer to timing

nsh> timer -d /dev/timer6
Open /dev/timer6
  flags: 00000000 timeout: 0 timeleft: 0 nsignals: 20
Set timer interval to 1000000
  flags: 00000000 timeout: 1000000 timeleft: 1000000 nsignals: 20
Attach timer handler
  flags: 00000002 timeout: 1000000 timeleft: 1000000 nsignals: 0
Start the timer
  flags: 00000003 timeout: 1000000 timeleft: 999875 nsignals: 1
  flags: 00000003 timeout: 1000000 timeleft: 999940 nsignals: 2
  flags: 00000003 timeout: 1000000 timeleft: 999947 nsignals: 3
  flags: 00000003 timeout: 1000000 timeleft: 999944 nsignals: 4
  flags: 00000003 timeout: 1000000 timeleft: 999953 nsignals: 5
  flags: 00000003 timeout: 1000000 timeleft: 999951 nsignals: 6
  flags: 00000003 timeout: 1000000 timeleft: 999954 nsignals: 7
  flags: 00000003 timeout: 1000000 timeleft: 999958 nsignals: 8
  flags: 00000003 timeout: 1000000 timeleft: 999954 nsignals: 9
  flags: 00000003 timeout: 1000000 timeleft: 999953 nsignals: 10
  flags: 00000003 timeout: 1000000 timeleft: 999945 nsignals: 11
  flags: 00000003 timeout: 1000000 timeleft: 999947 nsignals: 12
  flags: 00000003 timeout: 1000000 timeleft: 999949 nsignals: 13
  flags: 00000003 timeout: 1000000 timeleft: 999936 nsignals: 14
  flags: 00000003 timeout: 1000000 timeleft: 999948 nsignals: 15
  flags: 00000003 timeout: 1000000 timeleft: 999951 nsignals: 16
  flags: 00000003 timeout: 1000000 timeleft: 999955 nsignals: 17
  flags: 00000003 timeout: 1000000 timeleft: 999952 nsignals: 18
  flags: 00000003 timeout: 1000000 timeleft: 999951 nsignals: 19
  flags: 00000003 timeout: 1000000 timeleft: 999950 nsignals: 20
Stop the timer
  flags: 00000000 timeout: 1000000 timeleft: 999830 nsignals: 20
Finished
nsh>

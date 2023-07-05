rtc
=======
This sample shows the rtc alram

You can open terminal check the device,use the cmd: ls /dev.

NuttShell (NSH)
nsh> ls /dev
/dev:
 console
 null
 rtc0


Configure NuttX
keep defconfig

Build NuttX
$ make distclean
$ ./tools/configure.sh -l hpm6750evk2-sdk:rtc
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
 rtc0


2.Check builtin apps

nsh> ?
help usage:  help [-v] [<cmd>]

  ?       cd      exec    kill    mount   pwd     uname
  cat     echo    help    ls      printf  sleep   usleep
Builtin Apps:
  alarm  nsh    sh


3.You can get help by typing the alarm command

nsh> alarm
USAGE:
        alarm [-a <alarmid>] [-cr] [<seconds>]
Where:
        -a <alarmid>
                <alarmid> selects the alarm: 0..1 (default: 0)
        -c      Cancel previously set alarm
        -r      Read previously set alarm
        <seconds>
                The number of seconds until the alarm expires.
                (only if no -c or -r option given.)
nsh>


4.Run the alarm demo,set the number of seconds until the alarm expires.and when the time is uo,it will warn.

nsh> alarm -a 0 5
Opening /dev/rtc0
Alarm 0 set in 5 seconds
nsh> alarm_daemon: alarm 0 received

nsh>
nsh> alarm -a 1 3
Opening /dev/rtc0
Alarm 1 set in 3 seconds
nsh> alarm_daemon: alarm 1 received

nsh>


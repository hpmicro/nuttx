pwm
=======
This sample shows the pwm output function

You can open terminal check the device,use the cmd: ls /dev.

NuttShell (NSH)
nsh> ls /dev
/dev:
 console
 null
 pwm2


PWM2 channel             board interface               pin
    CH1                      J10[13]                   PD30
    CH2                      J10[12]                   PE04
    CH3                      J10[11]                   PE03
    CH4                      J10[10]                   PD29
    CH5                      J10[9]                    PD28

Notice:since the PWM pins is shared with the ETH pins, it's necessary to disconnect the relevant resistors.
that is, disconnect all the resistors at the silk screen of the GigE POP

Configure NuttX
keep defconfig

Build NuttX
$ make distclean
$ ./tools/configure.sh -l hpm6750evk2-sdk:pwm
$ make -j8

Flash the nuttx to the board and run

Open the terminal for interaction

Example Usage:

1.Check pwm bus

NuttShell (NSH)
nsh> ls /dev
/dev:
 console
 null
 pwm2


2.Check builtin apps

nsh> ?
help usage:  help [-v] [<cmd>]

  ?       cd      exec    kill    mount   pwd     uname
  cat     echo    help    ls      printf  sleep   usleep
Builtin Apps:
  nsh  pwm  sh


3.Run the pwm demo,use an oscilloscope to connect to the PWM pins to view the pwm_setup_waveform.
after 5s, the waveform output will automatically stop.

nsh> pwm
pwm_main: starting output with frequency: 10000 channel: 1 duty: 00008000 channel: 2 duty: 00008000 channel: 3 duty: 00008000 channel: 4 duty: 00008000 channel: 5 duty: 00008000
pwm_main: stopping output
nsh>

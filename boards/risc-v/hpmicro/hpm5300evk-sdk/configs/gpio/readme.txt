gpio
=======
For the input/output function of GPIO in this sample, you can verify interrupt input and input using buttom 
verify output using LEDS

You can open terminal check the device,use the cmd: ls /dev

nsh> ls /dev
/dev:
  console
  gpio0
  gpio1

device             board interface                use                    pin
 gpio0   <----->      LED2(LED_RED)   <----->    output     <----->      PA23
 gpio1   <----->      USER_KEY(sw5)   <----->   interrupt   <----->      PA09

Configure NuttX
keep defconfig

Build NuttX
$ make distclean
$ ./tools/configure.sh -l hpm5300evk-sdk:gpio
$ make -j

Flash the nuttx to the board and run

Open the terminal for interaction

about how to use help cmd: gpio -h

nsh> gpio -h
USAGE: gpio [-t <pintype>] [-w <signo>] [-o <value>] <driver-path>
       gpio -h
Where:
        <driver-path>: The full path to the GPIO pin driver.
        -t <pintype>:  Change the pin to this pintype (0-10):
        -w <signo>:    Wait for a signal if this is an interrupt pin.
        -o <value>:    Write this value (0 or 1) if this is an output pin.
        -h: Print this usage information and exit.
Pintypes:
         0: GPIO_INPUT_PIN
         1: GPIO_INPUT_PIN_PULLUP
         2: GPIO_INPUT_PIN_PULLDOWN
         3: GPIO_OUTPUT_PIN
         4: GPIO_OUTPUT_PIN_OPENDRAIN
         5: GPIO_INTERRUPT_PIN
         6: GPIO_INTERRUPT_HIGH_PIN
         7: GPIO_INTERRUPT_LOW_PIN
         8: GPIO_INTERRUPT_RISING_PIN
         9: GPIO_INTERRUPT_FALLING_PIN
        10: GPIO_INTERRUPT_BOTH_PIN

Example Usage:

1.dev/gpio0 output. this device maps LED RED. You can turn it on and off, set 0 is on, set 1 is off.

nsh> gpio -o 1 dev/gpio0
Driver: dev/gpio0
  Output pin:    Value=0
  Writing:       Value=1
  Verify:        Value=1
nsh>

nsh> gpio -o 0 dev/gpio0
Driver: dev/gpio0
  Output pin:    Value=1
  Writing:       Value=0
  Verify:        Value=0
nsh>

2.dev/gpio1 interrupt input.
input the following command line in terminal When you press the button(USER KEY),the log is:

nsh> gpio -w 0 dev/gpio1
Driver: dev/gpio1
  Interrupt pin: Value=1
  [Five second timeout with no signal]
nsh> gpio -w 1 dev/gpio1
Driver: dev/gpio1
  Interrupt pin: Value=1
  Verify:        Value=0
nsh>


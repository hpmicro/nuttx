ADC

Overview
==========================================
This sample shows ADC conversions and results.

HW Connections
==========================================
Instance    Channel   Component   Pin
ADC0/1/2    CH7       J10-8       PE21
ADC0/1/2    CH10      J10-7       PE24
ADC0/1/2    CH11      J10-6       PE25
ADC3        CH2       J10-5       PE29

Configuration & Build NuttX
==========================================
$ make distclean
$ ./tools/configure.sh -l hpm6750evk2-sdk:adc
$ make -j8

Programming
==========================================
Use a programmer to download the executable file(nuttx)


Running
==========================================

NuttShell (NSH)
nsh> ls /dev
/dev:
 adc0
 console
 null
nsh> adc
adc_madc_ioctl: oneshot mode: read result
adc_read: buflen: 5
adc_read: Returning: 5
ain: g_adcstate.count: 1
adc_main: Hardware initialized. Opening the ADC device: /dev/adc0
Sample:
1: channel: 11 value: 4095

Note:
If you select adc1, adc2 or adc3, you firstly need to change the device path as follows: 
(1) For adc1: nsh> adc -p /dev/adc1
(2) For adc2: nsh> adc -p /dev/adc2
(3) For adc3: nsh> adc -p /dev/adc3


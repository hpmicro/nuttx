i2c tools
=======
The i2c tools contains many debugging tools related to i2c bus, such as reading and writing, detecting i2c devices,and so on.

Notice: the sample verify using the IMU module,it's name is GY-85 module,but you can using other module.
the I2C dev command may have bad side effects on your I2C devices.Use only at your own risk.


You can open terminal check the device,use the cmd: ls /dev.

NuttShell (NSH)
nsh> ls /dev
/dev:
 console
 i2c0
 null

device             board interface               use                    pin
                       J11[3]                  I2C0.SCL                 PZ11
 i2c0    <----->       
                       J11[4]                  I2C0.SDA                 PZ10
Notice:you can access your i2c module at the info pin above

Configure NuttX
keep defconfig

Build NuttX
$ make distclean
$ ./tools/configure.sh -l hpm6750evk2-sdk:i2c_tools
$ make -j8

Flash the nuttx to the board and run

Open the terminal for interaction

about how to use help cmd: gpio -h

nsh> i2c help
Usage: i2c <cmd> [arguments]
Where <cmd> is one of:

  Show help     : ?
  List buses    : bus
  List devices  : dev [OPTIONS] <first> <last>
  Read register : get [OPTIONS] [<repetitions>]
  Dump register : dump [OPTIONS] [<num bytes>]
  Show help     : help
  Write register: set [OPTIONS] <value> [<repetitions>]
  Verify access : verf [OPTIONS] [<value>] [<repetitions>]

Where common "sticky" OPTIONS include:
  [-a addr] is the I2C device address (hex).  Default: 01 Current: 01
  [-b bus] is the I2C bus number (decimal).  Default: 0 Current: 0
  [-w width] is the data width (8 or 16 decimal).  Default: 8 Current: 8
  [-s|n], send/don't send start between command and data.  Default: -n Current: -n
  [-i|j], Auto increment|don't increment regaddr on repetitions.  Default: NO Current: NO
  [-f freq] I2C frequency.  Default: 400000 Current: 400000

Special non-sticky options:
  [-r regaddr] is the I2C device register index (hex).  Default: not used/sent

NOTES:
o An environment variable like $PATH may be used for any argument.
o Arguments are "sticky". For example, once the I2C address is
  specified, that address will be re-used until it is changed.

WARNING:
o The I2C dev command may have bad side effects on your I2C devices.
  Use only at your own risk.


Example Usage:

1.detect i2c bus
nsh> i2c bus
 BUS   EXISTS?
 Bus 0: YES
 Bus 1: NO
 Bus 2: NO
 Bus 3: NO

2.detect i2c device devices.it's will print all the i2c devices.

nsh> i2c dev 00 7f
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- 1e --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: 50 -- -- 53 -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
nsh>

3.read i2c device register value

nsh> i2c get -a 1e -b 0 -r 0
READ Bus: 0 Addr: 1e Subaddr: 00 Value: 10
nsh> i2c get -a 50 -b 0 -r 0
READ Bus: 0 Addr: 50 Subaddr: 00 Value: ff
nsh> i2c get -a 53 -b 0 -r 0
READ Bus: 0 Addr: 53 Subaddr: 00 Value: e5
nsh> i2c get -a 68 -b 0 -r 0
READ Bus: 0 Addr: 68 Subaddr: 00 Value: 1a
nsh> i2c get -a 53 -b 0 -r 40
READ Bus: 0 Addr: 53 Subaddr: 40 Value: e5
nsh>

4.write i2c device register value
nsh> i2c get -a 53 -r 31
READ Bus: 0 Addr: 53 Subaddr: 31 Value: 00
nsh> i2c set -a 53 -r 31 0b
WROTE Bus: 0 Addr: 53 Subaddr: 31 Value: 0b
nsh> i2c get -a 53 -r 31
READ Bus: 0 Addr: 53 Subaddr: 31 Value: 0b

5.verify i2c device access
nsh> i2c verf -a 53 -r 31 00
VERIFY Bus: 0 Addr: 53 Subaddr: 31 Wrote: 00 Read: 00
nsh> i2c verf -a 53 -r 31 0b
VERIFY Bus: 0 Addr: 53 Subaddr: 31 Wrote: 0b Read: 0b


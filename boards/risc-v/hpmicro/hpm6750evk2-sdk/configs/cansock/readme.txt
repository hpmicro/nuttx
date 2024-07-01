cansock
=======
For the transceiver function use socket of CAN in this sample, you can connect the CAN analyzer to the J17(CAN) interface
on the board,and then use the CAN analyzer host computer to test.

Board interface(J17)          CAN analyzer
      GND            <----->       GND
      CANL           <----->       CANL
      CANH           <----->       CANH

Configure NuttX
the CAN socket device support standard can and canfd, so only a maximum of 64 bytes can be received and send.
the sample use the cantools,it included cansend and candump.cansend tool is used to send can/canfd frames,
and candump tool is used to received can/canfd frames
if you want change baudrate,you can make menuconfig:
-> System Type                                                                                                                                                                                                                                                                                                                                                                                                      │  
  -> HPMICRO Peripheral Selection                                                                                                                                                                                                                                                                                                                                                                                   │  
    -> CAN0 (HPM_CAN0 [=y])                                                                                                                                                                                                                                                                                                                                                                                         │  
      -> CAN0 device driver options                                                                                                                                                                                                                                                                                                                                                                                 │  
        -> (500000)CAN0_BAUDRATE
        -> (5000000) CAN0_FD_BAUDRATE

Build NuttX
$ make distclean
$ ./tools/configure.sh -l hpm6750evk2-sdk:cansock
$ make -j8

Flash the nuttx to the board and run

Open the terminal for interaction


Example Usage:

1. find net cansock device
You can enter this command(?) to get help. the apps included netlink_route

nsh> ?
help usage:  help [-v] [<cmd>]

  ?       cd      exec    kill    mount   pwd     uname
  cat     echo    help    ls      printf  sleep   usleep
Builtin Apps:
  candump        cansend        netlink_route  nsh            sh

so you can enter this command(netlink_route), it's can find net devices
Index: it's means net device Interface index. Zero is reserved to mean no-index in the POSIX standards.

nsh>
nsh> netlink_route

Device List (Entries: 1)
  Index:  1  Name: "can0"


2. use cansend tool
you can enter this command(cansend ?) to get help

nsh> cansend ?
cansend - send CAN-frames via CAN_RAW sockets.
Usage: cansend <device> <can_frame>.
<can_frame>:
 <can_id>#{data}          for 'classic' CAN 2.0 data frames
 <can_id>#R{len}          for 'classic' CAN 2.0 data frames
 <can_id>##<flags>{data}  for CAN FD frames
<can_id>:
 3 (SFF) or 8 (EFF) hex chars
{data}:
 0..8 (0..64 CAN FD) ASCII hex-values (optionally separated by '.')
{len}:
 an optional 0..8 value as RTR frames can contain a valid dlc field
<flags>:
 a single ASCII Hex value (0 .. F) which defines canfd_frame.flags
Examples:
  5A1#11.2233.44556677.88 / 123#DEADBEEF / 5AA# / 123##1 / 213##311223344 /
  1F334455#1122334455667788 / 123#R / 00000123#R3

ex: send can frames(4 bytes)
nsh> cansend can0 123#12332233
nsh>

ex: send canfd frames(12 bytes)
nsh> cansend can0 333##1112233445566778899112233
nsh>


3.use candump tool
you can enter this command(candump) to get help

nsh> candump
candump - dump CAN bus traffic.
Usage: candump [options] <CAN interface>+
  (use CTRL-C to terminate candump)
Options:
         -t <type>   (timestamp: (a)bsolute/(d)elta/(z)ero/(A)bsolute w date)
         -H          (read hardware timestamps instead of system timestamps)
         -c          (increment color mode level)
         -i          (binary output - may exceed 80 chars/line)
         -a          (enable additional ASCII output)
         -S          (swap byte order in printed CAN data[] - marked with '`' )
         -s <level>  (silent mode - 0: off (default) 1: animation 2: silent)
         -l          (log CAN-frames into file. Sets '-s 2' by default)
         -L          (use log file format on stdout)
         -n <count>  (terminate after reception of <count> CAN frames)
         -r <size>   (set socket receive buffer to <size>)
         -D          (Don't exit if a "detected" can device goes down.
         -d          (monitor dropped CAN frames)
         -e          (dump CAN error frames in human-readable format)
         -x          (print extra message infos, rx/tx brs esi)
         -T <msecs>  (terminate after <msecs> without any reception)
Up to 16 CAN interfaces with optional filter sets can be specified
on the commandline in the form: <ifname>[,filter]*

ex: received can/canfd frames
nsh> candump can0
  can0  123  [08]  22 02 03 04 05 06 07 33
  can0  123  [01]  22
  can0  123  [12]  22 00 00 00 00 00 00 00 00 00 00 00
  can0  123  [20]  22 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  can0  123  [48]  22 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  can0  123  [64]  22 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00



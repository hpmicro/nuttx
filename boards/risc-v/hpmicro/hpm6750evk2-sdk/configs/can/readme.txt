can
=======
For the transceiver function of CAN in this sample, you can connect the CAN analyzer to the J17(CAN) interface
on the board,and then use the CAN analyzer host computer to test.

Board interface(J17)          CAN analyzer
      GND            <----->       GND
      CANL           <----->       CANL
      CANH           <----->       CANH

Configure NuttX
the CAN char device only support standard can, so only a maximum of 8 bytes can be received.

if you want change baudrate,you can make menuconfig:
-> System Type                                                                                                                                                                                                                                                                                                                                                                                                      │  
  -> HPMICRO Peripheral Selection                                                                                                                                                                                                                                                                                                                                                                                   │  
    -> CAN0 (HPM_CAN0 [=y])                                                                                                                                                                                                                                                                                                                                                                                         │  
      -> CAN0 device driver options                                                                                                                                                                                                                                                                                                                                                                                 │  
        -> (500000)CAN0_BAUDRATE


Build NuttX
$ make distclean
$ ./tools/configure.sh -l hpm6750evk2-sdk:can
$ make -j8

Flash the nuttx to the board and run

Open the terminal for interaction

Example Usage:

NuttShell (NSH)
nsh> ls /dev
/dev:
 can0
 console
 null
nsh> can
  nmsgs: 0
  min ID: 1 max ID: 536870911
  Bit timing:
  Baud: 41248
  TSEG1: 7
  TSEG2: 0
  SJW: 0
  ID:    1 DLC: 1
  ID:   18 DLC: 8
  Data received:
  0: 0x00
  1: 0x00
  2: 0x00
  3: 0x00
  4: 0x00
  5: 0x00
  6: 0x00
  7: 0x00
  ID:    2 DLC: 1
  ID:   18 DLC: 8
  Data received:
  0: 0x00
  1: 0x00
  2: 0x00
  3: 0x00
  4: 0x00
  5: 0x00
  6: 0x00
  7: 0x00
  ID:    3 DLC: 1



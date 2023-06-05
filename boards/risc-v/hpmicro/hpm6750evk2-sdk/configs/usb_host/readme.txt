1. Mass Storage Device Usage
-------------------------

Example Usage:

  NuttShell (NSH)
  nsh> ls /dev
   /dev:
   console
   null

Here a USB FLASH stick is inserted.  Nothing visible happens in the
shell.  But a new device will appear:

  nsh> ls /dev
  /dev:
   console
   null
   sda

  nsh> mount -t vfat /dev/sda /mnt/sda
  nsh> ls -l /mnt/sda


2. HID Keyboard Usage
------------------

If a (supported) USB keyboard is connected, a /dev/kbda device will appear:

  nsh> ls /dev
  /dev:
   console
   kbda
   null

/dev/kbda is a read-only serial device.


3. Mouse Usage
------------------

If a (supported) USB mouse is connected, a /dev/mouse0 device will appear:

  nsh> ls /dev
  /dev:
   console
   mouse0
   null

/dev/mouse0 is a read-only serial device.
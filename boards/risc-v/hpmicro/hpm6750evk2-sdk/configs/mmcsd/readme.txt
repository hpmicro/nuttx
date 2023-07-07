[Preparation]
Before running this exmaple, users need to ensure that:

- A microSD card (formatted with FAT32 filesystem) has been inserted into the SD slot.


[Run Example]
When the NSH mmcsd example starts, the sample prints out the mmcsd critical information as below:

    hpm_sdmmc_registercallback: Register 0x8000c1dc(0x83700)
    hpm_sdmmc_callbackenable: eventset: 02
    sdio_mediachange: cdstatus OLD: 00 NEW: 01
    hpm_sdmmc_callback: Callback 0x8000c1dc(0x83700) cbevents: 02 cdstatus: 01
    hpm_sdmmc_callback: Callback to 0x8000c1dc(0x83700)
    hpm_sdmmc_callbackenable: eventset: 01

Then the example runs into the normal NSH shell:

    NuttShell (NSH)
    nsh>
Now using the follow command, users can mount the SD card.

    mount -t vfat /dev/mmcsd1 /sd1


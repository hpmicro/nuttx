1. Verification environment

  Ubuntu/Bash 22.04 LTS shell under Windows 11.

2. Prerequisites

  Run the following command to install packages:

    sudo apt install \
    bison flex gettext texinfo libncurses5-dev libncursesw5-dev xxd \
    gperf automake libtool pkg-config build-essential gperf genromfs \
    libgmp-dev libmpc-dev libmpfr-dev libisl-dev binutils-dev libelf-dev \
    libexpat-dev gcc-multilib g++-multilib picocom u-boot-tools util-linux \
    kconfig-frontends curl

3. Download and install toolchain

  Download URL: https://github.com/hpmicro/riscv-gnu-toolchain/releases/tag/2022.05.15

  Please extract to "~/Toolchain" folder, then add the following statement to the last line of "~/.bashrc" file.

    export PATH=$PATH:~/Toolchain/riscv32-unknown-elf-newlib-multilib_2022.05.15_linux/riscv32-unknown-elf-newlib

4. Download and install openocd

  Download URL: https://github.com/hpmicro/riscv-openocd/releases/tag/hpm_xpi_v0.2.0

  Please extract to "~/Toolchain" folder, then add the following statement to the last line of "~/.bashrc" file.

    export PATH=$PATH:~/Toolchain/openocd-linux

5. Fix CH340 can't identify by Ubuntu 22.04 LTS

  $ sudo systemctl stop brltty-udev.service
  $ sudo systemctl mask brltty-udev.service
  $ sudo systemctl stop brltty.service
  $ sudo systemctl disable brltty.service

6. Git clone hpmicro nuttx

  $ git clone --recursive https://github.com/hpmicro/nuttx_hpmicro.git

  We add folders in those path.
  [1] nuttx_hpmicro/nuttx/arch/risc-v/include/hpmicro
    - This folder is related to irq nums.
  [2] nuttx_hpmicro/nuttx/arch/risc-v/src/hpmicro
    - This folder is a  "lower half" layer which is typically hardware-specific, has hpm_sdk and nuttx driver adapter.
  [3] nuttx_hpmicro/nuttx/boards/risc-v/hpmicro/hpmXXX-sdk
    - This folder is an "upper half" layer which registers driver to NuttX using a call such as **register_driver()** or **register_blockdriver()**.

7. Build nuttx

  Use hpm6750evk2 and nsh as a example.

  In the path: nuttx_hpmicro/nuttx

  $ make distclean
  $ ./tools/configure.sh -l hpm6750evk2-sdk:nsh
  $ make V=1

  note:
  [1] You can use "make menuconfig" to make any modifications to the installed ".config" file. Then you can do "make savedefconfig" to generate a new defconfig file that includes your modifications.
  [2] Default linker file is flash_xip.ld, you can config to ram.ld or flash_sdram_xip.ld.

8. Flash the nuttx with openocd and run

  Use hpm6750evk2 and ireDAP debugger as a example.

  Start openocd in the path: /nuttx/arch/risc-v/src/hpmicro/hpm_sdk/boards/openocd.
    $ openocd -f probes/cmsis_dap.cfg -f soc/hpm6750-single-core.cfg -f boards/hpm6750evk2.cfg

  Start picocom.
    $ sudo picocom -b 115200 /dev/ttyUSB0

  Start debug command as follows.
    $ riscv32-unknown-elf-gdb ./nuttx
    (gdb) target remote localhost:3333
    (gdb) mon reset halt
    (gdb) load
    (gdb) c

  Picocom console:
    ```
    NuttShell (NSH)
    nsh> 
    nsh> help
    help usage:  help [-v] [<cmd>]

        ?         echo      free      ls        ps        uname
        cat       env       help      mount     pwd       usleep
        cd        exec      kill      printf    sleep

    Builtin Apps:
        hello    nsh      sh
    nsh> hello
    Hello, World!!
    ```
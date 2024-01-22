1. Verification environment

  Ubuntu/Bash 22.04 LTS shell under Windows 11.

2. Prerequisites

  Run the following command to install packages:

    sudo apt install \
    bison flex gettext texinfo libncurses5-dev libncursesw5-dev xxd \
    gperf automake libtool pkg-config build-essential gperf genromfs \
    libgmp-dev libmpc-dev libmpfr-dev libisl-dev binutils-dev libelf-dev \
    libexpat-dev gcc-multilib g++-multilib picocom u-boot-tools util-linux \
    kconfig-frontends picocom curl

3. Download and install toolchain

  Download URL: https://github.com/hpmicro/riscv-gnu-toolchain/releases/tag/2022.05.15

  Please extract to "~/Toolchain" folder, then add the following statement to the last line of "~/.bashrc" file.

    export PATH=$PATH:~/Toolchain/riscv32-unknown-elf-newlib-multilib_2022.05.15_linux/riscv32-unknown-elf-newlib

4. Download and install openocd

  Download hpmicro sdk_env, openocd in the path: sdk_env/tools/openocd. This is windows version.

5. Configure and build NuttX

  $ mkdir ./nuttxspace
  $ cd ./nuttxspace
  $ git clone https://github.com/hpmicro/nuttx.git nuttx
  $ git clone https://github.com/apache/nuttx-apps.git apps
  $ cd nuttx
  $ git checkout -b nuttx_v12.4_with_hpmsdk origin/nuttx_v12.4_with_hpmsdk
  $ cd ../apps
  $ git checkout -b releases/12.4 origin/releases/12.4
  $ cd ../nuttx
  $ make distclean
  $ ./tools/configure.sh -l hpm6750evk2-sdk:nsh
  $ make V=1

  note:
  [1] The nuttx and apps version is releases/12.4.
  [2] You can use "make menuconfig" to make any modifications to the installed ".config" file. Then you can do "make savedefconfig" to generate a new defconfig file that includes your modifications.
  [3] Default linker file is flash_xip.ld, you can config to ram.ld or flash_sdram_xip.ld.

6. Flash the nuttx with openocd and run

  The openocd cfg files in the path: sdk_env/hpm_sdk/boards/openocd. If you use fireDAP debugger, in windows shell, start openocd command as follows.
    $ openocd -f probes/cmsis-dap.cfg -f soc/hpm6750-single-core.cfg -f boards/hpm6750evk2.cfg

  In Linux shell, start picocom.
    $ sudo picocom -b 115200 /dev/ttyUSB1

  In Linux shell, start debug command as follows.
    $ riscv32-unknown-elf-gdb ./nuttx
    (gdb) target remote [windows_ip_addr]:3333
    (gdb) load
    (gdb) c

7. HPMicro Source

  We add folders in those path.
  [1] [path to nuttxspace]/nuttx/arch/risc-v/include/hpmicro
    - This folder is related to irq nums.

  [2] [path to nuttxspace]/nuttx/arch/risc-v/src/hpmicro
    - This folder is a  "lower half" layer which is typically hardware-specific, has hpm_sdk and nuttx driver adapter.

  [3] [path to nuttxspace]/nuttx/boards/risc-v/hpmicro/hpm6750evk2-sdk
    - This folder is an "upper half" layer which registers driver to NuttX using a call such as **register_driver()** or **register_blockdriver()**.

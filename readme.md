Read the documentation in qemu-doc.html or on http://wiki.qemu-project.org

- QEMU team

========

This is a fork of [gnuarmeclipse's](http://sourceforge.net/projects/gnuarmeclipse/files/QEMU/) QEMU repository. Their repository adds some cpu/board definitions for common Kinetis chips, though it does not yet emulate any peripherals - it merely sets up a cortex-m0+ cpu with a specific amount of ram/flash.

This fork aims to add some *basic* support for a handful of peripherals in order to achieve automated unit testing of a project built using the FRDM-KL03Z board. The specific peripherals I aim to add, ordered by priority, are the LPUART, LPTMR, GPIO, and (possibly) TPM modules.

Currently supported features (for FRDM-KL03Z - no other MCUs have been modified; see kinetis_mcus.c to add support for your own. 20~50 LOC per MCU):
* SRAM_L (and SRAM_U)
* Bit Manipulation Engine
* LPUART (partial)
* LPTMR (fully functional except for Pulse Counter mode)

Build instructions
========

One thing the gnuarmeclipse repository lacks is a simple build procedure. You're expected to run some thousand-line build script that fetches the repository from the web, sets up some Docker containers, builds and packages everything inside of there.

That's a bit excessive and unwieldy for simple development builds. 

The easy way to build this is as follows (run in a shell):

```sh
export CFLAGS="-Wno-error"
./configure --target-list="gnuarmeclipse-softmmu" --prefix=/opt/gnuarmeclipse-mod/qemu
make all
sudo make install
```
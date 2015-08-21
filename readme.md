Read the documentation in qemu-doc.html or on http://wiki.qemu-project.org

- QEMU team

========

This is a fork of [gnuarmeclipse's](http://sourceforge.net/projects/gnuarmeclipse/files/QEMU/) QEMU repository. Their repository adds some cpu/board definitions for common Kinetis chips, though it does not yet emulate any peripherals as of yet - it only sets up a cortex-m0+ cpu with a specific amount of ram/flash.

This fork aims to add some *basic* support for a handful of peripherals in order to achieve automated unit testing of a project built using the FRDM-KL03Z board. 

Currently supported features (for FRDM-KL03Z - no other MCUs have been modified; see kinetis_mcus.c to add support for your own. 20~50 LOC per MCU):
* SRAM_L (and SRAM_U)
* Bit Manipulation Engine
* Low-Power UART (LPUART) (partial)
* Low-Power Timer (LPTMR) (fully functional except for Pulse Counter mode)
* Multi-purpose Clock Generator Lite (MCGLite) (though clock settings are not propagated to the core / peripherals)
* System Integration Module (SIM) (less the watchdog timer)
* System Mode Controller (SMC) (Though stop modes cannot be entered)
* GPIO/PORT (including NMI pin)

Build instructions
========

The easy way to build this is as follows (run in a shell):

```sh
export CFLAGS="-Wno-error"
./configure --target-list="gnuarmeclipse-softmmu" --prefix=/opt/gnuarmeclipse-mod/qemu
make all
sudo make install
```

Note: if you run into build errors during `make all` inside config-host.h and are on a system that uses unix line-endings, try running `dos2unix config-host.mak` and then running `make` again.

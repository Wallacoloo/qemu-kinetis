/*
 * Freescale Kinetis Cortex-M devices emulation
 *
 * Copyright (c) 2014 Liviu Ionescu
 *
 * This code is licensed under the GPL.
 */

#include "hw/sysbus.h"

#include "hw/arm/kinetis.h"
#include "hw/arm/cortexm.h"

/*
 Package identifier:
 FM = 32 QFN (5 mm x 5 mm)
 FT = 48 QFN (7 mm x 7 mm)
 LF = 48 LQFP (7 mm x 7 mm)
 LH = 64 LQFP (10 mm x 10 mm)
 MP = 64 MAPBGA (5 mm x 5 mm)
 LK = 80 LQFP (12 mm x 12 mm)
 MB = 81 MAPBGA (8 mm x 8 mm)
 LL = 100 LQFP (14 mm x 14 mm)
 ML = 104 MAPBGA (8 mm x 8 mm)
 MC = 121 MAPBGA (8 mm x 8 mm)
 LQ = 144 LQFP (20 mm x 20 mm)
 MD = 144 MAPBGA (13 mm x 13 mm)
 MJ = 256 MAPBGA (17 mm x 17 mm)
 */

/* ----- MK20DX128VLH5 ----- */
static cortex_m_core_info mk20dx128vlh5_core_info = {
    .device_name = "MK20DX128VLH5",
    .flash_size_kb = 128, /* +32K FlexNVM at 0x10000000 */
    .sram_size_kb = 8, /* +8K SRAM_L */
    .has_mpu = false,
    .has_fpu = false,
};

qemu_irq *mk20dx128vlh5_mcu_init(MachineState *machine)
{
    return cortex_m4_core_init(&mk20dx128vlh5_core_info, machine);
}

/* ----- MK22FN512VLH12 ----- */
static cortex_m_core_info mk22fn512vlh12_core_info = {
    .device_name = "MK22FN512VLH12",
    .flash_size_kb = 512,
    .sram_size_kb = 64, /* +64K SRAM_L */
    .has_mpu = false,
    .has_fpu = false,
};

qemu_irq *mk22fn512vlh12_mcu_init(MachineState *machine)
{
    return cortex_m4_core_init(&mk22fn512vlh12_core_info, machine);
}

/* ----- MK64FN1M0VLL12 ----- */
static cortex_m_core_info mk64fn1m0vll12_core_info = {
    .device_name = "MK64FN1M0VLL12",
    .flash_size_kb = 1024,
    .sram_size_kb = 192, /* +64K SRAM_L */
    .has_mpu = true,
    .has_fpu = true,
};

qemu_irq *mk64fn1m0vll12_mcu_init(MachineState *machine)
{
    return cortex_m4_core_init(&mk64fn1m0vll12_core_info, machine);
}

/* ----- MK60FN1M0VLQ12 ----- */
static cortex_m_core_info mk60fn1m0vlq12_core_info = {
    .device_name = "MK60FN1M0VLQ12",
    .flash_size_kb = 1024,
    .sram_size_kb = 192, /* +64K SRAM_L */
    .has_mpu = true,
    .has_fpu = true,
};

qemu_irq *mk60fn1m0vlq12_mcu_init(MachineState *machine)
{
    return cortex_m4_core_init(&mk60fn1m0vlq12_core_info, machine);
}

/* ----- MKL03Z32VFK4 ----- */
static cortex_m_core_info mkl03z32vfk4_core_info = {
    .device_name = "MKL03Z32VFK4",
    .flash_size_kb = 32,
    .sram_size_kb = 2, /* including the 0.5K SRAM_L */
    .sram_begin = 0x1FFFFE00,
    .has_mpu = false,
};

qemu_irq *mkl03z32vfk4_mcu_init(MachineState *machine)
{
    // 64 entry Nested Vector Interrupt Controller table
    qemu_irq *pic;

    pic = cortex_m0p_core_init(&mkl03z32vfk4_core_info, machine);
    // add the Bit Manipulation Engine components
    // allocate memory region for BME: AND operation (SRAM_U)
    sysbus_create_simple("klbme", 0x24000000, NULL);
    // allocate memory region for BME: OR operation (SRAM_U)
    sysbus_create_simple("klbme", 0x28000000, NULL);
    // allocate memory region for BME: XOR operation (SRAM_U)
    sysbus_create_simple("klbme", 0x2c000000, NULL);
    // allocate memory region for BME: AND operation (Peripherals)
    sysbus_create_simple("klbme", 0x44000000, NULL);
    // allocate memory region for BME: OR operation (Peripherals)
    sysbus_create_simple("klbme", 0x48000000, NULL);
    // allocate memory region for BME: XOR operation (Peripherals)
    sysbus_create_simple("klbme", 0x4c000000, NULL);
    //
    // allocate memory map for BME: Load-and-clear 1 bit (SRAM_U)
    sysbus_create_simple("klbme", 0x30000000, NULL);
    sysbus_create_simple("klbme", 0x34000000, NULL);
    sysbus_create_simple("klbme", 0x38000000, NULL);
    sysbus_create_simple("klbme", 0x3C000000, NULL);
    // allocate memory map for BME: Load-and-clear 1 bit (Peripherals)
    sysbus_create_simple("klbme", 0x50000000, NULL);
    sysbus_create_simple("klbme", 0x54000000, NULL);
    sysbus_create_simple("klbme", 0x58000000, NULL);
    sysbus_create_simple("klbme", 0x5C000000, NULL);

    // add TPMs (Timer/PWM Modules)
    sysbus_create_simple("kltpm", 0x40038000, NULL);
    sysbus_create_simple("kltpm", 0x40039000, NULL);

    // add LPTMR0 (Low Power Timer) to physical ram & NVIC IRQ
    // IRQ is 44 in absolute table; 28 in external IRQ
    sysbus_create_simple("kllptmr", 0x40040000, pic[28]);

    // add SIM (System Integration Module) to physical ram
    sysbus_create_simple("klsim", 0x40047000, NULL);

    // add PORT control for gpios
    // IRSs are 46 and 46 in absolute table; 30 and 31 external IRQ
    sysbus_create_simple("klport", 0x40049000, pic[30]);
    sysbus_create_simple("klport", 0x4004A000, pic[31]);

    // add LPUART0 (Low Power UART) to the appropriate place in physical ram & to the correct NVIC IRQ
    // IRQ is numberd 28 in absolute table, but subtract 16 to get the external IRQ location
    sysbus_create_simple("kllpuart", 0x40054000, pic[12]);

    // add the Multipurpose Clock Generator Lite (MCG_Lite)
    sysbus_create_simple("klmcglite", 0x40064000, NULL);

    // add the Low Level Wakeup Unit (LLWU)
    sysbus_create_simple("klllwu", 0x4007C000, pic[7]);

    // add SMC (System Mode Controller)
    sysbus_create_simple("klsmc", 0x4007E000, NULL);

    // add GPIO control
    SysBusDevice *gpio = OBJECT_CHECK(SysBusDevice,
        sysbus_create_simple("klgpio", 0x400FF000, NULL), "klgpio");
    // Map "Fast" GPIO alias
    sysbus_mmio_map(gpio, 1, 0xF8000000);
    return pic;
}

/* ----- MKL25Z128VLK4 ----- */
static cortex_m_core_info mkl25z128vlk4_core_info = {
    .device_name = "MKL25Z128VLK4",
    .flash_size_kb = 128,
    .sram_size_kb = 12, /* +4K SRAM_L */
    .has_mpu = false,
};

qemu_irq *mkl25z128vlk4_mcu_init(MachineState *machine)
{
    return cortex_m0p_core_init(&mkl25z128vlk4_core_info, machine);
}

/* ----- MKL26Z128VLH4 ----- */
static cortex_m_core_info mkl26z128vlh4_core_info = {
    .device_name = "MKL26Z128VLH4",
    .flash_size_kb = 128,
    .sram_size_kb = 12, /* +4K SRAM_L */
    .has_mpu = false,
};

qemu_irq *mkl26z128vlh4_mcu_init(MachineState *machine)
{
    return cortex_m0p_core_init(&mkl26z128vlh4_core_info, machine);
}

/* ----- MKL43Z256VLH4 ----- */
static cortex_m_core_info mkl43z256vlh4_core_info = {
    .device_name = "MKL43Z256VLH4",
    .flash_size_kb = 256,
    .sram_size_kb = 24, /* +8K SRAM_L */
    .has_mpu = false,
};

qemu_irq *mkl43z256vlh4_mcu_init(MachineState *machine)
{
    return cortex_m0p_core_init(&mkl43z256vlh4_core_info, machine);
}

/* ----- MKL46Z256VLL4 ----- */
static cortex_m_core_info mkl46z256vll4_core_info = {
    .device_name = "MKL46Z256VLL4",
    .flash_size_kb = 256,
    .sram_size_kb = 24, /* +8K SRAM_L */
    .has_mpu = false,
};

qemu_irq *mkl46z256vll4_mcu_init(MachineState *machine)
{
    return cortex_m0p_core_init(&mkl46z256vll4_core_info, machine);
}


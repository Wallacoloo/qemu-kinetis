/*
 * Freescale Kinetis boards emulation
 *
 * Copyright (c) 2014 Liviu Ionescu
 *
 * This code is licensed under the GPL.
 */

#include "hw/arm/kinetis.h"
#include "qemu/module.h"
#include "sysemu/sysemu.h"

/*
 * This file defines several Kinetis boards.
 * Where available, the board names follow the CMSIS Packs names.
 */

/* Common inits for all Kinetis boards */
void kinetis_board_init(MachineState *machine, QEMUMachine *qm)
{
#if defined(CONFIG_VERBOSE)
    if (verbosity_level > 0) {
        printf("Board: '%s' (%s).\n", qm->name, qm->desc);
    }
#endif
}

/* ----- FRDM-K20D50M ----- */
static void frdm_k20d50m_board_init(MachineState *machine);

static QEMUMachine frdm_k20d50m_machine = {
    .name = "FRDM-K20D50M",
    .desc = "Freescale Freedom Development Platform for "
            "Kinetis K20 USB MCUs (Experimental)",
    .init = frdm_k20d50m_board_init,
};

static void frdm_k20d50m_board_init(MachineState *machine)
{
    kinetis_board_init(machine, &frdm_k20d50m_machine);
    mk20dx128vlh5_mcu_init(machine);
    /* TODO: Add board inits */
}

/* ----- FRDM-K22F ----- */
static void frdm_k22f_board_init(MachineState *machine);

static QEMUMachine frdm_k22f_machine = {
    .name = "FRDM-K22F",
    .desc = "Freescale Freedom Development Platform for "
    "Kinetis K22 MCUs (Experimental)",
    .init = frdm_k22f_board_init,
};

static void frdm_k22f_board_init(MachineState *machine)
{
    kinetis_board_init(machine, &frdm_k22f_machine);
    mk22fn512vlh12_mcu_init(machine);
    /* TODO: Add board inits */
}

/* ----- FRDM-K64F ----- */
static void frdm_k64f_board_init(MachineState *machine);

static QEMUMachine frdm_k64f_machine = {
    .name = "FRDM-K64F",
    .desc = "Freescale Freedom Development Platform for "
            "Kinetis K6[34] and K24 MCUs (Experimental)",
    .init = frdm_k64f_board_init,
};

static void frdm_k64f_board_init(MachineState *machine)
{
    kinetis_board_init(machine, &frdm_k64f_machine);
    mk64fn1m0vll12_mcu_init(machine);
    /* TODO: Add board inits */
}

/* ----- TWR-K60F120M ----- */
static void twr_k60f120m_board_init(MachineState *machine);

static QEMUMachine twr_k60f120m_machine = {
    .name = "TWR-K60F120M",
    .desc = "Freescale Kinetis K60 120 MHz Tower System Module (Experimental)",
    .init = twr_k60f120m_board_init,
};

static void twr_k60f120m_board_init(MachineState *machine)
{
    kinetis_board_init(machine, &twr_k60f120m_machine);
    mk64fn1m0vll12_mcu_init(machine);
    /* TODO: Add board inits */
}

/* ----- FRDM-KL03Z ----- */
static void frdm_kl03z_board_init(MachineState *machine);

static QEMUMachine frdm_kl03z_machine = {
    .name = "FRDM-KL03Z",
    .desc = "Freescale Freedom Development Platform for "
            "Kinetis KL03Z MCUs (Experimental)",
    .init = frdm_kl03z_board_init,
};

static void frdm_kl03z_board_init(MachineState *machine)
{
    kinetis_board_init(machine, &frdm_kl03z_machine);
    mkl03z32vfk4_mcu_init(machine);
    /* TODO: Add board inits */
}

/* ----- FRDM-KL25Z ----- */
static void frdm_kl25z_board_init(MachineState *machine);

static QEMUMachine frdm_kl25z_machine = {
    .name = "FRDM-KL25Z",
    .desc = "Freescale Freedom Development Platform for "
            "Kinetis KL[12][45] MCUs (Experimental)",
    .init = frdm_kl25z_board_init,
};

static void frdm_kl25z_board_init(MachineState *machine)
{
    kinetis_board_init(machine, &frdm_kl25z_machine);
    mkl25z128vlk4_mcu_init(machine);
    /* TODO: Add board inits */
}

/* ----- FRDM-KL26Z ----- */
static void frdm_kl26z_board_init(MachineState *machine);

static QEMUMachine frdm_kl26z_machine = {
    .name = "FRDM-KL26Z",
    .desc = "Freescale Freedom Development Platform for "
            "Kinetis KL[12]6 MCUs (Experimental)",
    .init = frdm_kl26z_board_init,
};

static void frdm_kl26z_board_init(MachineState *machine)
{
    kinetis_board_init(machine, &frdm_kl26z_machine);
    mkl26z128vlh4_mcu_init(machine);
    /* TODO: Add board inits */
}

/* ----- FRDM-KL43Z ----- */
static void frdm_kl43z_board_init(MachineState *machine);

static QEMUMachine frdm_kl43z_machine = {
    .name = "FRDM-KL43Z",
    .desc = "Freescale Freedom Development Platform for "
    "Kinetis KL[34]3, KL[12]7 MCUs (Experimental)",
    .init = frdm_kl43z_board_init,
};

static void frdm_kl43z_board_init(MachineState *machine)
{
    kinetis_board_init(machine, &frdm_kl43z_machine);
    mkl43z256vlh4_mcu_init(machine);
    /* TODO: Add board inits */
}

/* ----- FRDM-KL46Z ----- */
static void frdm_kl46z_board_init(MachineState *machine);

static QEMUMachine frdm_kl46z_machine = {
    .name = "FRDM-KL46Z",
    .desc = "Freescale Freedom Development Platform for "
            "Kinetis KL[34]x MCUs (Experimental)",
    .init = frdm_kl46z_board_init,
};

static void frdm_kl46z_board_init(MachineState *machine)
{
    kinetis_board_init(machine, &frdm_kl46z_machine);
    mkl46z256vll4_mcu_init(machine);
    /* TODO: Add board inits */
}

/* ----- Boards inits ----- */
static void kinetis_machine_init(void)
{
    qemu_register_machine(&frdm_k20d50m_machine);
    qemu_register_machine(&frdm_k64f_machine);
    qemu_register_machine(&frdm_k22f_machine);
    qemu_register_machine(&twr_k60f120m_machine);
    qemu_register_machine(&frdm_kl03z_machine);
    qemu_register_machine(&frdm_kl25z_machine);
    qemu_register_machine(&frdm_kl26z_machine);
    qemu_register_machine(&frdm_kl46z_machine);
    qemu_register_machine(&frdm_kl43z_machine);
}

machine_init(kinetis_machine_init);


/*
 * System Mode Controller (SMC) found in the kinetis KL series (see KL03P24M48SF0RM.pdf sect 15).
 * 
 * The SMC is responsible for sequencing the system into and out of all low-power Stop and Run modes..
 *
 * Features lacking in this implementation:
 *   PMSTAT returns RUN (0x01) in either RUN mode (correct) or STOP (incorrect)
 *   PMSTAT returns VLPR (0x04) in either VLPR (correct) or VLPW, VLPS, VLLS (incorrect)
 *   PMCTRL:STOPA always reads 0
 *   Functionally, all power modes behave as RUN mode - these registers can be read and written,
 *     but don't actually do anything externally.
 *
 * Authored by Colin Wallace <wallace.colin.a@gmail.com>
 *
 * This code is regrettably licensed under the GPL.
 */

#include "hw/sysbus.h"

#define TYPE_KLSMC "klsmc"
#define KLSMC(obj) OBJECT_CHECK(KLSMCState, (obj), TYPE_KLSMC)


typedef struct KLSMCState {
    SysBusDevice parent_obj;

    // device registers (publicly readable/writeable)
    uint8_t PMPROT; // Power Mode Protection Register
    uint8_t PMCTRL; // Power Mode Control Register
    uint8_t STOPCTRL; // Stop Control Register
    // uint8_t PMSTAT; // Power Mode Status Register (Read-only)

    // internal state
    bool has_written_pmprot; // PMPROT is write-once

    MemoryRegion iomem;
} KLSMCState;


// write `value` to `dest`, but do not affect any bit i for which mask[i] = 0.
// returns a mask indicating which bits were changed
static uint8_t write_masked8(uint8_t *dest, uint8_t mask, uint8_t value)
{
    uint8_t old = *dest;

    *dest = (*dest & ~mask) | (value & mask);

    return old ^ *dest;
}

static uint64_t klsmc_read(void *opaque, hwaddr offset,
                           unsigned size)
{
    KLSMCState *s = (KLSMCState *)opaque;

    switch (offset)
    {
        case 0x0000:
        	return s->PMPROT;
        case 0x0001:
        	return s->PMCTRL;
        case 0x0002:
        	return s->STOPCTRL;
        case 0x0003:
        	// see note at top of file; this register not fully implemented
        	if ((s->PMCTRL & 0x60) == 0x40)
        	{
        		return 0x04; // VLPR
        	}
        	else
        	{
        		return 0x01; // RUN
        	}
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                      "klsmc_read: Bad offset %x\n", (int)offset);
    }
    
    return 0xDEADBEEF;
}

static void klsmc_write(void *opaque, hwaddr offset,
                        uint64_t value, unsigned size)
{
    // this function is called whenever the user program writes to a decorated address 
    KLSMCState *s = (KLSMCState *)opaque;
    uint8_t value8 = value;
    
    switch (offset)
    {
        case 0x0000:
        	if (!s->has_written_pmprot)
        	{
        		// PMPROT is write-once
        		write_masked8(&s->PMPROT, 0x22, value8);
        		s->has_written_pmprot = true;
        	}
        	break;
        case 0x0001:
        	if ((value8 & 0x60) == 0 || 
        		((value8 & 0x60) == 0x40 && (s->PMPROT & 0x20)))
        	{
        		// allow to set RUNM = 00 (Normal Run mode) always,
        		//   or RUNM = 10 (Very-low-power Run mode) if PMPROT:AVLP = 1
        		write_masked8(&s->PMCTRL, 0x60, value8);
        	}
        	if ( (value8 & 0x07) == 0x0 || 
        		((value8 & 0x07) == 0x2 && (s->PMPROT & 0x20)) ||
        		((value8 & 0x07) == 0x4 && (s->PMPROT & 0x02)) )
        	{
        		// allow to set STOPM=000 (Normal stop) always,
        		//   or STOPM=010 (VLPS) if PMPROT:AVLP = 1,
        		//   or STOPM=100 (VLLSx) if PMPROT:AVLLS = 1
        		write_masked8(&s->PMCTRL, 0x07, value8);
        	}
        	break;
        case 0x0002:
        	write_masked8(&s->STOPCTRL, 0xEF, value8);
        	break;
        case 0x0003:
        	// PMSTAT is read-only
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                      "klsmc_write: Bad offset %x\n", (int)offset);
    }
}


static const MemoryRegionOps klsmc_ops = {
    .read = klsmc_read,
    .write = klsmc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_klsmc = {
    .name = TYPE_KLSMC,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (VMStateField[]) {
    	VMSTATE_UINT8(PMPROT, KLSMCState),
    	VMSTATE_UINT8(PMCTRL, KLSMCState),
    	VMSTATE_UINT8(STOPCTRL, KLSMCState),
    	VMSTATE_BOOL(has_written_pmprot, KLSMCState),
        VMSTATE_END_OF_LIST()
    }
};

static void klsmc_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    KLSMCState *s = KLSMC(obj);

    memory_region_init_io(&s->iomem, OBJECT(s), &klsmc_ops, s, TYPE_KLSMC, 0x00001000);
    sysbus_init_mmio(sbd, &s->iomem);

    // register reset values
    s->PMPROT   = 0x00;
    s->PMCTRL   = 0x00;
    s->STOPCTRL = 0x03;

    s->has_written_pmprot = false;
}

static void klsmc_realize(DeviceState *dev, Error **errp)
{
}

static void klsmc_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = klsmc_realize;
    dc->vmsd = &vmstate_klsmc;
}

static const TypeInfo klsmc_arm_info = {
    .name          = TYPE_KLSMC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(KLSMCState),
    .instance_init = klsmc_init,
    .class_init    = klsmc_class_init,
};

static void klsmc_register_types(void)
{
    type_register_static(&klsmc_arm_info);
}

type_init(klsmc_register_types)
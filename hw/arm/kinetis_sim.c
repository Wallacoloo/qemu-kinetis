/*
 * System Integration Module (SIM) found in the kinetis KL series (see KL03P24M48SF0RM.pdf sect 14).
 * 
 * The SIM is responsible for configuring peripheral clocking, exposing device info, etc.
 *
 * Features lacking in this implementation:
 *   COP Watchdog will never trigger a reset
 *   SDID register exposes only generic family/ram size/pincount info
 *     (should be device-specific)
 *   CLKDIV1 register does not compute OUTDIV1 from FTFA_FOPT[LPBOOT], as it should.
 *   FCFG1[PFSIZE] is set to indicate 32 KB of program flash mem, no matter the device's actual flash size
 *   FCFG2[MAXADDR0] is set based on 32 KB of program flash mem, not device's actual flash size
 *   UIDxx registers are set to generic values
 *
 * Authored by Colin Wallace <wallace.colin.a@gmail.com>
 *
 * This code is regrettably licensed under the GPL.
 */


#include "hw/sysbus.h"

#define TYPE_KLSIM "klsim"
#define KLSIM(obj) OBJECT_CHECK(KLSIMState, (obj), TYPE_KLSIM)


typedef struct KLSIMState {
    SysBusDevice parent_obj;

    // device registers (publicly readable/writeable)
    uint32_t SOPT1; // Systems Options Register 1
    uint32_t SOPT2; // Systems Options Register 2
    // uint32_t unused;
    uint32_t SOPT4; // Systems Options Register 4
    uint32_t SOPT5; // Systems Options Register 5
    // uint32_t unused;
    uint32_t SOPT7; // Systems Options Register 7
    // uint32_t unused[3];
    uint32_t SDID; // System Device Id Register (Read-only)
    // uint32_t unused[3];
    uint32_t SCGC4; // System Clock Gating Control Register 4
    uint32_t SCGC5; // System Clock Gating Control Register 5
    uint32_t SCGC6; // System Clock Gating Control Register 6
    // uint32_t unused;
    uint32_t CLKDIV1; // System Clock Divider Register 1
    // uint32_t unused;
    uint32_t FCFG1; // Flash Config Register 1
    uint32_t FCFG2; // Flash Config Register 2 (Read-only)
    // uint32_t unused;
    uint32_t UIDMH; // Unique Id Register Mid-High (Read-only)
    uint32_t UIDML; // Unique Id Register Mid-Low (Read-only)
    uint32_t UIDL; // Unique Id Register Low (Read-only)
    // uint32_t unused[40]
    uint32_t COPC; // COP Control Register
    // uint32_t SRVCOP; // Service COP (Write-only)

    MemoryRegion iomem;
} KLSIMState;


// write `value` to `dest`, but do not affect any bit i for which mask[i] = 0.
// returns a mask indicating which bits were changed
static uint32_t write_masked(uint32_t *dest, uint32_t mask, uint32_t value)
{
    uint32_t old = *dest;

    *dest = (*dest & ~mask) | (value & mask);

    return old ^ *dest;
}

static uint64_t klsim_read(void *opaque, hwaddr offset,
                           unsigned size)
{
    KLSIMState *s = (KLSIMState *)opaque;

    switch (offset)
    {
        case 0x0000: // Systems Options Register 1
            return s->SOPT1;
        case 0x1004: // Systems Options Register 2
            return s->SOPT2;
        case 0x100C: // Systems Options Register 4
            return s->SOPT4;
        case 0x1010: // Systems Options Register 5
            return s->SOPT5;
        case 0x1018: // Systems Options Register 7
            return s->SOPT7;
        case 0x1024: // System Device Id Register
            return s->SDID;
        case 0x1034: // System Clock Gating Control Register 4
            return s->SCGC4;
        case 0x1038: // System Clock Gating Control Register 5
            return s->SCGC5;
        case 0x103C: // System Clock Gating Control Register 6
            return s->SCGC6;
        case 0x1044: // System Clock Divider Register 1
            return s->CLKDIV1;
        case 0x104C: // Flash Config Register 1
            return s->FCFG1;
        case 0x1050: // Flash Config Register 2
            return s->FCFG2;
        case 0x1058: // Unique Id Register Mid-High
            return s->UIDMH;
        case 0x105C: // Unique Id Register Mid-Low
            return s->UIDML;
        case 0x1060: // Unique Id Register Low
            return s->UIDL;
        case 0x1100: // COP Control Register
            return s->COPC;
        //case 0x1104: // Service COP (write-only - reads cause bus errors)
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                      "klsim_read: Bad offset %x\n", (int)offset);
    }
    
    return 0xDEADBEEF;
}

static void klsim_write(void *opaque, hwaddr offset,
                        uint64_t value, unsigned size)
{
    // this function is called whenever the user program writes to a decorated address 
    KLSIMState *s = (KLSIMState *)opaque;
    uint32_t value32 = value;
    
    switch (offset)
    {
        case 0x0000: // Systems Options Register 1
            write_masked(&s->SOPT1, 0x000F0000, value32);
        	break;
        case 0x1004: // Systems Options Register 2
            write_masked(&s->SOPT2, 0x0F0000F0, value32);
        	break;
        case 0x100C: // Systems Options Register 4
            write_masked(&s->SOPT4, 0x03040000, value32);
        	break;
        case 0x1010: // Systems Options Register 5
            write_masked(&s->SOPT5, 0x00010005, value32);
        	break;
        case 0x1018: // Systems Options Register 7
            write_masked(&s->SOPT7, 0x0000009F, value32);
        	break;
        //case 0x1024: // System Device Id Register (Read-only)
        case 0x1034: // System Clock Gating Control Register 4
            write_masked(&s->SCGC4, 0x00580040, value32);
        	break;
        case 0x1038: // System Clock Gating Control Register 5
            write_masked(&s->SCGC5, 0x00100601, value32);
        	break;
        case 0x103C: // System Clock Gating Control Register 6
            write_masked(&s->SCGC6, 0x2B000001, value32);
        	break;
        case 0x1044: // System Clock Divider Register 1
            write_masked(&s->CLKDIV1, 0xF00F0000, value32);
        	break;
        case 0x104C: // Flash Config Register 1
            write_masked(&s->FCFG1, 0x00000003, value32);
        	break;
        //case 0x1050: // Flash Config Register 2 (Read-only)
        //case 0x1058: // Unique Id Register Mid-High (Read-only)
        //case 0x105C: // Unique Id Register Mid-Low (Read-only)
        //case 0x1060: // Unique Id Register Low (Read-only)
        case 0x1100: // COP Control Register
            write_masked(&s->COPC, 0x000000FF, value32);
        	break;
        case 0x1104: // Service COP
        	break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                      "klsim_write: Bad offset %x\n", (int)offset);
    }
}


static const MemoryRegionOps klsim_ops = {
    .read = klsim_read,
    .write = klsim_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_klsim = {
    .name = TYPE_KLSIM,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

static void klsim_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    KLSIMState *s = KLSIM(obj);

    memory_region_init_io(&s->iomem, OBJECT(s), &klsim_ops, s, TYPE_KLSIM, 0x00002000);
    sysbus_init_mmio(sbd, &s->iomem);

    // register reset values
    s->SOPT1   = 0x00000000;
    s->SOPT2   = 0x00000000;
    s->SOPT4   = 0x00000000;
    s->SOPT5   = 0x00000000;
    s->SOPT7   = 0x00000000;
    // note: SDID depends upon the specific device (ram size, pin count, etc)
    // see sect 14.5.6 of KL03P24M48SF0RM.pdf for detailed info
    s->SDID    = 0x03120681;
    s->SCGC4   = 0xF0000030;
    s->SCGC5   = 0x00000182;
    s->SCGC6   = 0x00000001;
    // TODO: the initialization value of this field depends on FTFA_FOPT[LPBOOT]
    // see sect 14.5.10 of KL03P24M48SF0RM.pdf for detailed info
    s->CLKDIV1 = 0x00010000;
    // note: FCFG1 depends upon the specific device (Specifically, flash size)
    // see sect 14.5.11 of KL03P24M48SF0RM.pdf for detailed info
    s->FCFG1   = 0x03000000;
    // note: FCFG2 depends upon the specific device (Specifically, flash size)
    // see sect 14.5.12 of KL03P24M48SF0RM.pdf for detailed info
    s->FCFG2   = 0x04800000;
    // note: the lower 16 bits of UIDMH are a device-specific unique id
    s->UIDMH   = 0x00001234;
    // note: UIDML is a device-specific unique id
    s->UIDML   = 0x56789ABC;
    // note: UIDL is a device-specific unique id
    s->UIDL    = 0xDEF01234;
    s->COPC    = 0x0000000C;
}

static void klsim_realize(DeviceState *dev, Error **errp)
{
}

static void klsim_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = klsim_realize;
    dc->vmsd = &vmstate_klsim;
}

static const TypeInfo klsim_arm_info = {
    .name          = TYPE_KLSIM,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(KLSIMState),
    .instance_init = klsim_init,
    .class_init    = klsim_class_init,
};

static void klsim_register_types(void)
{
    type_register_static(&klsim_arm_info);
}

type_init(klsim_register_types)
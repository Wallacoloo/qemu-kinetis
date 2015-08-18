/*
 * Multipurpose Clock Generator Lite (MCG_Lite) found in the kinetis KL series (see KL03P24M48SF0RM.pdf sect 24).
 * 
 * The MCG Lite is responsible for selecting from internal / external clocks for the core and bus
 *   & appropriately dividing them
 *
 * Features lacking in this implementation:

 *
 * Authored by Colin Wallace <wallace.colin.a@gmail.com>
 *
 * This code is regrettably licensed under the GPL.
 */

#include "hw/sysbus.h"

#define TYPE_KLMCGLITE "klmcglite"
#define KLMCGLITE(obj) OBJECT_CHECK(KLMCGLITEState, (obj), TYPE_KLMCGLITE)


typedef struct KLMCGLITEState {
    SysBusDevice parent_obj;

    // device registers (publicly readable/writeable)
    uint8_t C1; // Control Register 1
    uint8_t C2; // Control Register 2
    uint8_t SC; // Status & Control Register
    uint8_t MC; // Miscellaneous Control Register

    MemoryRegion iomem;
} KLMCGLITEState;


// write `value` to `dest`, but do not affect any bit i for which mask[i] = 0.
// returns a mask indicating which bits were changed
static uint8_t write_masked8(uint8_t *dest, uint8_t mask, uint8_t value)
{
    uint8_t old = *dest;

    *dest = (*dest & ~mask) | (value & mask);

    return old ^ *dest;
}

static uint64_t klmcglite_read(void *opaque, hwaddr offset,
                           unsigned size)
{
    KLMCGLITEState *s = (KLMCGLITEState *)opaque;

    switch (offset)
    {
        case 0x00: // Control Register 1
            return s->C1;
        case 0x01: // Control Register 2:
            return s->C2;
        case 0x06: // Status Register
        {
            // assume that mode switches are instantaneous therefore S.CLKST = C1.CLKS
            uint8_t clkst = (s->C1 >> 6) & 3;
            // assume that the oscillator is never disabled
            bool oscinit0 = 1;
            return (clkst << 2) | (oscinit0 << 1);
        }
        case 0x08: // Status and Control Register
            return s->SC;
        case 0x18: // Misc Control Register
            return s->MC;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                      "klmcglite_read: Bad offset %x\n", (int)offset);
    }
    
    return 0xDEADBEEF;
}

static void klmgclite_write(void *opaque, hwaddr offset,
                        uint64_t value, unsigned size)
{
    // this function is called whenever the user program writes to a decorated address 
    KLMCGLITEState *s = (KLMCGLITEState *)opaque;
    uint8_t value8 = value;
    
    switch (offset)
    {
        case 0x00: // Control Register 1
            write_masked8(&s->C1, 0xC3, value8);
            break;
        case 0x01: // Control Register 2:
            write_masked8(&s->C2, 0x05, value8);
            break;
        // case 0x06: // Status Register is read-only
        case 0x08: // Status and Control Register
            write_masked8(&s->SC, 0x0E, value8);
            break;
        case 0x18: // Misc Control Register
            write_masked8(&s->MC, 0x87, value8);
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                      "klmgclite_write: Bad offset %x\n", (int)offset);
    }
}


static const MemoryRegionOps klmcglite_ops = {
    .read = klmcglite_read,
    .write = klmgclite_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_klmcglite = {
    .name = TYPE_KLMCGLITE,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (VMStateField[]) {
    	VMSTATE_UINT8(C1, KLMCGLITEState),
    	VMSTATE_UINT8(C2, KLMCGLITEState),
    	VMSTATE_UINT8(SC, KLMCGLITEState),
        VMSTATE_UINT8(MC, KLMCGLITEState),
        VMSTATE_END_OF_LIST()
    }
};

static void klmcglite_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    KLMCGLITEState *s = KLMCGLITE(obj);

    memory_region_init_io(&s->iomem, OBJECT(s), &klmcglite_ops, s, TYPE_KLMCGLITE, 0x00001000);
    sysbus_init_mmio(sbd, &s->iomem);

    // register reset values
    s->C1 = 0x01;
    s->C2 = 0x04;
    s->SC = 0x00;
    s->MC = 0x00;
}

static void klmcglite_realize(DeviceState *dev, Error **errp)
{
}

static void klmcglite_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = klmcglite_realize;
    dc->vmsd = &vmstate_klmcglite;
}

static const TypeInfo klmcglite_arm_info = {
    .name          = TYPE_KLMCGLITE,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(KLMCGLITEState),
    .instance_init = klmcglite_init,
    .class_init    = klmcglite_class_init,
};

static void klmcglite_register_types(void)
{
    type_register_static(&klmcglite_arm_info);
}

type_init(klmcglite_register_types)
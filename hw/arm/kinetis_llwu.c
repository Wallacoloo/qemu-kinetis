/*
 * Low Level Wakeup Unit (LLWU) found in the kinetis KL series 
 * (see KL03P24M48SF0RM.pdf sect 19).
 * 
 * The LLWU module allows the user to select up to 8 external pins and up to 8 internal
 * modules as interrupt wake-up sources from low-leakage power modes. It also supports up
 * to 8 internal modules as temporary DMA wake-up sources.
 *
 * Features lacking in this implementation:
 *  * IRQs
 *  * All registers read the last value written
 *
 * Authored by Colin Wallace <wallace.colin.a@gmail.com>
 *
 * This code is regrettably licensed under the GPL.
 */

#include "hw/sysbus.h"

#define TYPE_KLLLWU "klllwu"
#define KLLLWU(obj) OBJECT_CHECK(KLLLWUState, (obj), TYPE_KLLLWU)


typedef struct KLLLWUState {
    SysBusDevice parent_obj;

    // device registers (publicly readable/writeable)
    uint8_t PE1; // Pin Enable 1 Register
    uint8_t PE2; // Pin Enable 2 Register
    uint8_t ME; // Module Enable Register
    uint8_t F1; // Flag 1 Register
    uint8_t F2; // Flag 2 Register
    uint8_t FILT1; // Filter 1 Register
    uint8_t FILT2; // Filter 2 Register

    MemoryRegion iomem;
    qemu_irq irq;
} KLLLWUState;


// write `value` to `dest`, but do not affect any bit i for which mask[i] = 0.
// returns a mask indicating which bits were changed
static uint8_t write_masked8(uint8_t *dest, uint8_t mask, uint8_t value)
{
    uint8_t old = *dest;

    *dest = (*dest & ~mask) | (value & mask);

    return old ^ *dest;
}

// certain registers contain flags that are cleared by attempting to write a 1 to them.
// These "write 1 to clear" (w1c) bits are indicated by `mask`
// all w1c bits in mask for which write_val = 1 will cause those bits in *reg to be set to 0
static uint8_t write1clear8(uint8_t *reg, uint8_t mask, uint8_t write_val)
{
    return write_masked8(reg, mask & write_val, 0);
}

static uint64_t klllwu_read(void *opaque, hwaddr offset,
                           unsigned size)
{
    KLLLWUState *s = (KLLLWUState *)opaque;

    switch (offset)
    {
        case 0x0000:
            return s->PE1;
        case 0x0001:
            return s->PE2;
        case 0x0002:
            return s->ME;
        case 0x0003:
            return s->F1;
        case 0x0004:
            return s->F2;
        case 0x0005:
            return s->FILT1;
        case 0x0006:
            return s->FILT2;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                      "klllwu_read: Bad offset %x\n", (int)offset);
    }
    
    return 0xDEADBEEF;
}

static void klllwu_write(void *opaque, hwaddr offset,
                        uint64_t value, unsigned size)
{
    // this function is called whenever the user program writes to a decorated address 
    KLLLWUState *s = (KLLLWUState *)opaque;
    uint8_t value8 = value;
    
    switch (offset)
    {
        case 0x0000:
            s->PE1 = value8;
            break;
        case 0x0001:
            s->PE2 = value8;
            break;
        case 0x0002:
            s->ME = value8;
            break;
        case 0x0003:
            write1clear8(&s->F1, 0xFF, value8);
            break;
        //case 0x0004: // F2 is read-only
        case 0x0005:
            write_masked8(&s->FILT1, 0xEF, value8);
            break;
        case 0x0006:
            write_masked8(&s->FILT2, 0xEF, value8);
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                      "klllwu_write: Bad offset %x\n", (int)offset);
    }
}


static const MemoryRegionOps klllwu_ops = {
    .read = klllwu_read,
    .write = klllwu_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_klllwu = {
    .name = TYPE_KLLLWU,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (VMStateField[]) {
    	VMSTATE_UINT8(PE1, KLLLWUState),
        VMSTATE_UINT8(PE2, KLLLWUState),
        VMSTATE_UINT8(ME, KLLLWUState),
        VMSTATE_UINT8(F1, KLLLWUState),
        VMSTATE_UINT8(F2, KLLLWUState),
        VMSTATE_UINT8(FILT1, KLLLWUState),
        VMSTATE_UINT8(FILT2, KLLLWUState),
        VMSTATE_END_OF_LIST()
    }
};

static void klllwu_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    KLLLWUState *s = KLLLWU(obj);

    memory_region_init_io(&s->iomem, OBJECT(s), &klllwu_ops, s, TYPE_KLLLWU, 0x00001000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    // register reset values
    s->PE1   = 0x00;
    s->PE2   = 0x00;
    s->ME    = 0x00;
    s->F1    = 0x00;
    s->F2    = 0x00;
    s->FILT1 = 0x00;
    s->FILT2 = 0x00;
}

static void klllwu_realize(DeviceState *dev, Error **errp)
{
}

static void klllwu_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = klllwu_realize;
    dc->vmsd = &vmstate_klllwu;
}

static const TypeInfo klllwu_arm_info = {
    .name          = TYPE_KLLLWU,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(KLLLWUState),
    .instance_init = klllwu_init,
    .class_init    = klllwu_class_init,
};

static void klllwu_register_types(void)
{
    type_register_static(&klllwu_arm_info);
}

type_init(klllwu_register_types)
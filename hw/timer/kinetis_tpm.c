/*
 *Timer/PWM Module found in the kinetis KL series.
 *
 * Authored by Colin Wallace <wallace.colin.a@gmail.com>
 *
 * This code is regrettably licensed under the GPL.
 */

#include "hw/sysbus.h"

#define TYPE_KLTPM "kltpm"
#define KLTPM(obj) OBJECT_CHECK(KLTPMState, (obj), TYPE_KLTPM)



typedef struct KLLTPMState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    // device registers (publicly readable/writeable)
    uint32_t SC; // Status & Control
    uint32_t CNT; // Counter
    uint32_t MOD; // Modulo
    uint32_t C0SC; // Channel 0 Status & Control
    uint32_t C0V; // Channel 0 Value
    uint32_t C1SC; // Channel 1 Status & Control
    uint32_t C1V; // Channel 1 Value
    uint32_t STATUS; //Capture & Compare Status
    uint32_t CONF; // Configuration
} KLTPMState;




// this function is called whenever the user program reads from a register
//   inside the range of this peripheral.
// this can trigger things like the clearing of IRQs, etc.
static uint64_t kltpm_read(void *opaque, hwaddr offset,
                           unsigned size)
{
    KLTPMState *s = (KLTPMState *)opaque;

    switch (offset >> 2)
    {
    case 0:
        return s->SC; // Status & Control
    case 1:
        return s->CNT; // Counter
    case 2:
        return s->MOD; // Modulo
    case 3:
        return s->C0SC; // Channel 0 Status & Control
    case 4:
        return s->C0V; // Channel 0 Value
    case 5:
        return s->C1SC; // Channel 1 Status & Control
    case 6:
        return s->C1V; // Channel 1 Value
    case 7:
        return s->STATUS; //Capture & Compare Status
    case 8:
        return s->CONF; // Configuration

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "kltpm_read: Bad offset %x\n", (int)offset);
        return 0;
    }
}

static void kltpm_write(void *opaque, hwaddr offset,
                        uint64_t value, unsigned size)
{
    // this function is called whenever the user program writes to a register
    //   inside the range of this peripheral.
    KLTPMState *s = (KLTPMState *)opaque;
    uint32_t value32 = value;

    switch (offset >> 2)
    {
    case 0:
        s->SC = value32; // Status & Control
        break;
    case 1:
        s->CNT = value32; // Counter
        break;
    case 2:
        s->MOD = value32; // Modulo
        break;
    case 3:
        s->C0SC = value32; // Channel 0 Status & Control
        break;
    case 4:
        s->C0V = value32; // Channel 0 Value
        break;
    case 5:
        s->C1SC = value32; // Channel 1 Status & Control
        break;
    case 6:
        s->C1V = value32; // Channel 1 Value
        break;
    case 7:
        s->STATUS = value32; //Capture & Compare Status
        break;
    case 8:
        s->CONF = value32; // Configuration
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "kltpm_write: Bad offset %x\n", (int)offset);
    }
}



static const MemoryRegionOps kltpm_ops = {
    .read = kltpm_read,
    .write = kltpm_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_kltpm = {
    .name = TYPE_KLTPM,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(SC, KLTPMState),
        VMSTATE_UINT32(CNT, KLTPMState),
        VMSTATE_UINT32(MOD, KLTPMState),
        VMSTATE_UINT32(C0SC, KLTPMState),
        VMSTATE_UINT32(C0V, KLTPMState),
        VMSTATE_UINT32(C1SC, KLTPMState),
        VMSTATE_UINT32(C1V, KLTPMState),
        VMSTATE_UINT32(STATUS, KLTPMState),
        VMSTATE_UINT32(CONF, KLTPMState),
        VMSTATE_END_OF_LIST()
    }
};

static void kltpm_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    KLTPMState *s = KLTPM(obj);

    memory_region_init_io(&s->iomem, OBJECT(s), &kltpm_ops, s, TYPE_KLTPM, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);

    // register reset values
    s->SC =     0x00000000;
    s->CNT =    0x00000000;
    s->MOD =    0x0000FFFF;
    s->C0SC =   0x00000000;
    s->C0V =    0x00000000;
    s->C1SC =   0x00000000;
    s->C1V =    0x00000000;
    s->STATUS = 0x00000000;
    s->CONF =   0x00000000;
}

static void kltpm_realize(DeviceState *dev, Error **errp)
{
}

static void kltpm_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = kltpm_realize;
    dc->vmsd = &vmstate_kltpm;
}

static const TypeInfo kltpm_arm_info = {
    .name          = TYPE_KLTPM,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(KLTPMState),
    .instance_init = kltpm_init,
    .class_init    = kltpm_class_init,
};

static void kltpm_register_types(void)
{
    type_register_static(&kltpm_arm_info);
}

type_init(kltpm_register_types)

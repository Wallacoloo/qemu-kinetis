/*
 * Low-Power Timer found in the kinetis KL series.
 *
 * Authored by Colin Wallace <wallace.colin.a@gmail.com>
 *
 * This code is regrettably licensed under the GPL.
 */

#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "sysemu/sysemu.h"

#define TYPE_KLLPTMR "kllptmr"
#define KLLPTMR(obj) OBJECT_CHECK(KLLPTMRState, (obj), TYPE_KLLPTMR)

typedef struct KLLPTMRState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    // device registers (publicly readable/writeable)
    uint32_t CSR; // control status register
    uint32_t PSR; // prescale register
    uint32_t CMR; // compare register
    uint32_t CNR; // counter register

    QEMUTimer *timer;
    qemu_irq irq;
} KLLPTMRState;




static void kllptmr_interrupt(void * opaque)
{
    KLLPTMRState *s = (KLLPTMRState *)opaque;
}

static uint64_t kllptmr_read(void *opaque, hwaddr offset,
                           unsigned size)
{
    KLLPTMRState *s = (KLLPTMRState *)opaque;

    switch (offset >> 2) {
    case 0: // Control Status Register
        return s->CSR;
    case 1: // Prescale Register
        return s->PSR;
    case 2: // Compare Register
        return s->CMR;
    case 3: // Counter Register
        return s->CNR;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "kllptmr_read: Bad offset 0x%x\n", (int)offset);
        break;
    }

    return 0;
}

static void kllptmr_write(void * opaque, hwaddr offset,
                        uint64_t value, unsigned size)
{
    KLLPTMRState *s = (KLLPTMRState *)opaque;
    uint32_t value32 = value;

    switch (offset >> 2) {
    case 0: // Control Status Register
        s->CSR = value32 & 0x7F;
        // TODO: write-1-to-clear TCF
        break;
    case 1: // Prescale Register
        s->PSR = value32 & 0x7F;
        break;
    case 2: // Compare Register
        s->CMR = value32 & 0xFFFF;
        break;
    case 3: // Counter Register (read-only)
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "kllptmr_write: Bad offset 0x%x\n", (int)offset);
        break;
    }
}

static const MemoryRegionOps kllptmr_ops = {
    .read = kllptmr_read,
    .write = kllptmr_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int kllptmr_init(SysBusDevice *dev)
{
    KLLPTMRState *s = KLLPTMR(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &kllptmr_ops, s, "pl031", 0x1000);
    sysbus_init_mmio(dev, &s->iomem);

    sysbus_init_irq(dev, &s->irq);

    s->timer = timer_new_ns(rtc_clock, kllptmr_interrupt, s);
    return 0;
}

static const VMStateDescription vmstate_kllptmr = {
    .name = "kllptmr",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(CSR, KLLPTMRState),
        VMSTATE_UINT32(PSR, KLLPTMRState),
        VMSTATE_UINT32(CMR, KLLPTMRState),
        VMSTATE_UINT32(CNR, KLLPTMRState),
        VMSTATE_END_OF_LIST()
    }
};

static void kllptmr_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = kllptmr_init;
    dc->vmsd = &vmstate_kllptmr;
}

static const TypeInfo kllptmr_info = {
    .name          = TYPE_KLLPTMR,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(KLLPTMRState),
    .class_init    = kllptmr_class_init,
};

static void kllptmr_register_types(void)
{
    type_register_static(&kllptmr_info);
}

type_init(kllptmr_register_types)

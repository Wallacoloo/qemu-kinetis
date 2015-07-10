/*
 * Low-Power UART found in the kinetis KL series
 *
 * This code is regrettably licensed under the GPL.
 */

#include "hw/sysbus.h"
#include "sysemu/char.h"

#define TYPE_KLLPUART "kllpuart"
#define KLLPUART(obj) OBJECT_CHECK(KLLPUARTState, (obj), TYPE_KLLPUART)

typedef struct KLLPUARTState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    // device registers (publicly readable/writeable)
    uint32_t BAUD;
    uint32_t STAT;
    uint32_t CTRL;
    uint32_t DATA;
    uint32_t MATCH;

    /*uint32_t readbuff;
    uint32_t flags;
    uint32_t lcr;
    uint32_t rsr;
    uint32_t cr;
    uint32_t dmacr;
    uint32_t int_enabled;
    uint32_t int_level;
    uint32_t read_fifo[16];
    uint32_t ilpr;
    uint32_t ibrd;
    uint32_t fbrd;
    uint32_t ifl;
    int read_pos;
    int read_count;
    int read_trigger;*/
    CharDriverState *chr;
    qemu_irq irq;
} KLLPUARTState;

/*#define PL011_INT_TX 0x20
#define PL011_INT_RX 0x10

#define PL011_FLAG_TXFE 0x80
#define PL011_FLAG_RXFF 0x40
#define PL011_FLAG_TXFF 0x20
#define PL011_FLAG_RXFE 0x10*/


static void kllpuart_update(KLLPUARTState *s)
{
    uint32_t flags = 0;

    //flags = s->int_level & s->int_enabled;
    qemu_set_irq(s->irq, flags != 0);
}

static uint64_t kllpuart_read(void *opaque, hwaddr offset,
                           unsigned size)
{
    // this function is called whenever the user program reads from a register
    //   inside the range of this peripheral.
    // this can trigger things like the clearing of IRQs, etc.
    KLLPUARTState *s = (KLLPUARTState *)opaque;

    switch (offset >> 2)
    {
    case 0: // BAUD register
        return s->BAUD;
    case 1: // STAT register
        return s->STAT;
    case 2: // CTRL register
        return s->CTRL;
    case 3: // DATA register
        return s->DATA;
    case 4: // MATCH register
        return s->MATCH;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "kllpuart_read: Bad offset %x\n", (int)offset);
    }
}

static void kllpuart_write(void *opaque, hwaddr offset,
                        uint64_t value, unsigned size)
{
    // this function is called whenever the user program writes to a register
    //   inside the range of this peripheral.
    KLLPUARTState *s = (KLLPUARTState *)opaque;
    unsigned char ch;

    switch (offset >> 2)
    {
    case 0: // BAUD register
        break;
    case 1: // STAT register
        break;
    case 2: // CTRL register
        break;
    case 3: // DATA register
        ch = value;
        if (s->chr)
            qemu_chr_fe_write(s->chr, &ch, 1);
        break;
    case 4: // MATCH register
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "kllpuart_write: Bad offset %x\n", (int)offset);
    }
}

static int kllpuart_can_receive(void *opaque)
{
    KLLPUARTState *s = (KLLPUARTState *)opaque;

    return true;
}

static void kllpuart_receive(void *opaque, const uint8_t *buf, int size)
{
}

static void kllpuart_event(void *opaque, int event)
{
    //if (event == CHR_EVENT_BREAK)
}

static const MemoryRegionOps kllpuart_ops = {
    .read = kllpuart_read,
    .write = kllpuart_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_kllpuart = {
    .name = "kllpuart",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(BAUD, KLLPUARTState),
        VMSTATE_UINT32(STAT, KLLPUARTState),
        VMSTATE_UINT32(CTRL, KLLPUARTState),
        VMSTATE_UINT32(DATA, KLLPUARTState),
        VMSTATE_UINT32(MATCH, KLLPUARTState),
        VMSTATE_END_OF_LIST()
    }
};

static void kllpuart_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    KLLPUARTState *s = KLLPUART(obj);

    memory_region_init_io(&s->iomem, OBJECT(s), &kllpuart_ops, s, "kllpuart", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void kllpuart_realize(DeviceState *dev, Error **errp)
{
    // connect the UART to whatever external serial device qemu is configured with (?)
    KLLPUARTState *s = KLLPUART(dev);

    s->chr = qemu_char_get_next_serial();

    if (s->chr) {
        qemu_chr_add_handlers(s->chr, kllpuart_can_receive, kllpuart_receive,
                              kllpuart_event, s);
    }
}

static void kllpuart_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = kllpuart_realize;
    dc->vmsd = &vmstate_kllpuart;
}

static const TypeInfo kllpuart_arm_info = {
    .name          = TYPE_KLLPUART,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(KLLPUARTState),
    .instance_init = kllpuart_init,
    .class_init    = kllpuart_class_init,
};

static void kllpuart_register_types(void)
{
    type_register_static(&kllpuart_arm_info);
}

type_init(kllpuart_register_types)

/*
 * Low-Power UART found in the kinetis KL series.
 *
 * Authored by Colin Wallace <wallace.colin.a@gmail.com>
 *
 * This code is regrettably licensed under the GPL.
 */

#include "hw/sysbus.h"
#include "sysemu/char.h"

#define TYPE_KLLPUART "kllpuart"
#define KLLPUART(obj) OBJECT_CHECK(KLLPUARTState, (obj), TYPE_KLLPUART)

// definitions for field locations inside the registers
#define STAT_TDRE_SHIFT 23 // Transmit Data Register Empty Flag
#define STAT_RDRF_SHIFT 21 // Receive Data Register Full Flag
#define STAT_OR_SHIFT   19 // Receiver Overrun Flag

#define CTRL_ORIE_SHIFT 27 // Overrun Interrupt Enable
#define CTRL_TIE_SHIFT  23 // Transmit Interrupt Enable
#define CTRL_RIE_SHIFT  21 // Receiver Interrupt Enable


typedef struct KLLPUARTState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    // device registers (publicly readable/writeable)
    uint32_t BAUD;
    uint32_t STAT;
    uint32_t CTRL;
    uint32_t DATA;
    uint32_t MATCH;

    CharDriverState *chr;
    qemu_irq irq;
} KLLPUARTState;


static void kllpuart_receive(void *opaque, const uint8_t *buf, int size);

// write `value` to `dest`, but do not affect any bit i for which mask[i] = 0.
static void write_masked(uint32_t *dest, uint32_t mask, uint32_t value)
{
    *dest = (*dest & ~mask) | (value & mask);
}

// "overloads" of the functions in qemu/bitops.h for u32 types
// set the `bit_idx` bit of *dest to 1.
static void set_bit_u32(int bit_idx, uint32_t *dest)
{
    write_masked(dest, 1 << bit_idx, 1 << bit_idx);
}

// set the `bit_idx` bit of *dest to 0.
static void clear_bit_u32(int bit_idx, uint32_t *dest)
{
    write_masked(dest, 1 << bit_idx, 0 << bit_idx);
}


// check if any of the flags which are configured to raise interrupts have been set.
// trigger NVIC if so
static void kllpuart_check_irq(KLLPUARTState *s)
{
    bool is_irq = ((s->STAT & (1 << STAT_TDRE_SHIFT)) && (s->CTRL & (1 << CTRL_TIE_SHIFT))) ||
                 ((s->STAT & (1 << STAT_RDRF_SHIFT)) && (s->CTRL & (1 << CTRL_RIE_SHIFT))) ||
                 ((s->STAT & (1 << STAT_OR_SHIFT))   && (s->CTRL & (1 << CTRL_ORIE_SHIFT)));

    qemu_set_irq(s->irq, is_irq);
}

// this function is called whenever the user program reads from a register
//   inside the range of this peripheral.
// this can trigger things like the clearing of IRQs, etc.
static uint64_t kllpuart_read(void *opaque, hwaddr offset,
                           unsigned size)
{
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
        // reading the data register clears the Receive Data Register Full Flag
        clear_bit_u32(STAT_RDRF_SHIFT, &s->STAT);
        kllpuart_check_irq(s);
        return s->DATA;
    case 4: // MATCH register
        return s->MATCH;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "kllpuart_read: Bad offset %x\n", (int)offset);
        return 0;
    }
}

static void kllpuart_write(void *opaque, hwaddr offset,
                        uint64_t value, unsigned size)
{
    // this function is called whenever the user program writes to a register
    //   inside the range of this peripheral.
    KLLPUARTState *s = (KLLPUARTState *)opaque;
    unsigned char ch = value;
    uint32_t value32 = value;

    switch (offset >> 2)
    {
    case 0: // BAUD register
        s->BAUD = value32;
        break;
    case 1: // STAT register
        // only bits 29:25 can be written
        write_masked(&s->STAT, 0x3e000000, value32);
        // TODO: some more bits are write-1-to-clear
        break;
    case 2: // CTRL register
        s->CTRL = value32;
        // altering the IRQ mask could cause a previously masked pending IRQ to now be unmasked
        kllpuart_check_irq(s);
        break;
    case 3: // DATA register
        // clear Transmit Data Register Empty Flag
        clear_bit_u32(STAT_TDRE_SHIFT, &s->STAT);
        kllpuart_check_irq(s);
        // write character to some serial device attached to QEMU (e.g. stdio)
        if (s->chr)
            qemu_chr_fe_write(s->chr, &ch, 1);
        // set Transmit Data Register Empty Flag
        set_bit_u32(STAT_TDRE_SHIFT, &s->STAT);
        kllpuart_check_irq(s);
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
    return true;
}

static void kllpuart_receive(void *opaque, const uint8_t *buf, int size)
{
    KLLPUARTState *s = (KLLPUARTState *)opaque;

    if (s->STAT & (1 << STAT_RDRF_SHIFT))
    {
        // attempting to write a byte but the receive register is full
        // therefore set the overrun flag
        set_bit_u32(STAT_OR_SHIFT, &s->STAT);
        kllpuart_check_irq(s);
    }
    else
    {
        // copy the character to the receive register
        write_masked(&s->DATA, 0xFF, *buf);
        // set the Receive Data Register Full Flag
        set_bit_u32(STAT_RDRF_SHIFT, &s->STAT);
        kllpuart_check_irq(s);
    }
}

static void kllpuart_event(void *opaque, int event)
{
}

static const MemoryRegionOps kllpuart_ops = {
    .read = kllpuart_read,
    .write = kllpuart_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_kllpuart = {
    .name = TYPE_KLLPUART,
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

    memory_region_init_io(&s->iomem, OBJECT(s), &kllpuart_ops, s, TYPE_KLLPUART, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    // register reset values
    s->BAUD  = 0x0F000004;
    s->STAT  = 0x00C00000;
    s->CTRL  = 0x00000000;
    s->DATA  = 0x00001000;
    s->MATCH = 0x00000000;
}

static void kllpuart_realize(DeviceState *dev, Error **errp)
{
    // connect the UART to whatever external serial device qemu is configured with
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

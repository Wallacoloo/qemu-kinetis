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

// the type of QEMU clock to use internally
// VIRTUAL is a high-resolution monotonic counter that pauses when the machine is suspended
#define CLOCK_TYPE_QEMU QEMU_CLOCK_VIRTUAL

#define CSR_TCF_SHIFT 7 // Timer Compare Flag bit index
#define CSR_TIE_SHIFT 6 // Timer Interrupt Enabled bit index
#define CSR_TFC_SHIFT 2 // Timer Free-Running Counter mode (who tf would declare both a TCF and a TFC field?)
#define CSR_TEN_SHIFT 0 // Timer Enable bit index
#define PSR_PRESCALE_WIDTH 4 // Timer Prescale Value has 4 bits
#define PSR_PRESCALE_SHIFT 3 // Timer Prescale Value start bit index
#define PSR_PBYP_SHIFT     2 // Timer Prescaler Bypass bit index

typedef struct KLLPTMRState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    // device registers (publicly readable/writeable)
    uint32_t CSR; // control status register
    uint32_t PSR; // prescale register
    uint32_t CMR; // compare register
    // private variables
    uint64_t time_base; // any time at which the counter should be 0, under its current settings
    QEMUTimer *timer;
    qemu_irq irq;
} KLLPTMRState;

// normal C division always truncates the result towards 0; i.e. 5/2 = 2, -5/2 =-2; 5/-2 = -2
// This returns the division of a/b, but any non-exact result is floored.
// Only valid for positive divisors
// For example, -5 / 2 will return -3, 
//   5 / 2 will return 2, 
//   -4 / 2 will return -2
//   5 / -2 will return -2 (only valid for positive divisors)
static int64_t floordiv_pos_divisor(int64_t a, int64_t b)
{
    return (a - (a<0 ? b-1 : 0)) / b;
}


// write `value` to `dest`, but do not affect any bit i for which mask[i] = 0.
// returns a mask indicating which bits were changed
static uint32_t write_masked(uint32_t *dest, uint32_t mask, uint32_t value)
{
    uint32_t old = *dest;

    *dest = (*dest & ~mask) | (value & mask);

    return old ^ *dest;
}

// certain registers contain flags that are cleared by attempting to write a 1 to them.
// These "write 1 to clear" (w1c) bits are indicated by `mask`
// all w1c bits in mask for which write_val = 1 will cause those bits in *reg to be set to 0
static uint32_t write1clear(uint32_t *reg, uint32_t mask, uint32_t write_val)
{
    return write_masked(reg, mask & write_val, 0);
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

// return true if CSR->TEN field is set
static bool kllptmr_is_enabled(KLLPTMRState *s)
{
    return (s->CSR & BIT(CSR_TEN_SHIFT)) != 0;
}

// get the timer's prescale multiplier, taking into account the bypass field
static int kllptmr_get_prescale(KLLPTMRState *s)
{
    // prescale value if bypass was disabled
    int prescale_exponent = (s->PSR >> PSR_PBYP_SHIFT) & ((1 << PSR_PRESCALE_WIDTH)-1);
    int prescale_field = 1 << prescale_exponent;

    return s->PSR & BIT(PSR_PBYP_SHIFT) ? 1 : prescale_field;
}

// return the frequency of the unscaled clock input
static int kllptmr_get_unscaled_clock(KLLPTMRState *s)
{
    // TODO: use the actual clock frequency
    return 1000;
}

// how many ns should elapse between each successive counter increment
static uint64_t kllptmr_ns_per_tick(KLLPTMRState *s)
{
    return ((uint64_t)1000000000UL) * kllptmr_get_prescale(s) / kllptmr_get_unscaled_clock(s);
}

// return the current value of the Counter Register, if it were read now
static uint32_t kllptmr_get_count(KLLPTMRState *s)
{
    if (kllptmr_is_enabled(s))
    {
        uint64_t rel_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->time_base;
        uint64_t count = rel_time / kllptmr_ns_per_tick(s);
        return count & 0xFFFF;
    }
    else
    {
        // when the timer is disabled, counter register is read as 0.
        return 0;
    }
}

// schedule the next time QEMU needs to trigger our timer for us to raise an IRQ
static void sched_next_qemu_interrupt(KLLPTMRState *s)
{
    uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t ns_per_tick = kllptmr_ns_per_tick(s);
    uint64_t ns_from_time_base = (s->CMR+1) * ns_per_tick;
    uint64_t ns_per_overflow = 0x10000 * ns_per_tick;

    // the next IRQ should happen at s->time_base + ns_from_time_base + k*ns_per_overflow
    // it should be such that k is the minimum integer for which the above value > `now`
    // therefore s->time_base + ns_from_time_base + (k-1)*ns_per_overflow < now < s->time_base + ns_from_time_base + k*ns_per_overflow
    // k-1 < (now - (s->time_base + ns_from_time_base))/ns_per_overflow < k
    int num_overflows = 1 + floordiv_pos_divisor((int64_t)now - (int64_t)(s->time_base + ns_from_time_base), (int64_t)ns_per_overflow);
    uint64_t ns_of_next_irq = s->time_base + ns_from_time_base + num_overflows * ns_per_overflow;

    timer_mod_ns(s->timer, ns_of_next_irq);
}


// check if any of the flags which are configured to raise interrupts have been set.
// trigger NVIC if so
static void kllptmr_check_irq(KLLPTMRState *s)
{
    bool is_irq = (s->CSR & BIT(CSR_TIE_SHIFT)) && (s->CSR & BIT(CSR_TCF_SHIFT));
    qemu_set_irq(s->irq, kllptmr_is_enabled(s) && is_irq);
}

// called by QEMU at a time we configure
static void kllptmr_qemu_interrupt(void * opaque)
{
    KLLPTMRState *s = (KLLPTMRState *)opaque;
    if (kllptmr_is_enabled(s))
    {
        // IRQs should not occur if the timer is disabled
        KLLPTMRState *s = (KLLPTMRState *)opaque;
        uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        if ((s->CSR & BIT(CSR_TFC_SHIFT)) == 0)
        {
            // timer value is reset on every interrupt triggered
            s->time_base = now;
        }
        // activate the Timer Compare Flag to trigger IRQ
        set_bit_u32(CSR_TCF_SHIFT, &s->CSR);
        sched_next_qemu_interrupt(s);
        kllptmr_check_irq(s);
    }
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
        return kllptmr_get_count(s);
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
        {
            bool was_prev_enabled = kllptmr_is_enabled(s);
            write_masked(&s->CSR, 0x41, value32);
            bool just_enabled = !was_prev_enabled && kllptmr_is_enabled(s);
            bool just_disabled = was_prev_enabled && !kllptmr_is_enabled(s);

            // handle w1c Timer Compare Flag:
            write1clear(&s->CSR, BIT(CSR_TCF_SHIFT), value32);
            if (just_disabled)
            {
                // clear the Timer Compare Flag upon disabling
                clear_bit_u32(CSR_TCF_SHIFT, &s->CSR);
            }
            if (kllptmr_is_enabled(s))
            {
                // bits 5:1 can not be written when the timer is enabled
                //   nor WHILE the timer is being enabled (by a write=1 to TEN)
                uint32_t bits_changed = write_masked(&s->CSR, 0x3E, value32);
                if (bits_changed & 0x39)
                {
                    qemu_log_mask(LOG_GUEST_ERROR,
                      "kllptmr_write: Features not yet supported: CSR=%x\n", (int)value32);
                }
            }
            if (just_enabled)
            {
                // upon enabling the timer, the counter should have been set to 0
                //   therefore time_base is now.
                s->time_base = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
                sched_next_qemu_interrupt(s);
            }
            kllptmr_check_irq(s);
        }
        break;
    case 1: // Prescale Register
        if (!kllptmr_is_enabled(s))
        {
            // all of PSR can only be modified when timer is disabled
            uint32_t bits_changed = write_masked(&s->PSR, 0x7F, value32);
            if (bits_changed & 0x3)
            {
                qemu_log_mask(LOG_GUEST_ERROR,
                      "kllptmr_write: Features not yet supported: PSR=%x\n", (int)value32);
            }
        }
        break;
    case 2: // Compare Register
        if (!kllptmr_is_enabled(s) || (s->CSR & BIT(CSR_TCF_SHIFT)))
        {
            // CMR can only be modified while timer is disabled or Timer Compare Flag is set
            s->CMR = value32 & 0xFFFF;
            if (kllptmr_is_enabled(s))
            {
                sched_next_qemu_interrupt(s);
            }
        }
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

    memory_region_init_io(&s->iomem, OBJECT(s), &kllptmr_ops, s, TYPE_KLLPTMR, 0x1000);
    sysbus_init_mmio(dev, &s->iomem);

    sysbus_init_irq(dev, &s->irq);

    // register reset values
    s->CSR = 0x00000000;
    s->PSR = 0x00000000;
    s->CMR = 0x00000000;

    s->timer = timer_new_ns(CLOCK_TYPE_QEMU, kllptmr_qemu_interrupt, s);
    return 0;
}

static const VMStateDescription vmstate_kllptmr = {
    .name = TYPE_KLLPTMR,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(CSR, KLLPTMRState),
        VMSTATE_UINT32(PSR, KLLPTMRState),
        VMSTATE_UINT32(CMR, KLLPTMRState),
        VMSTATE_UINT64(time_base, KLLPTMRState),
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

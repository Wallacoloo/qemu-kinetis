/*
 * General-Purpose Input/Output controls found in the kinetis KL series.
 * Each instance of TYPE_KLGPIO represents only 1 PORT.
 * So if the device has, say, both PORTA and PORTB, one need instantiate 2 KLGPIO peripherals.
 *
 * Authored by Colin Wallace <wallace.colin.a@gmail.com>
 *
 * This code is regrettably licensed under the GPL.
 */

#include "qemu/bitops.h"
#include "hw/hw.h"
#include "hw/sysbus.h"

#define TYPE_KLGPIO "klgpio"
#define KLGPIO(obj) OBJECT_CHECK(KLGPIOState, (obj), TYPE_KLGPIO)

#define PORT_PCR_ISF_SHIFT 24 // Interrupt Status Flag bit index


typedef struct KLGPIOState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    // device registers (publicly readable/writeable)
    uint32_t PORT_PCR[32];
    //uint32_t PORT_GPCLR; // write-only
    //uint32_t PORT_GPCHR; // write-only
    //uint32_t reserved[6]
    //uint32_t PORT_ISFR; // concatenation of all PORT_PCRn[ISF] bits

    // private variables
    qemu_irq irq[32];
} KLGPIOState;


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

// check if any of the flags which are configured to raise interrupts have been set.
// trigger NVIC if so
static void klgpio_check_irq(KLGPIOState *s)
{
	int p;
	for (p=0; p<32; ++p)
	{
	    bool is_irq = (s->PORT_PCR[p] & BIT(PORT_PCR_ISF_SHIFT)) != 0;
	    qemu_set_irq(s->irq[p], is_irq);
	}
}

static uint64_t klgpio_read(void *opaque, hwaddr offset, unsigned size)
{
    KLGPIOState *s = (KLGPIOState*)opaque;

    switch (offset >> 2) {
    	case 0: case 1: case 2: case 3: case 4: case 5: case 6: case 7: 
    	case 8: case 9: case 10: case 11: case 12: case 13: case 14: case 15: 
    	case 16: case 17: case 18: case 19: case 20: case 21: case 22: case 23: 
    	case 24: case 25: case 26: case 27: case 28: case 29: case 30: case 31: 
    		// PORT_PCRn - Port Control Register n
    		return s->PORT_PCR[offset >> 2];
    	case 32: // PORT_GPCLR - Global Pin Control Low Reg
    	case 33: // PORT_GPCHR - Global Pin Control High Reg
    		return 0; // these registers always read as 0
    	// [34-39 are unmapped]
    	case 40: // PORT_ISFR - Interrupt Status Flag Reg
    		{
    			// construct the ISFR value by reading all the Pins' Interrupt Status Flags
    			uint32_t ISFR = 0x00000000;
    			int p;
    			for (p=0; p<32; ++p)
    			{
    				bool ISF = (s->PORT_PCR[p] & BIT(PORT_PCR_ISF_SHIFT)) != 0;
    				ISFR |= ISF << p;
    			}
    			return ISFR;
    		}
    	default:
    		qemu_log_mask(LOG_GUEST_ERROR,
                      "klgpio_read: Bad offset 0x%x\n", (int)offset);
    		break;
    }
    return 0;
}

static void klgpio_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    KLGPIOState *s = (KLGPIOState*)opaque;

    uint32_t value32 = value;

    switch (offset >> 2) {
    	case 0: case 1: case 2: case 3: case 4: case 5: case 6: case 7: 
    	case 8: case 9: case 10: case 11: case 12: case 13: case 14: case 15: 
    	case 16: case 17: case 18: case 19: case 20: case 21: case 22: case 23: 
    	case 24: case 25: case 26: case 27: case 28: case 29: case 30: case 31: // PORT_PCRn
    		write1clear(&s->PORT_PCR[offset >> 2], BIT(PORT_PCR_ISF_SHIFT), value32);
    		s->PORT_PCR[offset >> 2] = value32 & 0x000F0757;
    		klgpio_check_irq(s);
    		break;
    	case 32: // PORT_GPCLR
    	case 33: // PORT_GPCHR
    		// TODO: Implement
    		break;
    	// [34-39 are unmapped]
    	case 40: // PORT_ISFR
	    	{
	    		int p;
	    		for (p=0; p<32; ++p)
	    		{
	    			bool do_clear = (value32 & BIT(p)) != 0;
	    			write1clear(&s->PORT_PCR[p], BIT(PORT_PCR_ISF_SHIFT), BIT(do_clear));
	    		}
	    	}
    		// Some IRQs may have just been cleared
    		klgpio_check_irq(s);
    		break;
    	default:
    		qemu_log_mask(LOG_GUEST_ERROR,
                      "klgpio_write: Bad offset 0x%x\n", (int)offset);
    		break;
    }
}

static const MemoryRegionOps klgpio_ops = {
    .read = klgpio_read,
    .write = klgpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int klgpio_init(SysBusDevice *dev)
{
    KLGPIOState *s = KLGPIO(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &klgpio_ops, s, TYPE_KLGPIO, 0x2000);
    sysbus_init_mmio(dev, &s->iomem);

    // register reset values
    int p;
    for (p=0; p<32; ++p)
    {
    	// TODO: MUX, DSE, PFE, SRC, PE, PS fields may have non-zero reset values
    	s->PORT_PCR[p] = 0x00000000;
    }

    return 0;
}

static void klgpio_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);

    sdc->init = klgpio_init;
}

static const TypeInfo klgpio_info = {
    .name = TYPE_KLGPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(KLGPIOState),
    .class_init = klgpio_class_init,
};

static void klgpio_register_type(void)
{
    type_register_static(&klgpio_info);
}

type_init(klgpio_register_type)

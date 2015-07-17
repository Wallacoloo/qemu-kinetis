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

#define NUM_PORTS 2
#define PORT_PCR_ISF_SHIFT 24 // Interrupt Status Flag bit index
#define PORT_PCR_IRQC_SHIFT 16 // Interrupt Configuration bits
#define PORT_PCR_IRQC_WIDTH 4


typedef struct KLPORTEntry {
    // device registers (publicly readable/writeable)
    uint32_t PCR[32];
    //uint32_t PORT_GPCLR; // write-only
    //uint32_t PORT_GPCHR; // write-only
    //uint32_t reserved[6]
    //uint32_t PORT_ISFR; // concatenation of all PCRn[ISF] bits

    // private variables
    qemu_irq irq;
} KLPORTEntry;

typedef struct KLGPIOEntry {
    // device registers (publicly readable/writeable)
    uint32_t PDOR; // Port Data Output Register
    //uint32_t PSOR; // Port Set Output Register (write-only)
    //uint32_t PCOR; // Port Clear Output Register (write-only)
    //uint32_t PTOR; // Port Toggle Output Register (write-only)
    uint32_t PDIR; // Port Data Input Register. Usually read-only
    // PDIR is usually read-only, but we allow writes because there 
    // doesn't appear to be any documented way to connect a gpio to a 
    // source external to qemu, and this allows to do so by using a debugger
    uint32_t PDDR; // Port Data Direction Register
} KLGPIOEntry;

typedef struct KLGPIOState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    KLPORTEntry ports[NUM_PORTS];
    KLGPIOEntry gpios[NUM_PORTS];
} KLGPIOState;


static void klgpioentry_set_io_vals(KLGPIOState *s, int port_id, uint32_t pdor, uint32_t pdir);


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
static void klportentry_check_irq(KLPORTEntry *s)
{
    bool is_irq = false;
	int p;
	for (p=0; p<32; ++p)
	{
	    is_irq |= (s->PCR[p] & BIT(PORT_PCR_ISF_SHIFT)) != 0;
	}
    qemu_set_irq(s->irq, is_irq);
}

static uint64_t klportentry_read(KLGPIOState *s, int port_id, hwaddr offset, unsigned size)
{
    KLPORTEntry *port = &s->ports[port_id];
    switch (offset >> 2) {
    	case 0: case 1: case 2: case 3: case 4: case 5: case 6: case 7: 
    	case 8: case 9: case 10: case 11: case 12: case 13: case 14: case 15: 
    	case 16: case 17: case 18: case 19: case 20: case 21: case 22: case 23: 
    	case 24: case 25: case 26: case 27: case 28: case 29: case 30: case 31: 
    		// PCRn - Port Control Register n
    		return port->PCR[offset >> 2];
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
    				bool ISF = (port->PCR[p] & BIT(PORT_PCR_ISF_SHIFT)) != 0;
    				ISFR |= ISF << p;
    			}
    			return ISFR;
    		}
    	default:
    		qemu_log_mask(LOG_GUEST_ERROR,
                      "klportentry_read: Bad offset 0x%x\n", (int)offset);
    		break;
    }
    return 0xDEADBEEF;
}

static void klportentry_write(KLGPIOState *s, int port_id, hwaddr offset, uint64_t value, unsigned size)
{
    KLPORTEntry *port = &s->ports[port_id];
    KLGPIOEntry *gpio = &s->gpios[port_id];
    uint32_t value32 = value;

    switch (offset >> 2) {
    	case 0: case 1: case 2: case 3: case 4: case 5: case 6: case 7: 
    	case 8: case 9: case 10: case 11: case 12: case 13: case 14: case 15: 
    	case 16: case 17: case 18: case 19: case 20: case 21: case 22: case 23: 
    	case 24: case 25: case 26: case 27: case 28: case 29: case 30: case 31: // PORT_PCRn
    		write1clear(&port->PCR[offset >> 2], BIT(PORT_PCR_ISF_SHIFT), value32);
    		port->PCR[offset >> 2] = value32 & 0x000F0757;
    		klportentry_check_irq(port);
            // Clearing the flag should cause it to retrigger if IRQ set for logic 0/1
            //   & pin has the same value:
            klgpioentry_set_io_vals(s, port_id, gpio->PDOR, gpio->PDIR);
    		break;
    	case 32: // PORT_GPCLR - Global Pin Control Low Reg
            {
                uint16_t pin_mask = (value32 & (0xFFFF0000)) >> 16;
                uint16_t write_data = value32 & 0x0000FFFF;
                // write `write_data` to all PCR[n][0:15] for n in pin_mask
                int pin;
                for (pin=0; pin<32; ++pin)
                {
                    if (pin_mask & BIT(pin))
                    {
                        uint32_t new_value = (port->PCR[pin] & 0xFFFF0000) | write_data;
                        klportentry_write(s, port_id, pin*4, new_value, 4);
                    }
                }
            }
            break;
    	case 33: // PORT_GPCHR - Global Pin Control High Reg
    		{
                uint16_t pin_mask = (value32 & (0xFFFF0000)) >> 16;
                uint16_t write_data = value32 & 0x0000FFFF;
                // write `write_data` to all PCR[n][0:15] for n in pin_mask
                int pin;
                for (pin=0; pin<32; ++pin)
                {
                    if (pin_mask & BIT(pin))
                    {
                        uint32_t new_value = (port->PCR[pin] & 0x0000FFFF) | (write_data << 16);
                        klportentry_write(s, port_id, pin*4, new_value, 4);
                    }
                }
            }
            break;
    		break;
    	// [34-39 are unmapped]
    	case 40: // PORT_ISFR - Interrupt Status Flags Reg
	    	{
	    		int p;
	    		for (p=0; p<32; ++p)
	    		{
	    			bool do_clear = (value32 & BIT(p)) != 0;
	    			write1clear(&port->PCR[p], BIT(PORT_PCR_ISF_SHIFT), BIT(do_clear));
	    		}
	    	}
    		// Some IRQs may have just been cleared
    		// Clearing the flag should cause it to retrigger if IRQ set for logic 0/1
            //   & pin has the same value:
            klgpioentry_set_io_vals(s, port_id, gpio->PDOR, gpio->PDIR);
    		break;
    	default:
    		qemu_log_mask(LOG_GUEST_ERROR,
                      "klportentry_write: Bad offset 0x%x\n", (int)offset);
    		break;
    }
}



static void klgpioentry_set_io_vals(KLGPIOState *s, int port_id, uint32_t pdor, uint32_t pdir)
{
    // calculate bitmasks of pins that just experiences rising/falling edges
    KLGPIOEntry *gpio = &s->gpios[port_id];
    KLPORTEntry *port = &s->ports[port_id];
    uint32_t rising_edges = (~gpio->PDIR) & pdir;
    uint32_t falling_edges = gpio->PDIR & (~pdir);

    int pin;
    for (pin=0; pin<32; ++pin)
    {
        bool rising = (rising_edges & BIT(pin)) != 0;
        bool falling = (falling_edges & BIT(pin)) != 0;
        bool logic_high = (pdir & BIT(pin)) != 0;
        bool logic_low = !logic_high;
        uint32_t IRQC_field = (port->PCR[pin] >> PORT_PCR_IRQC_SHIFT) & ((1 << PORT_PCR_IRQC_WIDTH)-1);
        bool is_new_irq = (IRQC_field == 0x8 && logic_low)
                       || (IRQC_field == 0x9 && rising)
                       || (IRQC_field == 0xA && falling)
                       || (IRQC_field == 0xB && (rising || falling))
                       || (IRQC_field == 0xC && logic_high);
        port->PCR[pin] |= is_new_irq << PORT_PCR_ISF_SHIFT;
    }
    klportentry_check_irq(port);

    gpio->PDOR = pdor;
    gpio->PDIR = pdir;
}

static uint64_t klgpioentry_read(KLGPIOState *s, int port_id, hwaddr offset, unsigned size)
{
    KLGPIOEntry *gpio = &s->gpios[port_id];
    switch (offset >> 2) {
        case 0: // Port Data Output Register
            return gpio->PDOR;
        case 1: // Port Set Output Register
        case 2: // Port Clear Output Register
        case 3: // Port Toggle Output Register
            return 0x00000000; // write-only
        case 4: // Port Data Input Register
            return gpio->PDIR;
        case 5: // Port Data Direction Register
            return gpio->PDDR;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                      "klgpioentry_read: Bad offset 0x%x\n", (int)offset);
            break;
    }
    return 0xDEADBEEF;
}

static void klgpioentry_write(KLGPIOState *s, int port_id, hwaddr offset, uint64_t value, unsigned size)
{
    KLGPIOEntry *gpio = &s->gpios[port_id];

    uint32_t value32 = value;
    switch (offset >> 2) {
        case 0: // Port Data Output Register
            klgpioentry_set_io_vals(s, port_id, value32, gpio->PDIR);
            break;
        case 1: // Port Set Output Register
            klgpioentry_set_io_vals(s, port_id, gpio->PDOR | value32, gpio->PDIR);
            break;
        case 2: // Port Clear Output Register
            klgpioentry_set_io_vals(s, port_id, gpio->PDOR & (~value32), gpio->PDIR);
            break;
        case 3: // Port Toggle Output Register
            klgpioentry_set_io_vals(s, port_id, gpio->PDOR ^ value32, gpio->PDIR);
            break;
        case 4: // Port Data Input Register
            klgpioentry_set_io_vals(s, port_id, gpio->PDOR, value32);
            break;
        case 5: // Port Data Direction Register
            gpio->PDDR = value32;
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                      "klgpioentry_write: Bad offset 0x%x\n", (int)offset);
            break;
    }
}



static uint64_t klgpio_read(void *opaque, hwaddr offset, unsigned size)
{
    KLGPIOState *s = (KLGPIOState*)opaque;
    if (offset < 0x1000*NUM_PORTS)
    {
        // read one of the PORT structures
        // each structre occupies 0x1000 bytes
        return klportentry_read(s, offset/0x1000, offset%0x1000, size);
    }
    else if (offset >= 0x6000 && offset <= 0x7000)
    {
        // read one of the GPIO structures
        // each structure occupies 0x40 bytes
        hwaddr rel_offset = offset-0x6000;
        return klgpioentry_read(s, rel_offset/0x40, rel_offset%0x40, size);
    }
    else
    {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "klgpio_read: Bad offset 0x%x\n", (int)offset);
        return 0xDEADBEEF;
    }
}

static void klgpio_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    KLGPIOState *s = (KLGPIOState*)opaque;
    if (offset < 0x1000*NUM_PORTS)
    {
        // write one of the PORT structures
        // each structre occupies 0x1000 bytes
        klportentry_write(s, offset/0x1000, offset%0x1000, value, size);
    }
    else if (offset >= 0x6000 && offset <= 0x7000)
    {
        // write one of the GPIO structures
        // each structure occupies 0x40 bytes
        hwaddr rel_offset = offset-0x6000;
        klgpioentry_write(s, rel_offset/0x40, rel_offset%0x40, value, size);
    }
    else
    {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "klgpio_read: Bad offset 0x%x\n", (int)offset);
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

    // Covers 0x40049000 - 0x4004FFFF
    memory_region_init_io(&s->iomem, OBJECT(s), &klgpio_ops, s, TYPE_KLGPIO, 0x7000);
    sysbus_init_mmio(dev, &s->iomem);

    // register reset values
    int port, pin;
    for (port=0; port<NUM_PORTS; ++port)
    {
        sysbus_init_irq(dev, &s->ports[port].irq);
        for (pin=0; pin<32; ++pin)
        {
            // TODO: MUX, DSE, PFE, SRC, PE, PS fields may have non-zero reset values
            s->ports[port].PCR[pin] = 0x00000000;
            s->gpios[port].PDOR = 0x00000000;
            s->gpios[port].PDIR = 0x00000000;
            s->gpios[port].PDDR = 0x00000000;
            s->gpios[port].PDOR = 0x00000000;
        }
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

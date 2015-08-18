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

#define NUM_PORTS 2

#define TYPE_KLGPIO "klgpio"
#define KLGPIO(obj) OBJECT_CHECK(KLGPIOState, (obj), TYPE_KLGPIO)

#define TYPE_KLPORT "klport"
#define KLPORT(obj) OBJECT_CHECK(KLPORTState, (obj), TYPE_KLPORT)

#define PORT_PCR_MUX_SHIFT 8 // Pin Multiplex Mode bit index
#define PORT_PCR_MUX_WIDTH 3

#define PORT_PCR_ISF_SHIFT 24 // Interrupt Status Flag bit index
#define PORT_PCR_IRQC_SHIFT 16 // Interrupt Configuration bits
#define PORT_PCR_IRQC_WIDTH 4


typedef struct KLPORTState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    // device registers (publicly readable/writeable)
    uint32_t PCR[32];
    //uint32_t PORT_GPCLR; // write-only
    //uint32_t PORT_GPCHR; // write-only
    //uint32_t reserved[6]
    //uint32_t PORT_ISFR; // concatenation of all PCRn[ISF] bits

    // private variables
    qemu_irq irq;
    uint32_t last_known_pdir; // last known input value of the pin - used to detect edge interrupts
} KLPORTState;

typedef struct KLGPIOBank {
    uint32_t PDOR; // Port Data Output Register
    //uint32_t PSOR; // Port Set Output Register (write-only)
    //uint32_t PCOR; // Port Clear Output Register (write-only)
    //uint32_t PTOR; // Port Toggle Output Register (write-only)
    uint32_t PDIR; // Port Data Input Register. Usually read-only
    // PDIR is usually read-only, but we allow writes because there 
    // doesn't appear to be any documented way to connect a gpio to a 
    // source external to qemu, and this allows to do so by using a debugger
    uint32_t PDDR; // Port Data Direction Register
} KLGPIOBank;

typedef struct KLGPIOState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    // device registers (publicly readable/writeable)
    KLGPIOBank banks[NUM_PORTS];
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
static void klport_check_irq(KLPORTState *port)
{
    // need to read the Port Data Input register under the GPIO peripheral
    //   (not the PORT peripheral) to determine IRQ states
    uint32_t new_pdir;
    uint32_t periph_base_addr = port->parent_obj.mmio[0].addr;
    uint32_t port_number = (periph_base_addr-0x40049000) / 0x1000;
    uint32_t gpio_base_addr = 0x400FF000 + 0x40*port_number;
    uint32_t gpio_pdir_addr = gpio_base_addr+0x10;
    cpu_physical_memory_read(gpio_pdir_addr, &new_pdir, 4);

    uint32_t rising_edges = (~port->last_known_pdir) & new_pdir;
    uint32_t falling_edges = port->last_known_pdir & (~new_pdir);

    port->last_known_pdir = new_pdir;

    // update the individual Interrupt Status Flag for each pin
    int pin;
    for (pin=0; pin<32; ++pin)
    {
        // determine which transitions the pin just made
        bool rising = (rising_edges & BIT(pin)) != 0;
        bool falling = (falling_edges & BIT(pin)) != 0;
        bool logic_high = (new_pdir & BIT(pin)) != 0;
        bool logic_low = !logic_high;
        // check if the pin is enabled to trigger on IRQ on whichever transition was made
        uint32_t IRQC_field = (port->PCR[pin] >> PORT_PCR_IRQC_SHIFT) & ((1 << PORT_PCR_IRQC_WIDTH)-1);
        bool is_new_irq = (IRQC_field == 0x8 && logic_low)
                       || (IRQC_field == 0x9 && rising)
                       || (IRQC_field == 0xA && falling)
                       || (IRQC_field == 0xB && (rising || falling))
                       || (IRQC_field == 0xC && logic_high);
        port->PCR[pin] |= is_new_irq << PORT_PCR_ISF_SHIFT;
    }

    // determine if there's at least one IRQ on the port; relay to NVIC
    bool is_irq = false;
	for (pin=0; pin<32; ++pin)
	{
	    is_irq |= (port->PCR[pin] & BIT(PORT_PCR_ISF_SHIFT)) != 0;
	}
    qemu_set_irq(port->irq, is_irq);

    // Check if the NMI (Non-maskable interrupt) pin is triggering an IRQ
    // if so, this is treated as a special ARM IRQ rather than a peripheral IRQ.
    if (port_number == 1)
    {
        // NMI pin is Port B pin 5.
        uint8_t mux_mode = port->PCR[5] >> PORT_PCR_MUX_SHIFT & ((1 << PORT_PCR_MUX_WIDTH)-1);
        bool is_muxed_as_nmi = (mux_mode == 3);
        bool is_pin_active = (new_pdir & BIT(5)) == 0; // NMI pin is active low
        bool is_nmi_irq = is_muxed_as_nmi && is_pin_active;

        //qemu_set_irq(port->nmi_irq, is_nmi_irq);
        // write to the arm address that triggers / clears the NMI IRQ flag
        uint32_t addr = 0xE000E000 + 0x0D00 + 0x04;
        uint32_t icsr_value; // value in the ARM Interrupt Control & State Register
        // set bit 31 of icsr (NMIPENDSET)
        cpu_physical_memory_read(addr, &icsr_value, 4);
        write_masked(&icsr_value, 1 << 31, is_nmi_irq << 31);
        cpu_physical_memory_write(addr, &icsr_value, 4);
    }
}

static uint64_t klport_read(void *opaque, hwaddr offset, unsigned size)
{
    KLPORTState *port = (KLPORTState*)opaque;
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
                // reading the ISFR is a way that the GPIO peripheral notifies us of a pin input change,
                // so recalculate the IRQs
                klport_check_irq(port);
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
                      "klport_read: Bad offset 0x%x\n", (int)offset);
    		break;
    }
    return 0xDEADBEEF;
}

static void klport_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    KLPORTState *port = (KLPORTState*)opaque;
    uint32_t value32 = value;

    switch (offset >> 2) {
    	case 0: case 1: case 2: case 3: case 4: case 5: case 6: case 7: 
    	case 8: case 9: case 10: case 11: case 12: case 13: case 14: case 15: 
    	case 16: case 17: case 18: case 19: case 20: case 21: case 22: case 23: 
    	case 24: case 25: case 26: case 27: case 28: case 29: case 30: case 31: // PORT_PCRn
    		write1clear(&port->PCR[offset >> 2], BIT(PORT_PCR_ISF_SHIFT), value32);
    		port->PCR[offset >> 2] = value32 & 0x000F0757;
    		klport_check_irq(port);
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
                        klport_write(port, pin*4, new_value, 4);
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
                        klport_write(port, pin*4, new_value, 4);
                    }
                }
            }
    		break;
    	// [34-39 are unmapped]
    	case 40: // PORT_ISFR - Interrupt Status Flags Reg
	    	{
	    		int p;
	    		for (p=0; p<32; ++p)
	    		{
	    			bool do_clear = (value32 & BIT(p)) != 0;
	    			write1clear(&port->PCR[p], BIT(PORT_PCR_ISF_SHIFT), do_clear << PORT_PCR_ISF_SHIFT);
	    		}
	    	}
    		// Some IRQs may have just been cleared
            klport_check_irq(port);
    		break;
    	default:
    		qemu_log_mask(LOG_GUEST_ERROR,
                      "klportentry_write: Bad offset 0x%x\n", (int)offset);
    		break;
    }
}


static uint64_t klgpio_read(void *opaque, hwaddr offset, unsigned size)
{
    KLGPIOState *gpio = (KLGPIOState*)opaque;
    KLGPIOBank *bank = &gpio->banks[offset / 0x40];
    uint32_t rel_offset = offset % 0x40;
    switch (rel_offset >> 2) {
        case 0: // Port Data Output Register
            return bank->PDOR;
        case 1: // Port Set Output Register
        case 2: // Port Clear Output Register
        case 3: // Port Toggle Output Register
            return 0x00000000; // write-only
        case 4: // Port Data Input Register
            return bank->PDIR;
        case 5: // Port Data Direction Register
            return bank->PDDR;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                      "klgpioentry_read: Bad offset 0x%x\n", (int)offset);
            break;
    }
    return 0xDEADBEEF;
}

static void klgpio_set_io_vals(KLGPIOBank *bank, int bank_idx, uint32_t pdor, uint32_t pdir)
{
    bank->PDOR = pdor;
    bank->PDIR = pdir;
    
    // trigger a read to the PORT peripheral which notifies it to update its IRQs
    uint32_t dummy;
    uint32_t port_base_addr = 0x40049000 + 0x1000*bank_idx;
    uint32_t port_isfr_addr = port_base_addr+4*40;
    cpu_physical_memory_read(port_isfr_addr, &dummy, 4);
}

static void klgpio_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    KLGPIOState *gpio = (KLGPIOState*)opaque;
    int bank_num = offset / 0x40;
    KLGPIOBank *bank = &gpio->banks[bank_num];
    uint32_t rel_offset = offset % 0x40;
    uint32_t value32 = value;
    switch (rel_offset >> 2) {
        case 0: // Port Data Output Register
            klgpio_set_io_vals(bank, bank_num, value32, bank->PDIR);
            break;
        case 1: // Port Set Output Register
            klgpio_set_io_vals(bank, bank_num, bank->PDOR | value32, bank->PDIR);
            break;
        case 2: // Port Clear Output Register
            klgpio_set_io_vals(bank, bank_num, bank->PDOR & (~value32), bank->PDIR);
            break;
        case 3: // Port Toggle Output Register
            klgpio_set_io_vals(bank, bank_num, bank->PDOR ^ value32, bank->PDIR);
            break;
        case 4: // Port Data Input Register
            klgpio_set_io_vals(bank, bank_num, bank->PDOR, value32);
            break;
        case 5: // Port Data Direction Register
            bank->PDDR = value32;
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                      "klgpioentry_write: Bad offset 0x%x\n", (int)offset);
            break;
    }
}




static const MemoryRegionOps klport_ops = {
    .read = klport_read,
    .write = klport_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int klport_init(SysBusDevice *dev)
{
    KLPORTState *s = KLPORT(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &klport_ops, s, TYPE_KLPORT, 0x1000);
    sysbus_init_mmio(dev, &s->iomem);
    sysbus_init_irq(dev, &s->irq);

    // register reset values
    int pin;
    for (pin=0; pin<32; ++pin)
    {
        // TODO: MUX, DSE, PFE, SRC, PE, PS fields may have non-zero reset values
        s->PCR[pin] = 0x00000000;
    }

    return 0;
}

static void klport_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);

    sdc->init = klport_init;
}

static const TypeInfo klport_info = {
    .name = TYPE_KLPORT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(KLPORTState),
    .class_init = klport_class_init,
};


static const MemoryRegionOps klgpio_ops = {
    .read = klgpio_read,
    .write = klgpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int klgpio_init(SysBusDevice *dev)
{
    KLGPIOState *s = KLGPIO(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &klgpio_ops, s, TYPE_KLGPIO, 0x1000);
    sysbus_init_mmio(dev, &s->iomem);

    // register reset values
    int bank;
    for (bank=0; bank<NUM_PORTS; ++bank)
    {
        s->banks[bank].PDOR = 0x00000000;
        s->banks[bank].PDIR = 0x00000000;
        s->banks[bank].PDDR = 0x00000000;
        s->banks[bank].PDOR = 0x00000000;
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

static void klgpio_register_types(void)
{
    type_register_static(&klport_info);
    type_register_static(&klgpio_info);
}

type_init(klgpio_register_types)

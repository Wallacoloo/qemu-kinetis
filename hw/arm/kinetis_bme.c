/*
 * Bit Manipulation Engine (BME) found in the kinetis KL series (see KL03P24M48SF0RM.pdf sect 22).
 * The BME allows to make better use of the full 32-bit address space when not all bits are needed.
 * It uses bits 27:26, and 30:29 (0-based indexing)
 * in order to define various atomic read-modify-write operations.
 * This is, instead of `x = x | 0x25;`, one can write `*(&x | BME_OR_MASK) = 0x25;` to do so atomically.
 *
 * bits 19:0 indicate the unmasked address, relative to some base address.
 *
 * The three decorated storage options supported are bitwise-and, bitwise-or, and bitwise-xor.
 * bitwise-and address mask: 0x04000000
 * bitwise-or address mask:  0x08000000
 * bitwise-xor address maskL 0x0C000000
 *
 * bits 30-29 indicate the address space for the operation.
 * if addr[30:29] == 01, the access is to SRAM_U (base address = 0x20000000),
 * if addr[30:29] == 10, the address is relative to a peripheral (base address = 0x40000000)
 *
 * There is also a decorated store bit field insert operation,
 * which can be used to write to only a specific bitfield of a register.
 * i.e. x = (x & mask) | (data & ~mask)
 *
 * Authored by Colin Wallace <wallace.colin.a@gmail.com>
 *
 * This code is regrettably licensed under the GPL.
 */

#include "hw/sysbus.h"

#define TYPE_KLBME "klbme"
#define KLBME(obj) OBJECT_CHECK(KLBMEState, (obj), TYPE_KLBME)


typedef struct KLBMEState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
} KLBMEState;

// infers the size of a read/write based on its address alignment.
// i.e. addresses aligned to 4 bytes must be read/written in whole words.
// addresses aligned to 2-bytes are written as half-words,
// addresses aligned to a single byte are written as bytes
static uint32_t operand_size_from_alignment(uint32_t addr)
{
	if (addr & 0x1)
	{
		// byte-aligned read
		return 1;
	}
	else if (addr & 0x2)
	{
		// half-word-aligned read
		return 2;
	}
	else
	{
		// word-aligned read
		return 4;
	}
}

static uint64_t klbme_read(void *opaque, hwaddr offset,
                           unsigned size)
{
    KLBMEState *s = (KLBMEState *)opaque;

    // in order to map just specific memory regions to the BME,
    // multiple BME instances are created with different base addresses / lengths.
    // Therefore, to get the full physical address, we have to know our base address.
    uint32_t bme_periph_base_addr = s->parent_obj.mmio[0].addr;
    // physical address the instruction tried to read, including decoration bits.
    uint32_t decorated_addr = offset + bme_periph_base_addr;

    uint8_t op_type = (decorated_addr & 0x1C000000) >> 26;

    int read_size = operand_size_from_alignment(decorated_addr);

 	uint32_t b; // b-field of address
 	uint32_t phys_addr;
    uint32_t unmodified_value; // read the data before modifying it
    if (op_type & 0x4)
    {
    	// op_type = load unsigned bit field extract (does not perform any writeback)
    	uint32_t w;
        b = (decorated_addr & ( 0x03800000 | ((read_size-1) << 26) ))  >> 23;
        w = (decorated_addr & ( 0x00380000 | (((read_size-1)&0x1) << 22) ))  >> 19;
        uint32_t mask = ( (1 << (w+1)) - 1 ) << b;

    	// offset is specified as 5 nibbles, less one bit
        //   and is relative to some base address indicated by bits 30:29
	    phys_addr = decorated_addr & 0x6007FFFF;
	    cpu_physical_memory_read(phys_addr, &unmodified_value, read_size);

        return (unmodified_value & mask) >> b;
    }
    else
    {
    	// op_type = either load-and-clear 1 bit (0x2) or load-and-set 1 bit (0x1)
	    // value to write back to memory
	    uint32_t writeback_value;

	    // offset is specified as 5 nibbles and is relative to some base address indicated by bits 30:29
	    phys_addr = decorated_addr & 0x600FFFFF;
	    cpu_physical_memory_read(phys_addr, &unmodified_value, read_size);

	    b = (decorated_addr & ( 0x00E00000 | ((read_size-1) << 24) ))  >> 21;
	    
        if (op_type == 0x2)
	    {
	        // load and clear 1 bit
	        writeback_value = unmodified_value & ~(1 << b);
	    }
	    else if (op_type == 0x3)
	    {
	        // load and set 1 bit
	        writeback_value = unmodified_value | (1 << b);
	    }
	    else
	    {
	        qemu_log_mask(LOG_GUEST_ERROR,
	                  "klbme_read: invalid decorated load of address: 0x%x\n", decorated_addr);
	    }

	    cpu_physical_memory_write(phys_addr, &writeback_value, read_size);

	    return (unmodified_value >> b) & 1;
    }
    
    return 0xDEADBEEF;
}

static void klbme_write(void *opaque, hwaddr offset,
                        uint64_t value, unsigned size)
{
    // this function is called whenever the user program writes to a decorated address 
    KLBMEState *s = (KLBMEState *)opaque;
    // in order to map just specific memory regions to the BME,
    // multiple BME instances are created with different base addresses / lengths.
    // Therefore, to get the full physical address, we have to know our base address.
    uint32_t bme_periph_base_addr = s->parent_obj.mmio[0].addr;
   	// physical address the instruction tried to write to, including decoration bits.
    uint32_t decorated_addr = offset + bme_periph_base_addr;

    uint8_t op_type = (decorated_addr & 0x1C000000) >> 26;
    uint32_t read_size = operand_size_from_alignment(decorated_addr);
    
    uint32_t phys_addr;
    uint32_t rmw_data;
    if (op_type & 0x4)
    {
    	// decorated store bit field insert
    	// creates a mask from b and w field and only updates the masked bits with those from `value`
    	uint32_t b = (decorated_addr & ( 0x03800000 + ((read_size-1) << 26) ))  >> 23;
        uint32_t w = (decorated_addr & ( 0x00380000 + ((read_size-1) << 22) ))  >> 19;
        uint32_t mask = ( (1 << (w+1)) - 1 ) << b;

        // offset is specified as 5 nibbles, less one bit
        //   and is relative to some base address indicated by bits 30:29
	    phys_addr = decorated_addr & 0x6007FFFF;
	    cpu_physical_memory_read(phys_addr, &rmw_data, read_size);

        rmw_data = (rmw_data & (~mask)) | (value & mask);
    }
    else
    {
    	// offset is specified as 5 nibbles and is relative to some base address indicated by bits 30:29
	    phys_addr = decorated_addr & 0x600FFFFF;
	    cpu_physical_memory_read(phys_addr, &rmw_data, read_size);
	    // op_type == 01 for bitwise-and, 10 for bitwise or, 11 for bitwise xor
	    if (op_type == 0x1)
	    {
	    	rmw_data &= value;
	    }
	    else if (op_type == 0x2)
	    {
	    	rmw_data |= value;
	    }
	    else if (op_type == 0x3)
	    {
	    	rmw_data ^= value;
	    }
	    else
	    {
	    	qemu_log_mask(LOG_GUEST_ERROR,
	                  "klbme_write: bit manipulation opcode 00 is illegal. Attempt to write 0x%x = 0x%x\n", decorated_addr, (unsigned)value);
	    }
	}

    cpu_physical_memory_write(phys_addr, &rmw_data, read_size);
}


static const MemoryRegionOps klbme_ops = {
    .read = klbme_read,
    .write = klbme_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_klbme = {
    .name = TYPE_KLBME,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

static void klbme_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    KLBMEState *s = KLBME(obj);

    memory_region_init_io(&s->iomem, OBJECT(s), &klbme_ops, s, TYPE_KLBME, 0x04000000);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void klbme_realize(DeviceState *dev, Error **errp)
{
    //KLBMEState *s = KLBME(dev);
}

static void klbme_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = klbme_realize;
    dc->vmsd = &vmstate_klbme;
}

static const TypeInfo klbme_arm_info = {
    .name          = TYPE_KLBME,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(KLBMEState),
    .instance_init = klbme_init,
    .class_init    = klbme_class_init,
};

static void klbme_register_types(void)
{
    type_register_static(&klbme_arm_info);
}

type_init(klbme_register_types)
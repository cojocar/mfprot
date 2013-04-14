#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/mman.h>
#include <string.h>

#include "rs232.h"

static volatile unsigned *bcm2835_gpio;
static void
bcm2835_peri_write(volatile uint32_t* paddr, uint32_t value)
{
	*paddr = value;
	*paddr = value;
}

static uint32_t
bcm2835_peri_read(volatile uint32_t* paddr)
{
	uint32_t ret = *paddr;
	uint32_t dummy = *paddr;
	return ret;
}

static void
bcm2835_peri_set_bits(volatile uint32_t* paddr, uint32_t value, uint32_t mask)
{
	uint32_t v = bcm2835_peri_read(paddr);
	v = (v & ~mask) | (value & mask);
	bcm2835_peri_write(paddr, v);
}

static void
bcm2835_gpio_fsel(uint8_t pin, uint8_t mode)
{
	// Function selects are 10 pins per 32 bit word, 3 bits per pin
#define BCM2835_GPFSEL0 0x0
	volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPFSEL0/4 + (pin/10);
	uint8_t   shift = (pin % 10) * 3;
#define BCM2835_GPIO_FSEL_MASK 0x3
	uint32_t  mask = BCM2835_GPIO_FSEL_MASK << shift;
	uint32_t  value = mode << shift;
	bcm2835_peri_set_bits(paddr, value, mask);
}

static uint8_t
bcm2835_gpio_lev(uint8_t pin)
{
#define BCM2835_GPLEV0 0x0034
#define HIGH 1
#define LOW 0
	volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPLEV0/4 + pin/32;
	uint8_t shift = pin % 32;
	uint32_t value = bcm2835_peri_read(paddr);
	return (value & (1 << shift)) ? HIGH : LOW;
}


static int
rs232_get_cts(void)
{
	return bcm2835_gpio_lev(23);
}

static int
rs232_get_rx(void)
{
	return bcm2835_gpio_lev(24);
}

int
rs232_config_gpio()
{
	int mem_fd;
	if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
		printf("can't open /dev/mem \n");
		return -1;
	}

#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
	bcm2835_gpio = mmap(
			NULL,             //Any adddress in our space will do
			4*1024,       //Map length
			PROT_READ|PROT_WRITE,// Enable reading & writting to mapped memory
			MAP_SHARED,       //Shared with other processes
			mem_fd,           //File to map
			GPIO_BASE         //Offset to GPIO peripheral
			);
	close(mem_fd);

	/* GPIO 23 as input for cts */
	bcm2835_gpio_fsel(23, 0);

	/* CPIO 24 as input for rx */
	bcm2835_gpio_fsel(24, 0);
}

int
rs232_send_byte(mfprot_device dev, char *buf, int len)
{
	int w;
	while (rs232_get_cts())
		usleep(2);
	//printf("CTS_0->%d\n", get_cts());
	if (len != (w = write(dev, buf, len))) {
		perror("write");
		return w;
	}
	return len;
}

int
rs232_recv_data(mfprot_device dev, void *ptr, int len)
{
	int i;
	for (i = 0; i < len; ++i) {
		rs232_recv_byte(dev, ((char *)ptr)+i);
	}
}

int
rs232_recv_byte(mfprot_device dev, uint8_t *byte_out)
{
	int j, k, t;
	int bits;
	uint16_t byte;
	struct timespec last;

	/* wait for start bit */
	while (1) {
		clock_gettime(CLOCK_MONOTONIC, &last);
		if (rs232_get_rx() == 0) {
			break;
		}
	}

	byte = 0;
	for (bits = 0; bits < 9; ++bits) {
		uint8_t b = 0;
#define TIMES_PER_BIT 8
		for (t = 0; t < TIMES_PER_BIT;) {
			struct timespec cur;
			uint8_t b_tmp = 0;
			unsigned long delay;

			clock_gettime(CLOCK_MONOTONIC, &cur);
			b_tmp = rs232_get_rx();

#define usec (1000ul)
#define msec (1000ul*usec)
#define sec (1000ul*msec)

			if (cur.tv_sec > last.tv_sec) {
				delay = cur.tv_nsec + (sec - last.tv_nsec);
			} else if (cur.tv_sec == last.tv_sec) {
				delay = cur.tv_nsec - last.tv_nsec;
			} else {
				printf("XXX\n");
			}
			if (delay >= (104/TIMES_PER_BIT)*((bits*TIMES_PER_BIT)+t)*usec) {
				b += b_tmp;
				++t;
				//printf(":%d", b_tmp);
			}
		}
		if (bits == 0 || bits == 9)
			continue;
		if (b >= TIMES_PER_BIT/2) {
			byte |= 1 << (bits-1);
			//printf("%d", 1);
		} else {
			//printf("%d", 0);
		}
	}

	*byte_out = byte;

	/* wait for stop bits */
	while (rs232_get_rx() == 0)
		;
}


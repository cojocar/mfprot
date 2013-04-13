#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/mman.h>
#include <time.h>
#include <string.h>

#include "libmfprot.h"


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
config_gpio()
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

static int
get_cts(void)
{
	return bcm2835_gpio_lev(23);
}

static int
get_rx(void)
{
	return bcm2835_gpio_lev(24);
}

mfprot_device
mfprot_get_device(const char *path)
{
	mfprot_device ret;
	struct termios t;

	config_gpio();
	ret = open(path, O_RDWR|O_NOCTTY);
	if (ret == -1) {
		perror("open");
		goto err_open;
	}

	if (tcgetattr(ret, &t) < 0) {
		perror("tcgetattr");
		goto err_tcgetattr;
	}

	t.c_cflag = B9600 | /*CRTSCTS |*/ CS8 | CLOCAL | CREAD;
	//t.c_lflag = ICANON;
	t.c_lflag = 0;

	t.c_iflag = IGNPAR | ICRNL;
	//t.c_iflag = IGNPAR | ICRNL /*| IXOFF | IXON */;
	//t.c_oflag = 0;
	t.c_oflag = 0;

	t.c_cc[VINTR]    = 0;     /* Ctrl-c */ 
	t.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
	t.c_cc[VERASE]   = 0;     /* del */
	t.c_cc[VKILL]    = 0;     /* @ */
	t.c_cc[VEOF]     = 0;     /* Ctrl-d */
	t.c_cc[VTIME]    = 0;     /* inter-character timer unused */
	t.c_cc[VMIN]     = 1;     /* blocking read until 1 character arrives */
	t.c_cc[VSWTC]    = 0;     /* '\0' */
	t.c_cc[VSTART]   = 0;     /* Ctrl-q */ 
	t.c_cc[VSTOP]    = 0;     /* Ctrl-s */
	t.c_cc[VSUSP]    = 0;     /* Ctrl-z */
	t.c_cc[VEOL]     = 0;     /* '\0' */
	t.c_cc[VREPRINT] = 0;     /* Ctrl-r */
	t.c_cc[VDISCARD] = 0;     /* Ctrl-u */
	t.c_cc[VWERASE]  = 0;     /* Ctrl-w */
	t.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
	t.c_cc[VEOL2]    = 0;     /* '\0'  */

	tcflush(ret, TCIFLUSH);
	if (tcsetattr(ret, TCSANOW, &t) < 0) {
		perror("tcsetattr");
		goto err_tcsetattr;
	}

	dprintf("new device id: %d", ret);
	return ret;
err_tcsetattr:
err_tcgetattr:
	close(ret);
err_open:
	return -1;
}

void
mfprot_display_status(FILE *f, uint8_t status)
{
	int i;
	uint8_t m;
	struct {
		uint8_t mask;
		char *msg;
	} status_flag[] = {
		{0x01, "EEPROM error"},
		{0x02, "Card OK"},
		{0x04, "RX OK"},
		{0x08, "RS232 Error"},
		{0x10, "MFtype, 0->1KB, 1->4KB"},
		{0x20, "ULtype, 0->MF standard, 1->MF Ultralight"},
		{0x40, "MFRC error"},
		{0x80, "One"},
	};

	if (!(status & S_ONE)) {
		fprintf(f, "%08x is not a flag status\n", status);
		return;
	}

	for (i = 0, m = S_ONE; m; m >>= 1, i++) {
		fprintf(f, "%1x: %s\n", (m == (status & m)), status_flag[7-i].msg);
	}
}

static int
send_data(mfprot_device dev, char *buf, int len)
{
	int w;
	while (get_cts())
		usleep(100000);
	printf("CTS_0->%d\n", get_cts());
	if (len != (w = write(dev, buf, len))) {
		perror("write");
		return w;
	}
	return len;
}

static int
recv_data(mfprot_device dev, char *buf, int len)
{
	int i;

	dprintf("receinving %d bytes\n", len);
	/* TODO: track local_bufer */
	for (i = 0; i < len; ++i) {
		if (read(dev, &buf[i], 1) < 0) {
			perror("read");
			return i;
		}
	}
	return len;
}

#define usec (1000ul)
#define msec (1000ul*usec)
#define sec (1000ul*msec)

static int
recv_data_alt(mfprot_device dev, void *buf, int len)
{
	int i, j, k, t;
	int bits;
	uint16_t byte;
	struct timespec last;
	char local_buf[1];

	/* wait for start bit */
	while (1) {
		clock_gettime(CLOCK_MONOTONIC, &last);
		if (get_rx() == 0) {
			break;
		}
	}

	for (i = 0; i < len; ++i) {
		byte = 0;
		for (bits = 0; bits < 9; ++bits) {
			uint8_t b = 0;
#define TIMES_PER_BIT 4
			for (t = 0; t < TIMES_PER_BIT;) {
				struct timespec cur;
				uint8_t b_tmp = 0;
				unsigned long delay;

				clock_gettime(CLOCK_MONOTONIC, &cur);
				b_tmp = get_rx();

				if (cur.tv_sec > last.tv_sec) {
					delay = cur.tv_nsec + (sec - last.tv_nsec);
				} else if (cur.tv_sec == last.tv_sec) {
					delay = cur.tv_nsec - last.tv_nsec;
				} else {
					printf("XXX\n");
				}
				if (delay >= (104/TIMES_PER_BIT)*((i*8*TIMES_PER_BIT)+(bits*TIMES_PER_BIT)+t)*usec) {
					b += b_tmp;
					++t;
					//printf(":%d", b_tmp);
				}
			}
			if (bits == 0)
				continue;
			if (b >= TIMES_PER_BIT/2) {
				byte |= 1 << (bits-1);
				//printf("%d", 1);
			} else {
				//printf("%d", 0);
			}
			//printf("%d", b);
		}
		//printf("%02x", byte);
		local_buf[i] = byte;
	}

	for (i = 0; i < len; ++i) {
		//printf("%02x", local_buf[i]);
	}
	//printf("\n");

	memcpy(buf, local_buf, len);

	/* wait for stop bits */
	while (get_rx() == 0)
		;
}

uint8_t
mfprot_get_status(mfprot_device dev)
{
	uint8_t flag;

	send_data(dev, "S", 1);
	tcdrain(dev);
	recv_data_alt(dev, &flag, 1);

	dprintf("got status: %02x", flag);

	return flag;
}

uint8_t
mfprot_get_uid(mfprot_device dev, uint8_t id[7])
{
	uint8_t flag;
	uint8_t buf[8];

	send_data(dev, "U", 1);
	tcdrain(dev);

	recv_data_alt(dev, &buf[0], 1);
	recv_data_alt(dev, &buf[1], 1);
	recv_data_alt(dev, &buf[2], 1);
	recv_data_alt(dev, &buf[3], 1);
	recv_data_alt(dev, &buf[4], 1);
	recv_data_alt(dev, &buf[5], 1);
	recv_data_alt(dev, &buf[6], 1);
	recv_data_alt(dev, &buf[7], 1);
	//recv_data_alt(dev, &buf[7], 1);

	flag = buf[0];

	dprintf("got status: %02x", flag);

	memcpy(id, &buf[1], 7);
	return flag;
}

void
mfprot_display_uid(FILE *f, uint8_t id[7])
{
	fprintf(f, "UID=%02x%02x%02x%02x"
			"%02x%02x%02x\n",
			id[0], id[1], id[2], id[3],
			id[4], id[5], id[6]);
}

void
mfprot_release_device(mfprot_device dev)
{
	dprintf("release device id: %d", dev);
	close(dev);
}

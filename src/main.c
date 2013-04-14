#include <stdio.h>
#include <ctype.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>

#include "libmfprot.h"
#include "rs232.h"

mfprot_device
mfprot_get_device(const char *path)
{
	mfprot_device ret;
	struct termios t;

	rs232_config_gpio();
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
mfprot_display_block(FILE *f, uint8_t data[16])
{
	int i, j;

	for (j = 0; j < 2; ++j) {
		for (i = 0; i < 8; ++i) {
			fprintf(f, "%02x ", data[8*j+i]);
		}
		for (i = 0; i < 8; ++i) {
			fprintf(f, "%c", isprint(data[8*j+i]) ? data[8*j+i] : '.');
		}
		fprintf(f, "\n");
	}
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

uint8_t
mfprot_get_status(mfprot_device dev)
{
	uint8_t flag;

	rs232_send_byte(dev, "S", 1);
	tcdrain(dev);
	rs232_recv_data(dev, &flag, 1);

	dprintf("got status: %02x", flag);

	return flag;
}

uint8_t
mfprot_read_card_block(mfprot_device dev,
		uint8_t block_addr, uint8_t key_type,
		uint8_t key_code, uint8_t data[16])
{
	uint8_t flag;
	uint8_t arg2 = (key_type << 7) | key_code;
	int i;

	rs232_send_byte(dev, "R", 1);
	rs232_send_byte(dev, &block_addr, 1);
	rs232_send_byte(dev, &arg2, 1);
	//tcdrain(dev);

	rs232_recv_byte(dev, &flag);

	if (flag & S_CARD_OK) {
		rs232_recv_data(dev, &data[0], 16);
	} else {
		memset(data, 0xff, 16);
	}

	dprintf("got status: %02x", flag);
	return flag;
}


uint8_t
mfprot_get_uid(mfprot_device dev, uint8_t id[7])
{
	uint8_t flag;
	uint8_t buf[8];

	rs232_send_byte(dev, "U", 1);
	tcdrain(dev);

	rs232_recv_data(dev, &buf[0], 8);

	flag = buf[0];

	dprintf("got status: %02x", flag);

	memcpy(id, &buf[1], 7);
	return flag;
}

uint8_t
mfprot_display_firmware_desc(mfprot_device dev, FILE *f)
{
	char c;
	rs232_send_byte(dev, "z", 1);

	do {
		rs232_recv_byte(dev, &c);
		fprintf(f, "%c", c);
	} while (c);
	fprintf(f, "\n");
}

uint8_t
mfprot_set_key(mfprot_device dev,
		uint8_t key_code, uint8_t key[6])
{
	int i;
	uint8_t flag;
	rs232_send_byte(dev, "K", 1);

	rs232_send_byte(dev, &key_code, 1);
	for (i = 0; i < 6; ++i) {
		rs232_send_byte(dev, &key[i], 1);
	}

	rs232_recv_byte(dev, &flag);
	dprintf("got status: %02x", flag);

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

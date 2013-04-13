#ifndef __libmfprot_h__
#define __libmfprot_h__
#include <stdint.h>

typedef int mfprot_device;

mfprot_device mfprot_get_device(const char *path);
void mfprot_release_device(mfprot_device);

uint8_t mfprot_get_status(mfprot_device dev);
void mfprot_display_status(FILE *f, uint8_t status);

uint8_t mfprot_get_uid(mfprot_device dev, uint8_t id[7]);
void mfprot_display_uid(FILE *f, uint8_t id[7]);

uint8_t mfprot_display_firmware_desc(mfprot_device dev, FILE *f);

uint8_t
mfprot_read_card_block(mfprot_device dev,
		uint8_t block_addr, uint8_t key_type,
		uint8_t key_code, uint8_t data[16]);

uint8_t
mfprot_set_key(mfprot_device dev,
		uint8_t key_code, uint8_t key[6]);
void
mfprot_display_block(FILE *f, uint8_t data[16]);

#if defined NDEBUG
#define dprintf(format, ...)
#else
#define dprintf(format, ...) \
	fprintf(stderr, "%s::%s:%d\t" format "\n",\
			__FILE__, __FUNCTION__,  __LINE__, __VA_ARGS__ )
#endif

enum mfprot_status_flag_mask {
	S_EEPROM_ERR = 0x01,
	S_CARD_OK    = 0x02,
	S_RX_OK      = 0x04,
	S_RS232_ERR  = 0x04,
	S_MFTYPE     = 0x10,
	S_ULTYPE     = 0x20,
	S_MFC_ERR    = 0x40,
	S_ONE        = 0x80,
};

#endif

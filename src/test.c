#include <stdio.h>

#include "libmfprot.h"

int
main(int argc, char *argv[])
{
	mfprot_device d = mfprot_get_device("/dev/ttyAMA0");
	uint8_t status;
	uint8_t id[7];

	status = mfprot_get_status(d);
	mfprot_display_status(stdout, status);

	mfprot_get_uid(d, id);
	mfprot_display_uid(stdout, id);

	mfprot_display_firmware_desc(d, stdout);

	mfprot_release_device(d);
}

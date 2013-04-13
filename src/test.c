#include <stdio.h>

#include "libmfprot.h"

int
main(int argc, char *argv[])
{
	mfprot_device d = mfprot_get_device("/dev/ttyAMA0");
	uint8_t status;

	status = mfprot_get_status(d);
	mfprot_display_status(stdout, status);

	mfprot_release_device(d);
}

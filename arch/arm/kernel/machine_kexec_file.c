// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * kexec_file for arm
 *
 * Copyright (C) 2021 Joel Stanley, IBM Corp.
 */

#include <linux/kexec.h>

struct zimage_header {
        uint32_t instr[9];
        uint32_t magic;
#define ZIMAGE_MAGIC cpu_to_le32(0x016f2818)
        uint32_t start;
        uint32_t end;
        uint32_t endian;

        /* Extension to the data passed to the boot agent.  The offset
         * points at a tagged table following a similar format to the
         * ATAGs.
         */
        uint32_t magic2;
#define ZIMAGE_MAGIC2 (0x45454545)
        uint32_t extension_tag_offset;
};

static int kexec_zimage_probe(const char *buf, unsigned long len)
{
	struct zimage_header *hdr;
	int size;

	if (len < sizeof(struct zimage_header)) {
		pr_err("File too short to be a zImage\n");
		return -ENOEXEC;
	}

	hdr = (struct zimage_header *)buf;


	pr_debug("zImage header: 0x%08x 0x%08x 0x%08x\n", hdr->magic, hdr->start, hdr->end);

	if (hdr->magic != ZIMAGE_MAGIC) {
		pr_err("Not a zImage\n");
		return -ENOEXEC;
	}

	size = le32_to_cpu(hdr->end) - le32_to_cpu(hdr->start);

	pr_debug("zImage size 0x%x, file size 0x%lx\n", size, len);

	if (size > len) {
		pr_debug("zImage is truncated - file 0x%lx vs header 0x%x\n", len, size);
		return -ENOEXEC;
	}

	pr_debug("Found a valid zImage\n");

	return 0;
}

const struct kexec_file_ops kexec_zimage_ops = {
	.probe = kexec_zimage_probe,
};

const struct kexec_file_ops * const kexec_file_loaders[] = {
	&kexec_zimage_ops,
};

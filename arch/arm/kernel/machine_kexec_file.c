// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * kexec_file for arm
 *
 * Copyright (C) 2021 Joel Stanley, IBM Corp.
 */

#define pr_fmt(fmt)     "kexec-file: " fmt

#include <linux/kexec.h>
#include <linux/printk.h>
#include <linux/of.h>
#include <linux/libfdt.h>

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

static void *kexec_zimage_load(struct kimage *image, char *kernel,
			       unsigned long kernel_len, char *initrd,
			       unsigned long initrd_len, char *cmdline,
			       unsigned long cmdline_len)
{
	struct zimage_header *hdr;
	void *dtb;
	struct kexec_buf kbuf = {  };
	struct kexec_buf dtb_kbuf = {  };
	int size;
	int ret;

	hdr = (struct zimage_header *)kernel;
	size = le32_to_cpu(hdr->end) - le32_to_cpu(hdr->start);

	kbuf.image = image;
	kbuf.buf_min = 0;
	kbuf.buf_max = ULONG_MAX;
	kbuf.top_down = false;

	kbuf.buffer = kernel;
	kbuf.bufsz = kernel_len;
	kbuf.mem = KEXEC_BUF_MEM_UNKNOWN;
	kbuf.memsz = size;
	kbuf.buf_align = SZ_2M;

	ret = kexec_add_buffer(&kbuf);
	if (ret)
		return ERR_PTR(ret);

	pr_debug("Loading kernel at 0x%lx bufsz=0x%lx memsz=0x%lx\n",
			kbuf.mem, kbuf.bufsz, kbuf.memsz);

	image->start = kbuf.mem;

	/* load dtb */
	/* TODO: initrd */
//	dtb = of_kexec_alloc_and_setup_fdt(image, initrd_load_addr, initrd_len, cmdline, 0); */
	dtb = of_kexec_alloc_and_setup_fdt(image, 0, 0, cmdline, 0);
	if (!dtb) {
		pr_err("Preparing for new dtb failed\n");
		return ERR_PTR(ret);
	}

	/* trim it */
	fdt_pack(dtb);
	dtb_kbuf.image = image;
	dtb_kbuf.buffer = dtb;
	dtb_kbuf.bufsz = fdt_totalsize(dtb);
	dtb_kbuf.mem = KEXEC_BUF_MEM_UNKNOWN;
	dtb_kbuf.memsz = fdt_totalsize(dtb);
	dtb_kbuf.buf_align = SZ_2M;
	dtb_kbuf.buf_max = ULONG_MAX;
	dtb_kbuf.top_down = true;

	ret = kexec_add_buffer(&dtb_kbuf);
	if (ret)
		return ERR_PTR(ret);
	/* TODO: do we use these, or ignore them? */
//	image->arch.dtb = dtb;
//	image->arch.dtb_mem = dtb_kbuf.mem;

	pr_debug("Loading dtb at 0x%lx bufsz=0x%lx memsz=0x%lx\n",
			dtb_kbuf.mem, dtb_kbuf.bufsz, dtb_kbuf.memsz);

	return NULL;
}

const struct kexec_file_ops kexec_zimage_ops = {
	.probe = kexec_zimage_probe,
	.load = kexec_zimage_load,
};

const struct kexec_file_ops * const kexec_file_loaders[] = {
	&kexec_zimage_ops,
};

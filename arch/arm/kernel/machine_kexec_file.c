// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * kexec_file for arm
 *
 * Copyright (C) 2021 Joel Stanley, IBM Corp.
 */

#include <linux/kexec.h>

const struct kexec_file_ops * const kexec_file_loaders[] = {
	NULL
};

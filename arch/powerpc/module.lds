SECTIONS {
	.tocbss (NOLOAD) : {
		. = ALIGN(8);
	}

	/* Make empty section for module_frob_arch_sections to expand. */
	.stubs (ax) (NOLOAD) : {
		. = ALIGN(8);
	}

	/* FIXME: This should depend on CONFIG_DYNAMIC_FTRACE. */
	.ftrace.tramp (ax) (NOLOAD) : {
		. = ALIGN(8);
	}
}

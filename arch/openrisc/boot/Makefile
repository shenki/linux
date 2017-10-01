#
# arch/openrisc2/boot/Makefile
#
# This file is subject to the terms and conditions of the GNU General Public
# License.  See the file "COPYING" in the main directory of this archive
# for more details.
#

OBJCOPYFLAGS_vmlinux.bin := -O binary

targets += vmlinux.bin vmlinux.gz vmImage

$(obj)/vmlinux.bin: vmlinux FORCE
	$(call if_changed,objcopy)

$(obj)/vmlinux.gz: $(obj)/vmlinux.bin FORCE
	$(call if_changed,gzip)

$(obj)/vmImage: $(obj)/vmlinux.gz
	$(call if_changed,uimage)
	@$(kecho) 'Kernel: $@ is ready'


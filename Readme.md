# U-Boot builds for the Pine64 PineNote

## Installation from running Linux on the PineNote

	dd if=uboot.img of=/dev/mmcblk0 seek=16384
	dd if=idblock.bin of=/dev/mmcblk0 seek=64


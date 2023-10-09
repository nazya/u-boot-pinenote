#define DEBUG

#include <common.h>
#include <dm.h>
#include <fdtdec.h>
#include <input.h>
#include <keyboard.h>
#include <key_matrix.h>
#include <stdio_dev.h>

struct osd_eink_priv {
	struct udevice *dev;
	struct udevice *touchscreen_dev;
};

static int osd_eink_probe(struct udevice *dev)
{
	struct osd_eink_priv *priv = dev_get_priv(dev);
	int ret;
	debug("osd_eink_probe start\n");

	// by requesting this device it will get probed
	ret = uclass_get_device_by_phandle(UCLASS_I2C_GENERIC, dev,
					   "touchscreen",
					   &priv->touchscreen_dev);
	if (ret) {
		dev_err(dev, "Cannot get touchscreen device: %d\n", ret);
		return ret;
	}
	debug("osd_eink_probe end\n");
	return 0;
}


static const struct udevice_id osd_eink_ids[] = {
	{ .compatible = "pinenote,osd" },
	{ }
};

U_BOOT_DRIVER(tegra_kbd) = {
	.name	= "osd_eink",
	.id	= UCLASS_KEYBOARD,
	.of_match = osd_eink_ids,
	.probe = osd_eink_probe,
	.priv_auto_alloc_size = sizeof(struct osd_eink_priv),
};

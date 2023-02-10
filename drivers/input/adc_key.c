/*
 * (C) Copyright 2017 Rockchip Electronics Co., Ltd
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <key.h>
#include <input.h>
#include <stdio_dev.h>

static struct input_config button_input;

static int adc_key_ofdata_to_platdata(struct udevice *dev)
{
	struct dm_key_uclass_platdata *uc_key;
	u32 chn[2], mV;
	int vref, ret;

	uc_key = dev_get_uclass_platdata(dev);
	if (!uc_key)
		return -ENXIO;

	uc_key->type = ADC_KEY;
	uc_key->name = dev_read_string(dev, "label");
	ret = dev_read_u32_array(dev_get_parent(dev),
				 "io-channels", chn, ARRAY_SIZE(chn));
	if (ret) {
		printf("%s: read 'io-channels' failed, ret=%d\n",
		       uc_key->name, ret);
		return -EINVAL;
	}

	vref = dev_read_u32_default(dev_get_parent(dev),
				    "keyup-threshold-microvolt", -ENODATA);
	if (vref < 0) {
		printf("%s: read 'keyup-threshold-microvolt' failed, ret=%d\n",
		       uc_key->name, vref);
		return -EINVAL;
	}

	uc_key->code = dev_read_u32_default(dev, "linux,code", -ENODATA);
	if (uc_key->code < 0) {
		printf("%s: read 'linux,code' failed\n", uc_key->name);
		return -EINVAL;
	}

	mV = dev_read_u32_default(dev, "press-threshold-microvolt", -ENODATA);
	if (mV < 0) {
		printf("%s: read 'press-threshold-microvolt' failed\n",
		       uc_key->name);
		return -EINVAL;
	}

	uc_key->channel = chn[1];
	uc_key->adcval = mV / (vref / 1024);

	return 0;
}

static int adc_key_getc(struct stdio_dev *dev){
	return input_getc(&button_input);
}

static int adc_key_tstc(struct stdio_dev *dev){
	return input_tstc(&button_input);
}


static int adc_keys_read_keys(struct input_config *input){
	int key = KEY_D;
	int key_send = KEY_DOWN;
	if (key_is_pressed(key_read(key))){
		input_send_keycodes(&button_input, &key_send, 1);
		return 1;
	}
	return 0;
}

static int adc_probe(struct udevice *dev1){
	printf("adc_probe\n");
	int error;
	struct stdio_dev dev = {
		.name	= "mag_key",
		.flags	= DEV_FLAGS_INPUT,
		.getc	= adc_key_getc,
		.tstc	= adc_key_tstc,
		.priv = dev1,
	};

	/* * @param repeat_delay_ms   Delay before key auto-repeat starts (in ms) */
	/* * @param repeat_rate_ms    Delay between successive key repeats (in ms) */
	input_set_delays(&button_input, 500, 500);
	struct dm_key_uclass_platdata *uc_key;
	uc_key = dev_get_uclass_platdata(dev1);
	if (!uc_key){
		printf("adc_probe: cannot get dev_get_uclass_platdata\n");
		return -1;
	}
	printf("uc_key: %s\n", uc_key->name);

	error = input_init(&button_input, 0);
	if (error) {
		debug("%s: ADC KEY Cannot set up input\n", __func__);
		return -1;
	}
	input_add_tables(&button_input, false);
	button_input.read_keys = adc_keys_read_keys;

	error = input_stdio_register(&dev);
	if (error)
		return error;
	input_set_delays(&button_input, 500, 500);

	return 0;
}


U_BOOT_DRIVER(adc_key) = {
	.name   = "adc_key",
	.id     = UCLASS_KEY,
	.probe = adc_probe,
	.ofdata_to_platdata = adc_key_ofdata_to_platdata,
};

/* Key Bus */
static int adc_key_bus_bind(struct udevice *dev)
{
	return key_bind_children(dev, "adc_key");
}

static const struct udevice_id adc_key_bus_match[] = {
	{ .compatible = "adc-keys" },
	{ },
};

U_BOOT_DRIVER(adc_key_bus) = {
	.name	   = "adc_key_bus",
	.id	   = UCLASS_SIMPLE_BUS,
	.of_match  = adc_key_bus_match,
	.bind	   = adc_key_bus_bind,
};

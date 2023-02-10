/*
 * (C) Copyright 2017 Rockchip Electronics Co., Ltd
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */
#include <common.h>
#include <dm.h>
#include <key.h>
#include <linux/input.h>
#include <power/rk8xx_pmic.h>
#include <irq-generic.h>

#include <input.h>
#include <stdio_dev.h>

static struct input_config button_input;

static void rk8xx_pwron_rise_handler(int irq, void *data)
{
	struct udevice *dev = data;
	struct dm_key_uclass_platdata *key;

	key = dev_get_uclass_platdata(dev);
	key->rise_ms = key_timer(0);

	debug("%s: %llu ms\n", __func__, key->rise_ms);
}

static void rk8xx_pwron_fall_handler(int irq, void *data)
{
	struct udevice *dev = data;
	struct dm_key_uclass_platdata *key;

	key = dev_get_uclass_platdata(dev);
	key->fall_ms = key_timer(0);

	debug("%s: %llu ms\n", __func__, key->fall_ms);
}

static int pwrkey_getc(struct stdio_dev *dev){
	return input_getc(&button_input);
}

static int pwrkey_tstc(struct stdio_dev *dev){
	return input_tstc(&button_input);
}

static int pwrkey_read_keys(struct input_config *input){
	int key = KEY_POWER;
	int key_send_short = KEY_DOWN;
	int key_send_long = KEY_ENTER;
	int event;
	event = key_read(key);
	if (event == KEY_PRESS_DOWN){
		input_send_keycodes(&button_input, &key_send_short, 1);
		return 1;
	} else if (event == KEY_PRESS_LONG_DOWN) {
		input_send_keycodes(&button_input, &key_send_long, 1);
		return 1;
	}

	return 0;
}

static int rk8xx_pwrkey_probe(struct udevice *dev)
{
	struct rk8xx_priv *rk8xx = dev_get_priv(dev->parent);
	struct dm_key_uclass_platdata *key = dev_get_uclass_platdata(dev);
	int fall_irq, rise_irq;

	if (!rk8xx->irq_chip) {
		printf("Failed to get parent irq chip\n");
		return -ENOENT;
	}

	fall_irq = virq_to_irq(rk8xx->irq_chip, RK8XX_IRQ_PWRON_FALL);
	if (fall_irq < 0) {
		printf("Failed to register pwron fall irq, ret=%d\n", fall_irq);
		return fall_irq;
	}

	rise_irq = virq_to_irq(rk8xx->irq_chip, RK8XX_IRQ_PWRON_RISE);
	if (rise_irq < 0) {
		printf("Failed to register pwron rise irq, ret=%d\n", rise_irq);
		return rise_irq;
	}

	key->name = "rk8xx_pwr";
	key->type = GPIO_KEY;
	key->code = KEY_POWER;
	key->skip_irq_init = 1;

	irq_install_handler(fall_irq, rk8xx_pwron_fall_handler, dev);
	irq_install_handler(rise_irq, rk8xx_pwron_rise_handler, dev);
	irq_handler_enable(fall_irq);
	irq_handler_enable(rise_irq);

	// connect to stdio
	int error;
	struct stdio_dev dev_stdin = {
		.name	= "pwr_key_stdin",
		.flags	= DEV_FLAGS_INPUT,
		/* .start	= novena_gpio_button_init, */
		.getc	= pwrkey_getc,
		.tstc	= pwrkey_tstc,
		.priv = dev,
	};

	/* * @param repeat_delay_ms   Delay before key auto-repeat starts (in ms) */
	/* * @param repeat_rate_ms    Delay between successive key repeats (in ms) */
	input_set_delays(&button_input, 500, 500);
	struct dm_key_uclass_platdata *uc_key;
	uc_key = dev_get_uclass_platdata(dev);
	if (!uc_key){
		printf("adc_probe: cannot get dev_get_uclass_platdata\n");
		return -1;
	}
	printf("uc_key: %s\n", uc_key->name);

	error = input_init(&button_input, 0);
	if (error) {
		debug("%s: PWR KEY Cannot set up input\n", __func__);
		return -1;
	}
	input_add_tables(&button_input, false);
	button_input.read_keys = pwrkey_read_keys;

	error = input_stdio_register(&dev_stdin);
	if (error)
		return error;
	input_set_delays(&button_input, 500, 500);

	return 0;
}

U_BOOT_DRIVER(rk8xx_pwrkey) = {
	.name   = "rk8xx_pwrkey",
	.id     = UCLASS_KEY,
	.probe  = rk8xx_pwrkey_probe,
};

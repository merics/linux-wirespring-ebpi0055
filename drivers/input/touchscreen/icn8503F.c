#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

struct icn8503f_tsc {
	struct device *dev;
	struct input_dev *input;
	struct i2c_client *client;
	int gpio_reset;
	int gpio_int;
	int irq;
	int max_x;
	int max_y;
	int revert_x;
	int revert_y;
	int flashPresence;
	int fwVersion;
};

static int icn8503f_i2c_read(struct i2c_client *client, u16 addr, void *buf, size_t len) {
    u8 reg[2];
    struct i2c_msg msg[2] = {
	{
		.addr = client->addr,
		.flags = 0,
		.len = 2,
		.buf = reg,
	},
	{
		.addr = client->addr,
		.flags = I2C_M_RD,
		.len = len,
		.buf = buf,
	}
    };

    reg[0] = (u8)(addr >> 8);
    reg[1] = (u8)(addr & 0xFF);
    if (i2c_transfer(client->adapter, msg, 2) != 2) {
	dev_err(&client->dev, "i2c transfer failed.\n");
	return -EIO;
    }
    return 0;
}

#define MAX_TX_LEN 128

static int icn8503f_i2c_write(struct i2c_client *client, u16 addr, void *buf, size_t len) {
    u8 tx_buf[MAX_TX_LEN];
    struct i2c_msg msg[1] = {
	{
		.addr = client->addr,
		.flags = 0,
		.len = len + 2,
		.buf = tx_buf,
	}
    };

    if (len > (MAX_TX_LEN - 2)) {
	dev_err(&client->dev, "data len (%d) exceeds limit (%d)\n", len, MAX_TX_LEN);
	return -EIO;
    }

    tx_buf[0] = (u8)(addr >> 8);
    tx_buf[1] = (u8)(addr & 0xFF);
    memcpy(&tx_buf[2], buf, len);
    if (i2c_transfer(client->adapter, msg, 1) != 1) {
	dev_err(&client->dev, "i2c transfer failed.\n");
	return -EIO;
    }
    return 0;
}

static int icn8503f_read_reg(struct i2c_client *client, u16 addr, u8 *value) {
    return icn8503f_i2c_read(client, addr, value, 1);
}

static int icn8503f_write_reg(struct i2c_client *client, u16 addr, u8 value) {
    return icn8503f_i2c_write(client, addr, &value, 1);
}

static int icn8503f_get_flash_fwversion(struct i2c_client *client) {
    int ret;
    u8 value[2];

    ret = icn8503f_i2c_read(client, 0x0c, value, 2);
    if (ret) {
	dev_err(&client->dev, "can't get flash firmware version from device\n");
	return 0;
    }
    return (value[0] << 8) | value[1];
}

static void icn8503f_reset(struct i2c_client *client, struct icn8503f_tsc *tsc) {
    gpio_direction_output(tsc->gpio_reset, 0);
    msleep(70);
    gpio_set_value(tsc->gpio_reset, 1);
    msleep(100);
}

static int icn8503f_check(struct i2c_client *client, struct icn8503f_tsc *tsc) {
    u8 value;
    int ret;
    int fwVersion;

    icn8503f_reset(client, tsc);
    ret = icn8503f_read_reg(client, 0x0a, &value);
    if (ret) {
	dev_err(&client->dev, "flash memory presence check failed\n");
	return ret;
    }
    tsc->flashPresence = value;
    if (value != 0x85) {
	dev_err(&client->dev, "this driver only supports chip with flash memory\n");
	return -ENODEV;
    }
    fwVersion = icn8503f_get_flash_fwversion(client);
    tsc->fwVersion = fwVersion;
    if (fwVersion != 0x3300) {
	dev_err(&client->dev, "this driver only supports fw version 0x3300. I got %d from device\n", fwVersion);
	return -ENODEV;
    }
    return 0;
}

#define BUF_SIZE (1 * 7 + 2)

static irqreturn_t icn8503f_soft_irq(int irq, void *handle)
{
    struct icn8503f_tsc *tsc = handle;
    struct input_dev *input = tsc->input;
    u8 buf[BUF_SIZE];
    u8 finger;
    u16 tx, ty;
    u8 pressure;
    u8 tflags;
    int err;

    err = icn8503f_i2c_read(tsc->client, 0x1000, buf, BUF_SIZE);
    if (err) {
	dev_err(&tsc->client->dev, "can't read touch data\n");
	return IRQ_HANDLED;
    }
    finger = buf[1];
    tx = (buf[4] << 8) + buf[3];
    ty = (buf[6] << 8) + buf[5];
    pressure = buf[7] ? 200 : 0;
    tflags = buf[8];

    if (tsc->revert_x) {
	tx = tsc->max_x - tx - 1;
    }

    if (tsc->revert_y) {
	ty = tsc->max_y - ty - 1;
    }

    input_report_abs(input, ABS_X, tx);
    input_report_abs(input, ABS_Y, ty);
    input_report_abs(input, ABS_PRESSURE, pressure);
    input_report_key(input, BTN_TOUCH, pressure ? 1 : 0);
    input_sync(input);

/*
    dev_info(&tsc->client->dev, "touch data: %0x(%d) %0x(%d) %0x(%d) %0x(%d) %0x(%d) %0x(%d) %0x(%d) %0x(%d) %0x(%d)\n",
	     buf[0], buf[0],
	     buf[1], buf[1],
	     buf[2], buf[2],
	     buf[3], buf[3],
	     buf[4], buf[4],
	     buf[5], buf[5],
	     buf[6], buf[6],
	     buf[7], buf[7],
	     buf[8], buf[8] );
*/
    return IRQ_HANDLED;
}

static int icn8503f_probe_dt(struct i2c_client *client, struct icn8503f_tsc *tsc) {
	struct device_node *np = client->dev.of_node;
	u32 val32;

	if (!np) {
		dev_err(&client->dev, "missing device tree data\n");
		return -EINVAL;
	}

	tsc->gpio_reset = of_get_named_gpio(np, "rst-gpio", 0);
	if (!gpio_is_valid(tsc->gpio_reset)) {
	    dev_err(&client->dev, "rst-gpio is not defined in DT, rc=%d\n", tsc->gpio_reset);
	    return -EINVAL;
	}
	if (devm_gpio_request(&client->dev, tsc->gpio_reset, "icn8503f_reset_pin")) {
	    dev_err(&client->dev, "can't request rest gpio\n");
	    return -EINVAL;
	}

	tsc->gpio_int = of_get_named_gpio(np, "int-gpio", 0);
	if (!gpio_is_valid(tsc->gpio_int)) {
	    dev_err(&client->dev, "int-gpio is not defined in DT, rc=%d\n", tsc->gpio_int);
	    return -EINVAL;
	}
	if (devm_gpio_request(&client->dev, tsc->gpio_int, "icn8503f_int_pin")) {
	    dev_err(&client->dev, "can't request int gpio\n");
	    return -EINVAL;
	}

	if (!of_property_read_u32(np, "max-x", &val32))
		tsc->max_x = val32;
	else
		tsc->max_x = 800;

	if (!of_property_read_u32(np, "max-y", &val32))
		tsc->max_y = val32;
	else
		tsc->max_y = 480;

	if (!of_property_read_u32(np, "revert-x", &val32))
		tsc->revert_x = val32;
	else
		tsc->revert_x = 0;
	if (!of_property_read_u32(np, "revert-y", &val32))
		tsc->revert_y = val32;
	else
		tsc->revert_y = 0;

	return 0;
}

static int icn8503f_probe(struct i2c_client *client, const struct i2c_device_id *id) {
	struct icn8503f_tsc *tsc;
	struct input_dev *input;
	int err;

	//dev_info(&client->dev, "probe start\n");

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c check functionality returned error!\n");
		return -EIO;
	}

	tsc = devm_kzalloc(&client->dev, sizeof(struct icn8503f_tsc), GFP_KERNEL);
	if (!tsc) {
		dev_err(&client->dev, "can't allocate memory.\n");
		return -ENOMEM;
	}

	err = icn8503f_probe_dt(client, tsc);
	if (err) {
		return err;
	}

	err = icn8503f_check(client, tsc);
	if (err) {
		return err;
	}
	dev_info(&client->dev, "flashPresence: 0x%x, fwVersion: 0x%x\n", tsc->flashPresence, tsc->fwVersion);

	tsc->client = client;
	tsc->irq = gpio_to_irq(tsc->gpio_int);

	err = devm_request_threaded_irq(&client->dev, tsc->irq,
					NULL, icn8503f_soft_irq,
					IRQF_ONESHOT,
					client->dev.driver->name, tsc);
	if (err) {
		dev_err(&client->dev, "Failed to request irq %d: %d\n",
			tsc->irq, err);
		return err;
	}
	
	input = devm_input_allocate_device(&client->dev);
	if (!input) {
		return -ENOMEM;
	}

	tsc->input = input;
	input->name = "icn8503f";
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

	__set_bit(EV_ABS, input->evbit);
	__set_bit(ABS_X, input->absbit);
	__set_bit(ABS_Y, input->absbit);
	__set_bit(EV_KEY, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);
	input_set_abs_params(input, ABS_X, 0, tsc->max_x, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, tsc->max_y, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_drvdata(input, tsc);

	err = input_register_device(input);
	if (err) {
		dev_err(&client->dev,
			"Failed to register input device: %d\n", err);
		return err;
	}

	return 0;
}

static const struct i2c_device_id icn8503f_idtable[] = {
	{ "icn8503f", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, icn8503f_idtable);

static const struct of_device_id icn8503f_of_match[] = {
  { .compatible = "fsl,icn8503f", },
  { }
};

MODULE_DEVICE_TABLE(of, icn8503f_of_match);

static struct i2c_driver icn8503f_driver = {
	.driver		= {
		.name		= "icn8503f",
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(icn8503f_of_match),
	},
	.id_table	= icn8503f_idtable,
	.probe		= icn8503f_probe,
};

module_i2c_driver(icn8503f_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("icn8503f TouchScreen Driver");
MODULE_AUTHOR("Meric Sentunali <merics@wirespring.net>");

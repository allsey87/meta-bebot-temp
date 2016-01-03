/*
 * Range finder Driver for the BeBot MID Case
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_device.h>

#include "../rf-core/rf.h"

#define MID_REG_RF_ENABLE 0x3e
#define MID_REG_RF_OFFSET 0x40
#define MID_REG_RF_BRIGHTNESS 0x20

struct bebot_chassis_rfs_devtype {
	int rf_count;
};

static const struct bebot_chassis_rfs_devtype bebot_chassis_rfs_rev1_devtype = {
	.rf_count = 6,
};

static const struct bebot_chassis_rfs_devtype bebot_chassis_rfs_rev2_devtype = {
	.rf_count = 12,
};


static struct of_device_id bebot_chassis_rfs_dt_ids[] = {
	{ .compatible = "upb,bebot-chassis-rfs", .data = &bebot_chassis_rfs_rev2_devtype, },
	{ }
};
MODULE_DEVICE_TABLE(of, bebot_chassis_rfs_dt_ids);

static const struct i2c_device_id bebot_chassis_rfs_i2c_id_table[] = {
	{ "bebot-chassis-rfs",	(kernel_ulong_t)&bebot_chassis_rfs_rev2_devtype, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bebot_chassis_rfs_i2c_id_table);

struct bebot_rf;

struct bebot_chassis_rfs {
	struct bebot_rf *rfs;
	struct i2c_client *client;
};

struct bebot_rf {
	int num;
	bool registered;
	char name[32];
	struct rf_classdev cdev;
	struct bebot_chassis_rfs *parent;
};

static int bebot_chassis_rfs_enable_set(struct rf_classdev *cdev, bool enabled) {
   return 0;
}

static int bebot_chassis_rfs_enable_get(struct rf_classdev *cdev, bool *enabled) {
   *enabled = true;
   return 0;
}

static int bebot_chassis_rfs_brightness_get(struct rf_classdev *cdev, unsigned int *val) {
   int bytes_read = 0;
   u8 buffer[2], reg;
   struct bebot_rf *rf;
     
   rf = container_of(cdev, struct bebot_rf, cdev);
     
   reg = MID_REG_RF_BRIGHTNESS + (2 * rf->num);  
   
   bytes_read = i2c_smbus_read_i2c_block_data(rf->parent->client, reg, 2, buffer);
	if(bytes_read != 2) {
		*val = -1;
	}
	else {
	   *val = le16_to_cpu((buffer[1] << 8) | buffer[0]);
	}
   return 0;
}

/* not implemented */
static int bebot_chassis_rfs_offset_get(struct rf_classdev *cdev, unsigned int *val) {
   *val = -1;
   return 0;
}

static int bebot_chassis_rfs_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct bebot_chassis_rfs_devtype *devtype;
	struct device_node *np = client->dev.of_node, *child;
	struct bebot_chassis_rfs *bc_rfs;
	struct bebot_rf *rfs;
	int res = 0, i;

	if(!client->dev.of_node) {
		dev_err(&client->dev, "could not find device tree node\n");
		return -ENODEV;
	} else {
		const struct of_device_id *of_id =
			of_match_device(bebot_chassis_rfs_dt_ids, &client->dev);
	
		devtype = (struct bebot_chassis_rfs_devtype *)of_id->data;

		dev_info(&client->dev, "Found %d range finders\n", devtype->rf_count);
	}

   /* allocate memory to the device */
	bc_rfs = devm_kzalloc(&client->dev, sizeof(*bc_rfs), GFP_KERNEL);
	if (!bc_rfs)
		return -ENOMEM;
   /* allocate memory for the individual range finders */
	rfs = devm_kzalloc(&client->dev, devtype->rf_count * sizeof(*rfs), GFP_KERNEL);
	if (!rfs)
		return -ENOMEM;

	bc_rfs->rfs = rfs;

   /* store a reference to the i2c client for future transactions */
   bc_rfs->client = client;

	for_each_child_of_node(np, child) {
		u32 reg;
		int res;
		const char* name;
		res = of_property_read_u32(child, "reg", &reg);
		if ((res != 0) || (reg >= devtype->rf_count))
			continue;
      /* set the parent device */
   	rfs[reg].parent = bc_rfs;
   	/* set the rf number */
   	rfs[reg].num = reg;
      /* parse the name */      
      name = of_get_property(child, "label", NULL) ? : child->name;
      snprintf(rfs[reg].name,
               sizeof(rfs[reg].name),
               "bebot:%s",
               name);      
      /* set up the cdev entry */
		rfs[reg].cdev.name = rfs[reg].name;
		rfs[reg].cdev.enabled_set = bebot_chassis_rfs_enable_set;
	   rfs[reg].cdev.enabled_get = bebot_chassis_rfs_enable_get;
	   rfs[reg].cdev.brightness_get = bebot_chassis_rfs_brightness_get;
	   rfs[reg].cdev.offset_get = bebot_chassis_rfs_offset_get;

	 	res = rf_classdev_register(&client->dev, &rfs[reg].cdev);
		if(res != 0) {
			goto exit;
		} else {
			rfs[reg].registered = true;
		}
	}
	
	/* Power !down all sensors by default to save power */
	i2c_smbus_write_word_data(client, MID_REG_RF_ENABLE, 0xffff);
	
	return 0;
exit:
	for (i = 0; i < devtype->rf_count; i++) {
		if(rfs[i].registered) {
			rf_classdev_unregister(&rfs[i].cdev);
			rfs[i].registered = false;
		}
	}

	return res;
}

static int bebot_chassis_rfs_remove(struct i2c_client *client)
{
	return 0;
}

static struct i2c_driver bebot_chassis_rfs_i2c_driver = {
	.driver = {
		.name 		= "bebot-chassis-rfs",
		.owner		= THIS_MODULE,
		.of_match_table = of_match_ptr(bebot_chassis_rfs_dt_ids),
	},
	.probe		= bebot_chassis_rfs_probe,
	.remove		= bebot_chassis_rfs_remove,
	.id_table	= bebot_chassis_rfs_i2c_id_table,
};

module_i2c_driver(bebot_chassis_rfs_i2c_driver);

MODULE_ALIAS("i2c:bebot-chassis-rfs");
MODULE_DESCRIPTION("Range finder driver for the BeBot MID Chassis");
MODULE_AUTHOR("Michael Allwright <allsey87@gmail.com>");
MODULE_LICENSE("GPL");

/* GRPPWM LED trigger for pca-9633 i2c led driver
 * 
 * Copyright (C) 2023 ChargePoint Inc.
 *
 * Author: Amitesh Singh
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/i2c-pca963x.h>


#define BIT_LDR3    6
#define BIT_LDR2    4
#define BIT_LDR1    2
#define BIT_LDR0    0

/**
 * LED output state
 */
#define REG_LEDOUT      0x08
/**
 * LED driver x is off
 */
#define LDR_STATE_OFF       0x00

/**
 * LED driver x is fully on (individual brightness and group dimming/ blinking
 * not controlled)
 */
#define LDR_STATE_ON        0x01

/**
 * LED driver x individual brightness can be controlled through its
 * PWMx register
 */
#define LDR_STATE_IND       0x02

/**
 * LED driver x individual brightness and group dimming/ blinking can be
 * controlled through its PWMx register and the GRPPWM registers. If using
 * LDR_STATE_IND_GRP the controller takes the minimum value of PWM* and
 * GRPPWM register
 */
#define LDR_STATE_IND_GRP   0x03


#define GROUP_CONTROL_MODE_BLINKING 0
#define GROUP_CONTROL_MODE_DIMMING 1

/**
 * Group duty cycle control
 */
#define REG_GRPPWM      0x06

/**
 * Group frequency
 */
#define REG_GRPFREQ     0x07

/**
 * LED output state
 */
#define REG_LEDOUT      0x08

/**
 * Mode register 2
 */
#define REG_MODE2       0x01
/**
 * Bit for group control; 0=dimming, 1=blinking
 */
#define BIT_DMBLNK  5


typedef struct {
     struct led_classdev *ldev;
     struct i2c_client *i2c_client;
     u8 grppwm;
     u8 ldrstate;
     u8 ldrstateall;
     u8 ldrbit;
     u8 grpctrlmode;
     u8 onoffratio_perc;
     u8 period;
} grppwm_trig_data;

static ssize_t gpwm_show(struct device *dev,
                         struct device_attribute *attr,
                         char *buf)
{
   struct led_classdev *ldev;
   grppwm_trig_data *grppwm_data;

   ldev = dev_get_drvdata(dev);
   grppwm_data = led_get_trigger_data(ldev);

   return sprintf(buf, "%d", grppwm_data->grppwm);
}

static ssize_t gpwm_store(struct device *dev,
                          struct device_attribute *att,
                          const char *buf, size_t count)
{
   struct led_classdev *ldev;
   grppwm_trig_data *grppwm_data;
   long int val = 0;

   ldev = dev_get_drvdata(dev);
   grppwm_data = led_get_trigger_data(ldev);

   if (!grppwm_data->i2c_client) {
        pr_err("i2c client is not present");
        return -ENODEV;
   }

   if (kstrtol(buf, 10, &val))
     return -ENOMEM;

   grppwm_data->grppwm = val;

   i2c_smbus_write_byte_data(grppwm_data->i2c_client, REG_GRPPWM, grppwm_data->grppwm);
   return count;
}

static ssize_t ldrstate_show(struct device *dev,
                             struct device_attribute *attr,
                             char *buf)
{
   struct led_classdev *ldev;
   grppwm_trig_data *grppwm_data;

   ldev = dev_get_drvdata(dev);
   grppwm_data = led_get_trigger_data(ldev);

   return sprintf(buf, "%d", grppwm_data->ldrstate);
}

static ssize_t ldrstateall_store(struct device *dev,
                              struct device_attribute *att,
                              const char *buf, size_t count)
{
   struct led_classdev *ldev;
   grppwm_trig_data *grppwm_data;
   long int val = 0;
   u8 reg;

   ldev = dev_get_drvdata(dev);
   grppwm_data = led_get_trigger_data(ldev);

   if (!grppwm_data->i2c_client) {
        pr_err("i2c client is not present");
        return -ENODEV;
   }
   if (kstrtol(buf, 10, &val))
     return -ENOMEM;

   grppwm_data->ldrstateall = val;

   //don't touch the wifi led
   reg = ( /* grppwm_data->ldrstateall << BIT_LDR3 | */
           grppwm_data->ldrstateall << BIT_LDR2 |
           grppwm_data->ldrstateall << BIT_LDR1 |
           grppwm_data->ldrstateall << BIT_LDR0 );

   i2c_smbus_write_byte_data(grppwm_data->i2c_client, REG_LEDOUT, reg);

   return count;
}

static ssize_t ldrstateall_show(struct device *dev,
                             struct device_attribute *attr,
                             char *buf)
{
   struct led_classdev *ldev;
   grppwm_trig_data *grppwm_data;

   ldev = dev_get_drvdata(dev);
   grppwm_data = led_get_trigger_data(ldev);

   return sprintf(buf, "%d", grppwm_data->ldrstateall);
}

static ssize_t ldrstate_store(struct device *dev,
                              struct device_attribute *att,
                              const char *buf, size_t count)
{
   struct led_classdev *ldev;
   grppwm_trig_data *grppwm_data;
   long int val = 0;

   ldev = dev_get_drvdata(dev);
   grppwm_data = led_get_trigger_data(ldev);

   if (!grppwm_data->i2c_client) {
        pr_err("i2c client is not present");
        return -ENODEV;
   }

   if (kstrtol(buf, 10, &val))
     return -ENOMEM;

   grppwm_data->ldrstate = val;

   return count;
}
static ssize_t ldrbit_show(struct device *dev,
                           struct device_attribute *attr,
                           char *buf)
{
   struct led_classdev *ldev;
   grppwm_trig_data *grppwm_data;

   ldev = dev_get_drvdata(dev);
   grppwm_data = led_get_trigger_data(ldev);

   return sprintf(buf, "%d", grppwm_data->ldrbit);
}

static ssize_t ldrbit_store(struct device *dev,
                            struct device_attribute *att,
                            const char *buf, size_t count)
{
   struct led_classdev *ldev;
   grppwm_trig_data *grppwm_data;
   long int val = 0;
   u8 prev_reg = 0;
   u8 reg = 0;

   ldev = dev_get_drvdata(dev);
   grppwm_data = led_get_trigger_data(ldev);

   if (!grppwm_data->i2c_client) {
        pr_err("i2c client is not present");
        return -ENODEV;
   }
   if (kstrtol(buf, 10, &val))
     return -ENOMEM;

   grppwm_data->ldrbit = val;

   prev_reg = i2c_smbus_read_byte_data(grppwm_data->i2c_client, REG_LEDOUT);

   reg = prev_reg & ~(0b11 << grppwm_data->ldrbit);
   reg |= (grppwm_data->ldrstate << grppwm_data->ldrbit);

   i2c_smbus_write_byte_data(grppwm_data->i2c_client, REG_LEDOUT, reg);

   return count;
}

static ssize_t grpctrlmode_show(struct device *dev,
                                struct device_attribute *attr,
                                char *buf)
{
   struct led_classdev *ldev;
   grppwm_trig_data *grppwm_data;

   ldev = dev_get_drvdata(dev);
   grppwm_data = led_get_trigger_data(ldev);

   return sprintf(buf, "%d", grppwm_data->grpctrlmode);
}

static ssize_t grpctrlmode_store(struct device *dev,
                                 struct device_attribute *att,
                                 const char *buf, size_t count)
{
   struct led_classdev *ldev;
   grppwm_trig_data *grppwm_data;
   long int val = 0;
   u8 prev_reg = 0;

   ldev = dev_get_drvdata(dev);
   grppwm_data = led_get_trigger_data(ldev);

   if (!grppwm_data->i2c_client) {
        pr_err("i2c client is not present");
        return -ENODEV;
   }
   if (kstrtol(buf, 10, &val))
     return -ENOMEM;

   grppwm_data->grpctrlmode = val;

   prev_reg = i2c_smbus_read_byte_data(grppwm_data->i2c_client, REG_MODE2);

   if (grppwm_data->grpctrlmode == GROUP_CONTROL_MODE_BLINKING) {
        i2c_smbus_write_byte_data(grppwm_data->i2c_client, REG_MODE2, prev_reg | (1 << BIT_DMBLNK));

   } else if (grppwm_data->grpctrlmode == GROUP_CONTROL_MODE_DIMMING) {
        i2c_smbus_write_byte_data(grppwm_data->i2c_client, REG_MODE2, prev_reg & ~(1 << BIT_DMBLNK));
   } else {
        pr_err("wrong group control mode option %ld is provided", val);
   }

   return count;
}

static ssize_t onoffratio_perc_show(struct device *dev,
                                    struct device_attribute *attr,
                                    char *buf)
{
   struct led_classdev *ldev;
   grppwm_trig_data *grppwm_data;

   ldev = dev_get_drvdata(dev);
   grppwm_data = led_get_trigger_data(ldev);

   return sprintf(buf, "%d", grppwm_data->onoffratio_perc);
}

static ssize_t onoffratio_perc_store(struct device *dev,
                                     struct device_attribute *att,
                                     const char *buf, size_t count)
{
   struct led_classdev *ldev;
   grppwm_trig_data *grppwm_data;
   long int val;

   ldev = dev_get_drvdata(dev);
   grppwm_data = led_get_trigger_data(ldev);

   if (!grppwm_data->i2c_client) {
        pr_err("i2c client is not present");
        return -ENODEV;
   }
   if (kstrtol(buf, 10, &val))
     return -ENOMEM;
   
   val = (val * 256)/100;

   if (val < 0)
      val = 0;
   else if (val > 255)
      val = 255;

   grppwm_data->onoffratio_perc = val;

   i2c_smbus_write_byte_data(grppwm_data->i2c_client, REG_GRPPWM, grppwm_data->onoffratio_perc);

   return count;
}

static ssize_t period_show(struct device *dev,
                           struct device_attribute *attr,
                           char *buf)
{
   struct led_classdev *ldev;
   grppwm_trig_data *grppwm_data;

   ldev = dev_get_drvdata(dev);
   grppwm_data = led_get_trigger_data(ldev);

   return sprintf(buf, "%d", grppwm_data->period);
}

static ssize_t period_store(struct device *dev,
                            struct device_attribute *att,
                            const char *buf, size_t count)
{
   struct led_classdev *ldev;
   grppwm_trig_data *grppwm_data;
   long int val;

   ldev = dev_get_drvdata(dev);
   grppwm_data = led_get_trigger_data(ldev);

   if (!grppwm_data->i2c_client) {
        pr_err("i2c client is not present");
        return -ENODEV;
   }
   if (kstrtol(buf, 10, &val))
     return -ENOMEM;

   grppwm_data->period = val;

   i2c_smbus_write_byte_data(grppwm_data->i2c_client, REG_GRPFREQ, grppwm_data->period);
   
   return count;
}

/*
 *  0 - 255
 */
static DEVICE_ATTR(gpwm, S_IRUGO | S_IWUSR, gpwm_show, gpwm_store);

/*
 *  LDR_STATE_OFF, LDR_STATE_ON, LDR_STATE_IND, LDR_STATE_IND_GRP
 */
static DEVICE_ATTR(ldrstate, S_IRUGO | S_IWUSR, ldrstate_show, ldrstate_store);

/*
 *  LDR_STATE_OFF, LDR_STATE_ON, LDR_STATE_IND, LDR_STATE_IND_GRP
 */
static DEVICE_ATTR(ldrstateall, S_IRUGO | S_IWUSR, ldrstateall_show, ldrstateall_store);

/*
 *  BIT_LDR0...3
 */
static DEVICE_ATTR(ldrbit, S_IRUGO | S_IWUSR, ldrbit_show, ldrbit_store);

/*
 * GROUP_CONTROL_MODE_BLINKING, GROUP_CONTROL_MODE_DIMMING
 */
static DEVICE_ATTR(grpctrlmode, S_IRUGO | S_IWUSR, grpctrlmode_show, grpctrlmode_store);

/*
 *  ontime/offtime * 100
 */
static DEVICE_ATTR(onoffratio_perc, S_IRUGO | S_IWUSR, onoffratio_perc_show, onoffratio_perc_store);

/*
 *  period cycles
 *  values 
 *   3      125ms          1/24 Hz * 3 cycles
 *   6      250ms          1/24 Hz * 6 cycles
 *  12      500ms          1/24 Hz * 12 cycles
 *  24      1s             1/24 Hz * 24 cycles
 *  255     Max (10.73s)   1/24 Hz * 255 cycles
 */
static DEVICE_ATTR(period, S_IRUGO | S_IWUSR, period_show, period_store);

static int grppwm_trig_activate(struct led_classdev *ldev)
{
   grppwm_trig_data *grppwm_data;

   grppwm_data = kzalloc(sizeof(*grppwm_data), GFP_KERNEL);
   if (!grppwm_data)
     return -ENOMEM;
   grppwm_data->ldev = ldev;

   grppwm_data->i2c_client = i2c_get_pca963x_device();

   if (!grppwm_data->i2c_client) {
        pr_err("unable to populate i2c client, check leds-pca963x driver");
        kfree(grppwm_data);
        return -ENODEV;
   }

   led_set_trigger_data(ldev, grppwm_data);

   device_create_file(ldev->dev, &dev_attr_gpwm);
   device_create_file(ldev->dev, &dev_attr_ldrstate);
   device_create_file(ldev->dev, &dev_attr_ldrstateall);
   device_create_file(ldev->dev, &dev_attr_ldrbit);
   device_create_file(ldev->dev, &dev_attr_grpctrlmode);
   device_create_file(ldev->dev, &dev_attr_onoffratio_perc);
   device_create_file(ldev->dev, &dev_attr_period);

   return 0;
}

static void grppwm_trig_deactivate(struct led_classdev *ldev)
{
   grppwm_trig_data *grppwm_data;

   grppwm_data = led_get_trigger_data(ldev);

   device_remove_file(ldev->dev, &dev_attr_gpwm);
   device_remove_file(ldev->dev, &dev_attr_ldrstate);
   device_remove_file(ldev->dev, &dev_attr_ldrstateall);
   device_remove_file(ldev->dev, &dev_attr_ldrbit);
   device_remove_file(ldev->dev, &dev_attr_grpctrlmode);
   device_remove_file(ldev->dev, &dev_attr_onoffratio_perc);
   device_remove_file(ldev->dev, &dev_attr_period);

   kfree(grppwm_data);
}

static struct led_trigger grppwm_trig = {
     .name = "grppwm",
     .activate = grppwm_trig_activate,
     .deactivate = grppwm_trig_deactivate
};

static int __init grppwm_init(void)
{
   return led_trigger_register(&grppwm_trig);
}

static void __exit grppwm_exit(void)
{
   led_trigger_unregister(&grppwm_trig);
}

module_init(grppwm_init);
module_exit(grppwm_exit);

MODULE_AUTHOR("Amitesh Singh");
MODULE_DESCRIPTION("implement grppwm on pca9633 i2c");
MODULE_LICENSE("GPL v2");


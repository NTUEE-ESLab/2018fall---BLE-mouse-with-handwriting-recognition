#include <linux/fs.h>
#include <linux/module.h>
#include <asm/uaccess.h>
#include <linux/pci.h>
#include <linux/input.h>
#include <linux/platform_device.h>


#define subs_verticle_cursor_event   1
#define subs_horizon_cursor_event  2
#define subs_mouse_button_event   4



#define mouse_button_left_click              1
#define mouse_button_roll_push             2
#define mouse_button_right_click           3
#define mouse_button_left_push             4
#define mouse_button_right_push           5
#define mouse_button_left_end_push      6
#define mouse_button_roll_end_push      7
#define mouse_button_right_end_push   8
#define mouse_button_roll_up                  9
#define mouse_button_roll_down             10
#define keyboard_char_push                     11


struct input_dev *virmouse_input_dev;
static struct platform_device *virmouse_dev; 


static ssize_t write_virmouse(struct device *dev,
                              struct device_attribute *attr,
                              const char *buffer, size_t count)
{
        int event = *((int*)buffer);
        int value = *(((int*)buffer) + 1);

        if (event == subs_horizon_cursor_event ) input_report_rel(virmouse_input_dev, REL_X, value);
        else if (event == subs_verticle_cursor_event ) 
        {
            input_report_rel(virmouse_input_dev, REL_Y, value);
            if (count == (sizeof(int) << 2)) input_report_rel(virmouse_input_dev, REL_X, *(((int*)buffer) + 3));
        }
        else if (event == subs_mouse_button_event)
        {
            if (value > 0) 
            {
                switch (value)
                {
                    case mouse_button_left_click:
                        input_report_key(virmouse_input_dev, BTN_LEFT, 1);
                        input_report_key(virmouse_input_dev, BTN_LEFT, 0);
                        break;
                    case mouse_button_right_click:
                        input_report_key(virmouse_input_dev, BTN_RIGHT, 1);
                        input_report_key(virmouse_input_dev, BTN_RIGHT, 0);
                        break;
                    case mouse_button_roll_push:
                        input_report_rel(virmouse_input_dev, REL_WHEEL, 1);
                        break;
                    case mouse_button_roll_up:
                        input_report_rel(virmouse_input_dev, REL_WHEEL, 1);
                        break;
                    case mouse_button_roll_down:
                        input_report_rel(virmouse_input_dev, REL_WHEEL, -1);
                        break;
                    case mouse_button_left_push:
                        input_report_key(virmouse_input_dev, BTN_LEFT, 1);
                        break;
                    case mouse_button_right_push:
                        input_report_key(virmouse_input_dev, BTN_RIGHT, 1);
                        break;
                    case mouse_button_left_end_push:
                        input_report_key(virmouse_input_dev, BTN_LEFT, 0);
                        break;
                    case mouse_button_right_end_push:
                        input_report_key(virmouse_input_dev, BTN_RIGHT, 0);
                        break;
                }                        
            }
        }
        else if (event == keyboard_char_push)
        {
            int little = 0;
            if (value < 0) 
            {
                little = 1;
                value = -value;
            }
            if (little) input_report_key(virmouse_input_dev, KEY_RIGHTSHIFT, 1);
            input_report_key(virmouse_input_dev, value, 1);
            input_report_key(virmouse_input_dev, value, 0);
            if (little) input_report_key(virmouse_input_dev, KEY_RIGHTSHIFT, 0);
        }

        input_sync(virmouse_input_dev);

        return count;

}


DEVICE_ATTR(vmevent, 0644, NULL, write_virmouse);


static struct attribute *virmouse_attrs[] = {
        &dev_attr_vmevent.attr,
        NULL
};


static struct attribute_group virmouse_attr_group = {
        .attrs = virmouse_attrs,
};


int __init virmouse_init(void)
{
        
        virmouse_dev = platform_device_register_simple("virmouse", -1, NULL, 0);
        if (IS_ERR(virmouse_dev))
		{
                printk ("virmouse_init: error\n");
                return PTR_ERR(virmouse_dev);
        }

        
        sysfs_create_group(&virmouse_dev->dev.kobj, &virmouse_attr_group);

        
        virmouse_input_dev = input_allocate_device();
        if (!virmouse_input_dev) 
		{
                printk("Bad input_allocate_device()\n");
                return -ENOMEM;
        }

        
        set_bit(EV_REL, virmouse_input_dev->evbit);
        set_bit(REL_X, virmouse_input_dev->relbit);
        set_bit(REL_Y, virmouse_input_dev->relbit);
        set_bit(REL_WHEEL, virmouse_input_dev->relbit);


        
        
         virmouse_input_dev->name = "Virtual Mouse";
         virmouse_input_dev->phys = "vmd/input0"; 
         virmouse_input_dev->id.bustype = BUS_VIRTUAL;
         virmouse_input_dev->id.vendor  = 0x0000;
         virmouse_input_dev->id.product = 0x0000;
         virmouse_input_dev->id.version = 0x0000;

         virmouse_input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REL);
         virmouse_input_dev->keybit[BIT_WORD(BTN_MOUSE)] = BIT_MASK(BTN_LEFT) | BIT_MASK(BTN_RIGHT) | BIT_MASK(BTN_MIDDLE);
         virmouse_input_dev->relbit[0] = BIT_MASK(REL_X) | BIT_MASK(REL_Y);
         virmouse_input_dev->keybit[BIT_WORD(BTN_MOUSE)] |= BIT_MASK(BTN_SIDE) | BIT_MASK(BTN_EXTRA) ;
         virmouse_input_dev->relbit[0] |= BIT_MASK(REL_WHEEL);

         set_bit(KEY_Q, virmouse_input_dev->keybit);
         set_bit(KEY_W, virmouse_input_dev->keybit);
         set_bit(KEY_E, virmouse_input_dev->keybit);
         set_bit(KEY_R, virmouse_input_dev->keybit);
         set_bit(KEY_T, virmouse_input_dev->keybit);
         set_bit(KEY_Y, virmouse_input_dev->keybit);
         set_bit(KEY_U, virmouse_input_dev->keybit);
         set_bit(KEY_I, virmouse_input_dev->keybit);
         set_bit(KEY_O, virmouse_input_dev->keybit);
         set_bit(KEY_P, virmouse_input_dev->keybit);
         set_bit(KEY_A, virmouse_input_dev->keybit);
         set_bit(KEY_S, virmouse_input_dev->keybit);
         set_bit(KEY_D, virmouse_input_dev->keybit);
         set_bit(KEY_F, virmouse_input_dev->keybit);
         set_bit(KEY_G, virmouse_input_dev->keybit);
         set_bit(KEY_H, virmouse_input_dev->keybit);
         set_bit(KEY_J, virmouse_input_dev->keybit);
         set_bit(KEY_K, virmouse_input_dev->keybit);
         set_bit(KEY_L, virmouse_input_dev->keybit);
         set_bit(KEY_Z, virmouse_input_dev->keybit);
         set_bit(KEY_X, virmouse_input_dev->keybit);
         set_bit(KEY_C, virmouse_input_dev->keybit);
         set_bit(KEY_V, virmouse_input_dev->keybit);
         set_bit(KEY_B, virmouse_input_dev->keybit);
         set_bit(KEY_N, virmouse_input_dev->keybit);
         set_bit(KEY_M, virmouse_input_dev->keybit);
         set_bit(KEY_RIGHTSHIFT, virmouse_input_dev->keybit);
        
        
        
        input_register_device(virmouse_input_dev);

        
        return 0;
}


void virmouse_uninit(void)
{
        
        input_unregister_device(virmouse_input_dev);
        sysfs_remove_group(&virmouse_dev->dev.kobj, &virmouse_attr_group);
        platform_device_unregister(virmouse_dev);
        return;
}

module_init(virmouse_init);
module_exit(virmouse_uninit);

MODULE_AUTHOR("EMBBEDED LAB");
MODULE_DESCRIPTION("Virtual Mouse Driver");
MODULE_LICENSE("GPL");

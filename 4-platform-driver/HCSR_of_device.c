/*
 * A sample program to show the binding of platform driver and device.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include "HCSR_of_device.h"
#include <linux/kthread.h>


static struct P_chip P1_chip = {
		.name	= "HCSR_0",
		.dev_no 	= 20,
		.plf_dev = {
			.name	= "HCSR_0",
			.id	= -1,
		}
};

static struct P_chip P2_chip = {
		.name	= "HCSR_1",
		.dev_no 	= 55,
		.plf_dev = {
			.name	= "HCSR_1",
			.id	= -1,
		}
};


/**
 * register the device when module is initiated
 */

static int p_device_init(void)
{
	int ret = 0;
	
	
	printk("changes\n");
	/* Register the device */
	platform_device_register(&P1_chip.plf_dev);
	
	printk(KERN_ALERT "Platform device 1 is registered in init \n");

	platform_device_register(&P2_chip.plf_dev);

	printk(KERN_ALERT "Platform device 2 is registered in init \n");
	
	return ret;
}

static void p_device_exit(void)
{
    	platform_device_unregister(&P1_chip.plf_dev);

	platform_device_unregister(&P2_chip.plf_dev);
	//kfree(&P1_chip);
	//kfree(&P2_chip);
	printk(KERN_ALERT "Goodbye, unregister the device\n");
}

module_init(p_device_init);
module_exit(p_device_exit);
MODULE_LICENSE("GPL");

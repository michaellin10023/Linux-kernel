/* 
Assignment 3 A SPI-based 1-wire Device Driver for LED Strip
this is an spi device program to register WS2812 led ring.
by team 20: Kausic (1213203730), beibei (1210467866)
*/


#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/spi/spi.h>


static struct spi_device *ringLed_spi_device;

static struct spi_board_info ring_info = 
{
	.modalias ="WS2812" ,
	.bus_num = 1,
	.chip_select = 1,
	.mode=SPI_MODE_0,
	.max_speed_hz=6666666,  // a clock cycle: 0.15us
};

// initialize the spi device (this function is called in "insmod ring_device.ko")
static int initialize(void)
{	
	int ret;
	struct spi_master *master;

	master = spi_busnum_to_master( ring_info.bus_num );
	if( !master )
		{printk("error assigining bus number to master \n");return -ENODEV;}

	ringLed_spi_device = spi_new_device( master, &ring_info );

	if( !ringLed_spi_device )
		{printk("error adding new Spi device\n");return -ENODEV;}
	ringLed_spi_device->bits_per_word = 16;

	ret = spi_setup( ringLed_spi_device );
	if( ret )
	{
		printk("SPI_setup failed\n");
		return -1;
	}
	return 0;
}

// unregister the spi device (this function is called in "rmmod ring_device.ko")
static void cleanup(void)
{
	spi_unregister_device(ringLed_spi_device);
}

module_init(initialize);
module_exit(cleanup);
MODULE_LICENSE("GPL");

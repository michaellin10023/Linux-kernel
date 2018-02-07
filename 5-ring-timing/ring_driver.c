/* 
Assignment 3 A SPI-based 1-wire Device Driver for LED Strip
Part 1: this is an spi driver program to control WS2812 led ring.
by team 20: Kausic (1213203730), beibei (1210467866)
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/moduleparam.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/delay.h>
//#include<linux/kdev_t.h>


//--------------------------DECLARATIONS AND DEFINITIONS---------------------------------

#define DEVICE_NAME "WS2812" 	
#define DEVICES 1
#define DEVICE_CLASS "WS2812"

static dev_t device_no;
int mux1=44;  //gpio numbers 
int mux2=72;
int ls = 24;

struct Info 
{
	int start_led;
	int val;
	int color;
};

static struct cdev cdev_info;
static struct class *ringLed_class;
static unsigned char data[16][24];    //pixels after encoding

static struct spi_device *spi_dev;

//*********** function declarations *****************
static int __init ringLed_spi_init(void);   
static void __exit ringLed_spi_exit(void);

static int ringLed_open(struct inode *, struct file *);
static int ringLed_release(struct inode *, struct file *);
static long ringLed_ioctl(struct file *, unsigned int ,unsigned long);
static ssize_t ringLed_write(struct file *f, const char *t, size_t i, loff_t *lo);
static int initialize_ringLed_Driver(void);

static struct file_operations fops = { .owner = THIS_MODULE, 
				   	.open = ringLed_open,
				   	.write = ringLed_write,
				   	.release = ringLed_release, 
				   	.unlocked_ioctl = ringLed_ioctl,
				     };

static unsigned char t[24];  

//******************************************************

uint8_t Bit1_encode(void)
{
	return 0xF0;    //"0000 0000 1111 0000" for data bit "1"
}

uint8_t Bit0_encode(void)
{
	return 0xC0;    //"0000 0000 1100 0000" for data bit "0"
}

//reset: to reset SPI to a suitable mode
void reset(struct spi_device *spi)
{	
	int i;
	for(i=0;i<24;i++)
		t[i]=0;
	spi_write(spi,t,sizeof(t));
	udelay(55);
}

//encode: encode the (r,g,b) decimal values to bit string
void encode(int g,int r, int b)
{
	
	int i;
	int val=8;
	for (i=0;i<val;i++)
		t[i]=((g>>i)& 0x01 )? Bit1_encode() : Bit0_encode();  //green
	
	for (i=val;i<2*val;i++)
		t[i]=((r>>(i-val))& 0x01 ) ? Bit1_encode() : Bit0_encode(); //red

	for (i=2*val;i<3*val;i++)	
		t[i]=((b>>(i-2*val))& 0x01 ) ? Bit1_encode(): Bit0_encode(); //blue
	
}

//off: to turn off the light. Write data bit "0" to all 									
void off(struct spi_device *spi)
{	
	int i,j;
	encode(0,0,0);
	for (i=0;i<16;i++)  //for 16 leds
	{
		for (j=0;j<24;j++)  //for 24 bits per led 
			data[i][j]=t[j];
	}
	spi_write(spi,data,sizeof(data));
	//reset(spi);
}  

//set_Leds: accept the requirement for different color intensity for different leds
//          call "encode()" function to encode (r,g,b) value into bit string
void set_Leds(int led_no,int color,int intensity)
{	int r=0,g=0,b=0,i;
	//led_no--;
	if(color==0)
		g=intensity;
	if(color==1)
		r=intensity;
	if(color==2)
		b=intensity;
	encode(g,r,b);
	for(i=0;i<24;i++)
		data[led_no][i]=t[i];

}

//initialize_Led: set all leds zero as initialization
void initialize_Leds(void)
{
	int i,j;
	encode(0,0,0);
	for (i=0;i<16;i++)
	{
		for (j=0;j<24;j++)
			data[i][j]=t[j];
	}
}

//once the SPI device and driver are matched, the "probe()" function is called
//probe:  call the function to initialize the device driver
static int ringLed_probe(struct spi_device *spi)
{   
	int ret;
	printk("SPI device and driver matched\n");
	spi_dev = spi;
	ret = initialize_ringLed_Driver();
	if(ret<0)
		{printk("Error in intiailizing \n"); return -1;}

	initialize_Leds();
	off(spi_dev);
	return 0;
}

//remove: free the resources (gpios)
static int  ringLed_remove(struct spi_device *spi)
{
	printk("ringLed  spi removed successfully \n");
	gpio_free(mux1);
	gpio_free(mux2);
	gpio_free(ls);
	return 0;
}

//spi device id table:to match the spi device and driver
static const struct spi_device_id device_table[] = {
    { "WS2812", 0 },
    { }
};

//spi driver data structure
static struct spi_driver ringLed_spi_driver = {
	.driver = {
		.name =		"WS2812",
		.owner =	THIS_MODULE,
	},
	.probe =	ringLed_probe,
	.remove =	ringLed_remove,
	.id_table= device_table,

};

// initialize the device driver
static int initialize_ringLed_Driver(void)
{
	int result=1;
	printk("Initializing\n");
	//-------- registration --------------:
	if (alloc_chrdev_region(&device_no, 0, 1, DEVICE_NAME) < 0) 
		{printk(KERN_DEBUG "Can't register device\n"); return -1;}
	//-------- class_creation ------------:
	if((ringLed_class=class_create(THIS_MODULE,DEVICE_CLASS))==NULL)
		{printk("Error in Creating class\n");return -1;}
	cdev_init(&cdev_info, &fops);
	cdev_info.owner = THIS_MODULE;
	// Connect the major/minor number to the cdev 
	result = cdev_add(&cdev_info, (device_no), 1);

	if (result) 
	{ printk("Bad cdev\n");return result; }
	//--------- device_creation --------------:
	if(device_create(ringLed_class,NULL,device_no,NULL,DEVICE_NAME)==NULL)
		{printk("Error in Device creation"); return -1;}
	return 0;
}

//gpio_init: io multiplexing
static int gpio_init (void)
{
	if(gpio_request(mux1,"mux1")!=0) { printk("Error requesting mux1\n");return -1;}
	if(gpio_request(mux2,"mux2")!=0) { printk("Error requesting mux2\n");return -1;}
	if(gpio_request(ls,"level_shifter")!=0) { printk("Error requesting Level shifter\n");return -1;}
	if(gpio_direction_output(mux1,1)!=0) {printk("Error in GPIO direction mux1"); return -1;}
	gpio_direction_output(mux2,0);
	if(gpio_direction_output(ls,0)!=0) {printk("Error in GPIO direction level_shifter"); return -1;}
	gpio_set_value_cansleep(mux1,1);
	gpio_set_value_cansleep(mux2,0);
	gpio_set_value_cansleep(ls,0);
	return 0;
}

// spi driver initialization: this function is called in "insmod ring_driver.ko".
// register the spi driver
static int __init ringLed_spi_init(void)
{
	int ret=0;
	if(ret<0)
		{printk("Error in gpio initialization\n");return -1;}
	ret = spi_register_driver(&ringLed_spi_driver);
	if (ret < 0) 
		{printk("spi registering failed \n"); return -1;}
	printk("Done\n");
	return 0;
}

// spi driver exit: this function is called in "rmmod ring_driver.ko".
// unregister the spi driver, char device, destroy the device and class
static void __exit ringLed_spi_exit(void)
{
	//unregister_chrdev(MAJOR_NUMBER, "spiLed");

	spi_unregister_driver(&ringLed_spi_driver);
	unregister_chrdev_region(device_no,DEVICES);
	cdev_del(&cdev_info);
	device_destroy(ringLed_class, MKDEV(MAJOR(device_no), 0));
	class_destroy(ringLed_class);
	
	printk(KERN_ALERT "exited\n");
	return;
}

//************************************************************************
//************* open, release, ioctl, write functions ******************** 

static int ringLed_open(struct inode *node, struct file *filp) {
	printk(KERN_ALERT "The driver has been opened\n");
	
	return 0;
}

static int ringLed_release(struct inode *node, struct file *filp) {
	printk(KERN_ALERT "The driver has been released\n");
	return 0;
}

// ioctl: get the user command ("RESET") from user space,
// 	  reset the device WS2812 and configure GPIOs
static long ringLed_ioctl(struct file *fp,unsigned int i,unsigned long j)
{	
	//static char  cmd[20];
	unsigned int cmd;
	int ret;

	copy_from_user(&cmd,&i,sizeof(cmd));
	if(cmd==0)
	{	
		printk("Reseting WS2812 and configuring GPIO ");
		ret=gpio_init();
		if(ret<0)
			{printk("Error in gpio initialization\n");return -1;}
		reset(spi_dev);
	}
	else
		return -1;
	return 0;
}

// write: get the user info (pixel) from user space,
//        set the leds according to pixel requirements
static ssize_t ringLed_write(struct file *f, const char *t, size_t i, loff_t *lo)
{
	struct Info *info_buff = (struct Info *) kmalloc(sizeof(struct Info),GFP_KERNEL);
	copy_from_user(info_buff,t,sizeof(struct Info));
	set_Leds(info_buff->start_led,info_buff->color,info_buff->val);
	spi_write(spi_dev,data,sizeof(data));
	return 0;
}

//**********************************************
module_init( ringLed_spi_init);
module_exit( ringLed_spi_exit);
MODULE_LICENSE("GPL");

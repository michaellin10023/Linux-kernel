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
#include <linux/ktime.h>
#include <linux/pci.h>
#include <asm/div64.h>
//#include<linux/kdev_t.h>
//--------------------------DECLARATIONS AND DEFINITIONS---------------------------------
#define DEVICE_NAME "WS2812_2" 	
#define DEVICES 1
#define DEVICE_CLASS "WS2812_2"
#define GIP_GPIO_BAR		1
static dev_t device_no;
#define PORTA_DATA			0x00	
int mux1=45;
int ls = 28;
int pin =12;

struct Info 
{
	int start_led;
	int val;
	int color;
};

/**********************************************************************RDTSC ************/
#if defined(__i386__)
static __inline__ unsigned long long rdtsc(void)   // This has been obtained from an outside source
{					 // this is assembly code to obtain the Time stamp counter value
					// based on the platform.
  unsigned long long int x;
     __asm__ volatile (".byte 0x0f, 0x31" : "=A" (x));
     return x;
}
#elif defined(__x86_64__)

static __inline__ unsigned long long rdtsc(void)
{
  unsigned hi, lo;
  __asm__ __volatile__ ("rdtsc" : "=a"(lo), "=d"(hi));
  return ( (unsigned long long)lo)|( ((unsigned long long)hi)<<32 );
}

#elif defined(__powerpc__)

static __inline__ unsigned long long rdtsc(void)
{
  unsigned long long int result=0;
  unsigned long int upper, lower,tmp;
  __asm__ volatile(
	        "0:                  \n"
	        "\tmftbu   %0           \n"
	        "\tmftb    %1           \n"
	        "\tmftbu   %2           \n"
	        "\tcmpw    %2,%0        \n"
	        "\tbne     0b         \n"
	        : "=r"(upper),"=r"(lower),"=r"(tmp)
	        );
  result = upper;
  result = result<<32;
  result = result|lower;

  return(result);
}

#else

#error "No tick counter is available!"

#endif

/********************************** RDTSC DEFINITION ***************************************************/
//================= parameters ================
static struct cdev cdev_info;
static struct class *ringLed_class;
static int initialize_ringLed_Driver(void);
void write1(void);
void write0(void);
resource_size_t start = 0, len = 0;
static void __iomem *reg_base;

u32 val_data = 0;
int value=1,i;
unsigned offset=4;
void __iomem *reg_data ;

//============================================

void reset(struct spi_device *spi)
{	
	gpio_set_value_cansleep(pin,0);
	udelay(55);
}

void Bit1_send(void)
{   
	write1();
	ndelay(700);
	write0();
	ndelay(600);
}

void Bit0_send(void)
{
	write1(); 
	ndelay(400);
	write0();
	ndelay(900);
}
   
ktime_t t1 ,t2;
long long unsigned int i1,i2;

// hr timer handler after timer expores
enum hrtimer_restart th_r(struct hrtimer *t)  
{
	long long unsigned int diff;
	i2=rdtsc();
	diff=(i2-i1);
	diff*=1000;
	do_div(diff,402);
	printk("Hrtimer delay (rdtsc) : %llu\n",diff);
	return 0;
}

// hr timer handler after timer expores
enum hrtimer_restart th_k(struct hrtimer *t)  
{
	t2=ktime_get();
	printk("Hrtimer delay (ktime) : %llu\n",ktime_to_ns(ktime_sub(t2,t1)));
	return 0;
}
static struct hrtimer hrt;

void bit_bang_write(void)
{
	int i,l;
	for (l=1;l<25;l++)
	{
		for (i=0;i<16*l;i++)
			Bit1_send();
		mdelay(500);
	}

}

//gpio_init: io multiplexing
static int gpio_init (void)
{
	if(gpio_request(mux1,"mux1")!=0) { printk("Error requesting mux1\n");return -1;}
	if(gpio_request(pin,"mux2")!=0) { printk("Error requesting mux2\n");return -1;}
	if(gpio_request(ls,"level_shifter")!=0) { printk("Error requesting Level shifter\n");return -1;}
	if(gpio_direction_output(mux1,0)!=0) {printk("Error in GPIO direction mux1"); return -1;}
	if(gpio_direction_output(pin,0)){printk("Error in GPIO direction pin"); return -1;}
	if(gpio_direction_output(ls,0)!=0) {printk("Error in GPIO direction level_shifter"); return -1;}
	gpio_set_value_cansleep(mux1,0);
	gpio_set_value_cansleep(pin,0);
	gpio_set_value_cansleep(ls,0);
	return 0;
}

// ===================== measurements begin ========================
//==================================================================

// measure the execution time of using gpio_set_value_cansleep() function
void measure_setvalue(void)
{	
	long long unsigned int diff;
	i1=rdtsc();
	gpio_set_value_cansleep(pin,1);
	i2=rdtsc();
	gpio_set_value_cansleep(pin,0);
	diff=(i2-i1);
	diff*=1000;
	do_div(diff,402);
	printk("gpio_set_value_cansleep writing Time  : %llu nano seconds %llu ticks \n",diff,(i2-i1));
}

// measure the execution time of writing to register
void measure_registerWrite(void)
{
	long long unsigned int diff;
	i1=rdtsc();
	write1();
	i2=rdtsc();
	write0();
	diff=(i2-i1);
	diff*=1000;
	do_div(diff,402);	
	printk("Register writing Time  : %llu nano seconds %llu ticks \n",diff,(i2-i1));
}

// measure the execution time of ndelay(350) using rdtsc
void measure_ndelay_rdtsc(void)
{
	long long unsigned int diff;
	i1=rdtsc();
	ndelay(350);
	i2=rdtsc();
	diff=(i2-i1);
	diff*=1000;
	do_div(diff,402);
	printk("ndelay Time (rdtsc) : %llu Nanoseconds %llu ticks \n",diff,(i2-i1));
}


// measure the execution time of ndelay(350) using ktime_get
void measure_ndelay_ktime(void)
{
	t1=ktime_get();
	ndelay(350);
	t2=ktime_get();
	printk("ndelay Time (ktime_get) : %llu Nanoseconds \n",ktime_to_ns(ktime_sub(t2,t1)));
}

// measure the execution time of ndelay(350) using hrtimer
void measure_ndelay_hrtimer(void)
{
	ktime_t exp=ktime_set(1,0);

  	hrtimer_init(&hrt,CLOCK_MONOTONIC, HRTIMER_MODE_REL);
 	hrt.function = NULL;
 	hrtimer_start( &hrt, exp, HRTIMER_MODE_REL);	
	ndelay(350);
	t2=hrtimer_get_remaining(&hrt);
	printk("ndelay Time (hrtimer) : %llu Nanoseconds \n",ktime_to_ns(ktime_sub(exp,t2)));
	hrtimer_cancel(&hrt);
}

// measure the execution time of hrtimer (with expire time 350ns) using rdtsc
void measure_hrtimer_rdtsc(void)
{
	ktime_t exp = ktime_set(0,350);
	hrtimer_init(&hrt,CLOCK_MONOTONIC, HRTIMER_MODE_REL);
 	hrt.function = th_r;
 	i1=rdtsc();
 	hrtimer_start( &hrt, exp, HRTIMER_MODE_REL);
 	mdelay(1000);
 	hrtimer_cancel(&hrt);
}

// measure the execution time of hrtimer (with expire time 350ns) using ktime_get
void measure_hrtimer_ktime(void)
{
	ktime_t exp = ktime_set(0,350);
	hrtimer_init(&hrt,CLOCK_MONOTONIC, HRTIMER_MODE_REL);
 	hrt.function = th_k;
 	t1=ktime_get();
 	hrtimer_start( &hrt, exp, HRTIMER_MODE_REL);
 	mdelay(1000);
 	hrtimer_cancel(&hrt);
}

// ===================== measurements end ========================
//================================================================

// write 1 (high voltage) to gpio 
void write1()
{
	val_data = ioread32(reg_data);
	iowrite32(val_data | BIT(offset % 32), reg_data);
}

// write 0 (low voltage) to gpio 
void write0()
{
	val_data = ioread32(reg_data);	
	iowrite32(val_data & ~BIT(offset % 32), reg_data);
}


static int initialize_ringLed_Driver(void)
{
	struct pci_dev *pdev;
	int result=1;

	printk("Initializing\n");
	// pin multiplexing
	result=gpio_init();
	if(result<0)
	{
		printk("error in Gpio initialization\n");
		return -1;
	}

	pdev = pci_get_device(0x8086,0x0934, NULL);

	start = pci_resource_start(pdev, GIP_GPIO_BAR);
	len = pci_resource_len(pdev, GIP_GPIO_BAR);
	if (!start || len == 0) {
	printk( "bar%d not set\n", GIP_GPIO_BAR);
	}
	// getting the base address
	reg_base = ioremap_nocache(start, len);
	if (NULL == reg_base) {
		printk( "I/O memory remapping failed\n");
	}

	reg_data= reg_base + PORTA_DATA;

	//===== measurements =======
	measure_setvalue();
	measure_registerWrite();
	measure_ndelay_rdtsc();
	measure_ndelay_ktime();
	measure_ndelay_hrtimer();
	measure_hrtimer_rdtsc();
	measure_hrtimer_ktime();
	bit_bang_write();
	
	return 0;
}

static void __exit ringLed_exit(void)
{
	//unregister_chrdev(MAJOR_NUMBER, "spiLed");
	gpio_free(mux1);
	gpio_free(pin);
	gpio_free(ls);
	hrtimer_cancel( &hrt);
	unregister_chrdev_region(device_no,DEVICES);
	cdev_del(&cdev_info);
	device_destroy(ringLed_class, MKDEV(MAJOR(device_no), 0));
	class_destroy(ringLed_class);
	
	printk(KERN_ALERT "exited\n");
	return;
}


//*************************
module_init( initialize_ringLed_Driver);
module_exit( ringLed_exit);
MODULE_LICENSE("GPL");

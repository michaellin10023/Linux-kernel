/* ----------------------------------------------- DRIVER HCSR --------------------------------------------------
 
 Basic driver example to show skelton methods for several file operations.
 
 ----------------------------------------------------------------------------------------------------------------*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/string.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <asm/div64.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <asm/delay.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/stat.h>
#define DEVICE_NAME                 "HCSR"  // device name to be created and registered
#define MAX 15
#if defined(__i386__)

static __inline__ unsigned long long rdtsc(void)   // This has been obtained from an outside source
{													// this is assembly code to obtain the Time stamp counter value
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
static int gpio_num_1;
static int gpio_num_2;
static int gpio_num_3;
static int gpio_num_4;
static int gpio_num_5;
static int gpio_num_6;
static int gpio_num_7;
static int gpio_num_8;

void cmd_table(int, int);			// The configuration parameters are stored in this.
struct Cmd{
	char cmd_name[20];
	int para_1;
	int para_2;
};

// configuration 
struct configure{					// structure to store the user input parameters
    int trigPin; // trigPin=10 if trig is connected to IO 10
    int echoPin; 
    int m;      //sample Per Measurement
    int delta;  //sample Period
} conf;

struct fifo_buffer{                      // the fifo buffer
	unsigned long long int time_stamp;
	unsigned long long int value;
} buff;

/* per device structure */
struct HCSR_dev {
    struct cdev cdev;               /* The cdev structure */
    char name[20];                  /* Name of device*/
    struct fifo_buffer buff[5];            /* buffer for the input string */
    int head;
	int curr;                        // indices for the fifo buffer
	int count;
    struct configure conf; 
    int measurement_flag;
} *HCSR_devp;

static struct miscdevice my_dev[MAX];// hcsr_misc;
static struct hrtimer hrt;
int ultrasonic_speed= 346;    // 346 m/s
long long unsigned int distance = 0;
int trigger_flag=0;    
int expire_time=5;
int data_available=0;
static int irq_num;


void Triggering (void)  // this function triggers the sensor thus causing ultra sound to be transmitted.
{	
	trigger_flag=1;
	gpio_set_value_cansleep(HCSR_devp->conf.trigPin,0);
	udelay(2);
	gpio_set_value_cansleep(HCSR_devp->conf.trigPin,1);
	udelay(10);
	//mdelay(2000);
	gpio_set_value_cansleep(HCSR_devp->conf.trigPin,0);
	trigger_flag=0;
	//printk("Ultrasonic has been triggerd\n");
	
}

void write_buffer(unsigned long long int tsc, unsigned long long int val) // This function is used to write into the fifo buffer
{
	int next;
	 next = HCSR_devp->curr + 1;
    if (next >= 5)
        next = 0;

    if (next == HCSR_devp->head) 
    	{ HCSR_devp->head+=1;
          	if(HCSR_devp->head>=5)  
          		HCSR_devp->head=0;    
    	}

    HCSR_devp->buff[HCSR_devp->curr].time_stamp = tsc;
    HCSR_devp->buff[HCSR_devp->curr].value= val; 
    HCSR_devp->curr = next;            
	}

int read_buffer(struct fifo_buffer *fb) // This function reads from the fifo buffer. this reads the first of the latest 5
{
	int next;
	if (HCSR_devp->head == HCSR_devp->curr)
		return -1;
	
    next = HCSR_devp->head + 1;
    if(next >= 5)
        next = 0;

    fb->time_stamp=HCSR_devp->buff[HCSR_devp->head].time_stamp;
	fb->value = HCSR_devp->buff[HCSR_devp->head].value;
    HCSR_devp->head = next;             
    return 0;  

}
ktime_t ref1;
ktime_t ref2;
static irq_handler_t handling_irq(unsigned int irq, void *dev_id) // interrupt handler
{
	int val=gpio_get_value(HCSR_devp->conf.echoPin);
	if(val==1)												// rising edge
	{
	ref1=ktime_set(0,0);
	ref1= ktime_get();
	irq_set_irq_type(irq_num,IRQ_TYPE_EDGE_FALLING);
	}

	else					// falling edge

	{
		
		ref2=ktime_set(0,0);				// performing the calculations and fiding the distance.
		ref2=ktime_get();
		//ktime_t time_lapsed = ktime_sub(ref2, ref1);
		long long int t = ktime_to_us(ktime_sub(ref2,ref1));  
		distance = ((t*ultrasonic_speed)/2);
		do_div(distance,100000);
		data_available=1;
		irq_set_irq_type(irq_num,IRQ_TYPE_EDGE_RISING);
		
	}

	return (irq_handler_t) IRQ_HANDLED;
	}


static int delay_flag=1;
int timer_int_handler_callback(struct hrtimer *t)  // hr timer handler after timer expores
{
delay_flag=1;
return 0;
}
void measure(void)			// makes m+2 measurement ever delta milliseconds
{
	static int i;
	static unsigned long long sum=0;
	static unsigned long long max=0;
	static unsigned long long min= 100000000;
	HCSR_devp->measurement_flag=1;
	
	for (i=0;i< HCSR_devp->conf.m +2 ;i++)
	 {  ktime_t exp_t;
	 	if(delay_flag==0)
	 	{
	 		//printk("inisde delay_flag\n");
	 		i--;
	 		continue;
	 	}
	 	Triggering();
	 	//while(data_available==0)
	 	mdelay(1);
	 	//data_available=0;
	 	sum+=distance;
	 	//printk("sum : %llu \n",sum);
	 	if(distance > max)
	 		max=distance;
	 	if(distance<min)
	 		min=distance;
	 	//mdelay(HCSR_devp->conf.delta);
	 exp_t = ktime_set(0,(HCSR_devp->conf.delta)*1000*1000);
	
	hrt.function = &timer_int_handler_callback;
	hrtimer_start( &hrt, exp_t, HRTIMER_MODE_REL);					//hr_timer used to implement delay
	delay_flag=0;
	 }
	 
	 sum = sum - max;
	 sum = sum - min;
	 do_div(sum,HCSR_devp->conf.m);
	 //printk("sume calculated : %llu \n",sum);
	 write_buffer(rdtsc(),sum);
	 //printk("RDTSC %llu \n",rdtsc());
	 HCSR_devp->measurement_flag=0;
	}

// it is used to configuring parametes
void configure_parameters(int m , int d)
{
	HCSR_devp->conf.m =m;//3;
	HCSR_devp->conf.delta =d;//20;
}
/*
* Open HCSR driver
*/
int HCSR_driver_open(struct inode *inode, struct file *file)
{
	struct HCSR_dev *HCSR_devp;


	/* Get the per-device structure that contains this cdev */
	HCSR_devp = container_of(inode->i_cdev, struct HCSR_dev, cdev);


	/* Easy access to cmos_devp from rest of the entry points */
	file->private_data = HCSR_devp;
	printk("\n%s is openning \n", HCSR_devp->name);
	return 0;
}

/*
 * Release HCSR driver
 */
int HCSR_driver_release(struct inode *inode, struct file *file)
{
	struct HCSR_dev *HCSR_devp = file->private_data;
	
	printk("\n%s is closing\n", HCSR_devp->name);
	
	return 0;
}

/*
 * Write to HCSR driver
 */
int HCSR_driver_write(struct file *file, int * T)
{
	struct HCSR_dev *HCSR_devp = file->private_data;
	int write_parameter;
	get_user(write_parameter,T);
	if(HCSR_devp->measurement_flag==1) // if there is a measurement going return EINVAL
		return -EINVAL;
	else 
	{
		if(write_parameter!=0) // if the parameter is non-zero clear the buffer else make the measurement.
		{
			//printk("Clearing buffer\n");
			HCSR_devp->curr=0;
			HCSR_devp->head=0;
		}
		measure(); // make the actual measurement.
	}
		return 0;
}
/*
 * Read to HCSR driver
 */
ssize_t HCSR_driver_read(struct file *file, struct fifo_buffer *f)
{
	int bytes_read = 0;
	struct HCSR_dev *HCSR_devp = file->private_data;
	struct fifo_buffer *tmp = (struct fifo_buffer * )kmalloc(sizeof(struct fifo_buffer), GFP_KERNEL);
	while(read_buffer(tmp)==-1); // wait until data is written
	copy_to_user(f,tmp,sizeof(struct fifo_buffer)); // copy to user space.
	kfree(tmp);
	return bytes_read;

}

int HCSR_ioctl(struct file *file,  struct Cmd * cmd)   // pin and parameter configuration
{
    
    int i;
    struct Cmd *cmd_buf;
    cmd_buf = ( struct Cmd* )kmalloc(sizeof( struct Cmd), GFP_KERNEL); 
    copy_from_user(cmd_buf, cmd, sizeof(struct Cmd));
 
 
    if(strcmp(cmd_buf->cmd_name, "CONFIG_PINS") == 0){  // pin configuration
 
        
        if(cmd_buf->para_1 < 0 || cmd_buf->para_1 > 19){  // checking  if its valid
           return -EINVAL;
        }
 
        if(cmd_buf->para_2 < 0 || cmd_buf->para_2 > 19 || cmd_buf->para_2 ==7 || cmd_buf->para_2 == 8) { // checking if echo is valid
           return -EINVAL;
        }
 
      cmd_table( cmd_buf->para_1, cmd_buf->para_2);  // pin mux table
               
    }
 
    else if(strcmp(cmd_buf->cmd_name, "SET_PARAMETERS") == 0){   // configuring parameters
        configure_parameters(cmd_buf->para_1,cmd_buf->para_2);        

        
        return 0;
    }
    else{
        printk("command input error!\n");
        return -EINVAL;
    }
    
    return 0;
}


/* File operations structure. Defined in linux/fs.h */
static struct file_operations HCSR_fops = {
    .owner		= THIS_MODULE,           /* Owner */
    .open		= HCSR_driver_open,        /* Open method */
    .release	= HCSR_driver_release,     /* Release method */
    .write		= HCSR_driver_write,       /* Write method */
    .read		= HCSR_driver_read,        /* Read method */
    .unlocked_ioctl         = HCSR_ioctl,
};


int trig_callback(void * data)
{
	//printk("Ultrasonic transmitter triggered \n");
	trigger_flag=1;
	return HRTIMER_NORESTART;
}




static short int num_devices = 1;
module_param(num_devices, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);



int __init HCSR_driver_init(void)   //initilize HCSR driver
{
	printk("Number of devices : %d\n",num_devices);
	char name_dev[20];
	/* Registering the Miscellaneous drivers */
	int i=0;
	int retval;
	
	for(i=0;i<num_devices;i++)   // registering misc devices
	{
    my_dev[i].minor = i;
	sprintf(name_dev,"HCSR_%d",i); 
	my_dev[i].name=name_dev;
    	my_dev[i].fops = &HCSR_fops;
	printk("Registering %s\n", my_dev[i].name);
    	retval = misc_register(&my_dev[i]);
	}
	
	
	HCSR_devp = (struct HCSR_dev *)kmalloc(sizeof(struct HCSR_dev), GFP_KERNEL);
	//initializing HCSR indices
	HCSR_devp->head=0;
	HCSR_devp->curr=0;
	HCSR_devp->count=0;
	hrtimer_init(&hrt,CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	
	if (!HCSR_devp) {
		printk("Bad Kmalloc HCSR-DEVP\n"); return -ENOMEM;
	}

	return 0;
}
/* Driver Exit */
void __exit HCSR_driver_exit(void)
{
	// Free resources.
	static int i=0;
	gpio_free(gpio_num_1);
	gpio_free(gpio_num_2);
	gpio_free(gpio_num_3);
	gpio_free(gpio_num_4);
	gpio_free(gpio_num_5);
	gpio_free(gpio_num_6);
	gpio_free(gpio_num_7);
	gpio_free(gpio_num_8);
	gpio_free(HCSR_devp->conf.trigPin);
	gpio_free(HCSR_devp->conf.echoPin);
	kfree(HCSR_devp);
	free_irq(irq_num,NULL);
	
	int r=hrtimer_cancel( &hrt);
	/*De-registering the MISC Drivers*/
	for (i=0;i<num_devices;i++)
	misc_deregister(&my_dev[i]);
	printk("HCSR driver removed.\n");
}

module_init(HCSR_driver_init);
module_exit(HCSR_driver_exit);
MODULE_LICENSE("GPL v2");


void cmd_table(int pin_1, int pin_2){  // 1: output, 2:input
 
    //printk("hello this is cmd table\n");
    //struct HCSR_dev *HCSR_devp = file->private_data;
 
    switch(pin_1){  //trig, output
        case 0:    
            //printk("trig pin is 0\n");
            gpio_num_1 = 11;
            gpio_num_2 = 32;
            if( gpio_request(11, "gpio_out_11") != 0 )  printk("gpio_out_11 error!\n");
            if( gpio_request(32, "dir_out_32") != 0 )  printk("dir_out_32 error!\n");
            HCSR_devp->conf.trigPin = 11;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value(32, 0);
            break;
        case 1:
           // printk("trig pin is 1\n");
            gpio_num_1 = 12;
            gpio_num_2 = 28;
            gpio_num_3 = 45;
            if( gpio_request(12, "gpio_out_12") != 0 )  printk("gpio_out_12 error!\n");
            if( gpio_request(28, "dir_out_28") != 0 )  printk("dir_out_28 error!\n");
            if( gpio_request(45, "pin_mux_45") != 0 )  printk("pin_mux_45 error!\n");
            HCSR_devp->conf.trigPin = 12;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value_cansleep(28, 0);
            gpio_set_value_cansleep(45, 0);
            break;
        case 2:
            //printk("trig pin is 2\n");
            gpio_num_1 = 13;
            gpio_num_2 = 34;
            gpio_num_3 = 77;
            if( gpio_request(13, "gpio_out_13") != 0 )  printk("gpio_out_13 error!\n");
            if( gpio_request(34, "dir_out_34") != 0 )  printk("dir_out_34 error!\n");
            if( gpio_request(77, "pin_mux_77") != 0 )  printk("pin_mux_77 error!\n");
            HCSR_devp->conf.trigPin = 13;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value_cansleep(34, 0);
            gpio_set_value_cansleep(77, 0);
            break;
        case 3:
            //printk("trig pin is 3\n");
            gpio_num_1 = 14;
            gpio_num_2 = 16;
            gpio_num_3 = 76;
            gpio_num_4 = 64;
            if( gpio_request(14, "gpio_out_14") != 0 )  printk("gpio_out_14 error!\n");
            if( gpio_request(16, "dir_out_16") != 0 )  printk("dir_out_16 error!\n");
            if( gpio_request(76, "pin_mux_76") != 0 )  printk("pin_mux_76 error!\n");
            if( gpio_request(64, "pin_mux_64") != 0 )  printk("pin_mux_64 error!\n");
            HCSR_devp->conf.trigPin = 14;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value_cansleep(16, 0);
            gpio_set_value_cansleep(76, 0);
            gpio_set_value_cansleep(64, 0);
            break;
        case 4:
            //printk("trig pin is 4\n");
            gpio_num_1 = 6;
            gpio_num_2 = 36;
            if( gpio_request(6, "gpio_out_6") != 0 )  printk("gpio_out_6 error!\n");
            if( gpio_request(36, "dir_out_36") != 0 )  printk("dir_out_36 error!\n");
            HCSR_devp->conf.trigPin = 6;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value_cansleep(36, 0);
            break;
        case 5:
            //printk("trig pin is 5\n");
            gpio_num_1 = 0;
            gpio_num_2 = 18;
            gpio_num_3 = 66;
            if( gpio_request(0, "gpio_out_0") != 0 )  printk("gpio_out_0 error!\n");
            if( gpio_request(18, "dir_out_18") != 0 )  printk("dir_out_18 error!\n");
            if( gpio_request(66, "pin_mux_66") != 0 )  printk("pin_mux_66 error!\n");
            HCSR_devp->conf.trigPin = 0;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value_cansleep(18, 0);
            gpio_set_value_cansleep(66, 0);
            break;
        case 6:
            //printk("trig pin is 6\n");
            gpio_num_1 = 1;
            gpio_num_2 = 20;
            gpio_num_3 = 68;
            if( gpio_request(1, "gpio_out_1") != 0 )  printk("gpio_out_1 error!\n");
            if( gpio_request(20, "dir_out_20") != 0 )  printk("dir_out_20 error!\n");
            if( gpio_request(68, "pin_mux_68") != 0 )  printk("pin_mux_68 error!\n");
            HCSR_devp->conf.trigPin = 1;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value_cansleep(20, 0);
            gpio_set_value_cansleep(68, 0);
            break;
        case 7:
            //printk("trig pin is 7\n");
            gpio_num_1 = 38;
            if( gpio_request(38, "gpio_out_38") != 0 )  printk("gpio_out_38 error!\n");
            HCSR_devp->conf.trigPin = 38;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            break;
        case 8:
            //printk("trig pin is 8\n");
            gpio_num_1 = 40;
            if( gpio_request(40, "gpio_out_40") != 0 )  printk("gpio_out_40 error!\n");
            HCSR_devp->conf.trigPin = 40;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            break;
 
        case 9:
            //printk("trig pin is 9\n");
            gpio_num_1 = 4;
            gpio_num_2 = 22;
            gpio_num_3 = 70;
            if( gpio_request(4, "gpio_out_4") != 0 )  printk("gpio_out_4 error!\n");
            if( gpio_request(22, "dir_out_22") != 0 )  printk("dir_out_22 error!\n");
            if( gpio_request(70, "pin_mux_70") != 0 )  printk("pin_mux_70 error!\n");
            HCSR_devp->conf.trigPin = 4;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value_cansleep(22, 0);
            gpio_set_value_cansleep(70, 0);
            break;
        case 10:
            //printk("trig pin is 10\n");
            gpio_num_1 = 10;
            gpio_num_2 = 26;
            gpio_num_3 = 74;
            if( gpio_request(10, "gpio_out_10") != 0 )  printk("gpio_out_10 error!\n");
            if( gpio_request(26, "dir_out_26") != 0 )  printk("dir_out_26 error!\n");
            if( gpio_request(74, "pin_mux_74") != 0 )  printk("pin_mux_74 error!\n");
            HCSR_devp->conf.trigPin = 10;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value_cansleep(26, 0);
            gpio_set_value_cansleep(74, 0);
            break;
        case 11:
            //printk("trig pin is 11\n");
            gpio_num_1 = 5;
            gpio_num_2 = 24;
            gpio_num_3 = 44;
            gpio_num_4 = 72;
            if( gpio_request(5, "gpio_out_5") != 0 )  printk("gpio_out_5 error!\n");
            if( gpio_request(24, "dir_out_24") != 0 )  printk("dir_out_24 error!\n");
            if( gpio_request(44, "pin_mux_44") != 0 )  printk("pin_mux_44 error!\n");
            if( gpio_request(72, "pin_mux_72") != 0 )  printk("pin_mux_72 error!\n");
            HCSR_devp->conf.trigPin = 5;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value_cansleep(24, 0);
            gpio_set_value_cansleep(44, 0);
            gpio_set_value_cansleep(72, 0);
            break;
        case 12:
            //printk("trig pin is 12\n");
            gpio_num_1 = 15;
            gpio_num_2 = 42;
            if( gpio_request(15, "gpio_out_15") != 0 )  printk("gpio_out_15 error!\n");
            if( gpio_request(42, "dir_out_42") != 0 )  printk("dir_out_42 error!\n");
            HCSR_devp->conf.trigPin = 15;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value_cansleep(42, 0);
            break;
        case 13:
            //printk("trig pin is 13\n");
            gpio_num_1 = 7;
            gpio_num_2 = 30;
            gpio_num_3 = 46;
            if( gpio_request(7, "gpio_out_7") != 0 )  printk("gpio_out_7 error!\n");
            if( gpio_request(30, "dir_out_30") != 0 )  printk("dir_out_30 error!\n");
            if( gpio_request(46, "pin_mux_46") != 0 )  printk("pin_mux_46 error!\n");
            HCSR_devp->conf.trigPin = 7;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value_cansleep(30, 0);
            gpio_set_value_cansleep(46, 0);
            break;
        case 14:
            //printk("trig pin is 14\n");
            gpio_num_1 = 48;
            if( gpio_request(48, "gpio_out_48") != 0 )  printk("gpio_out_48 error!\n");
            HCSR_devp->conf.trigPin = 48;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            break;
        case 15:
            //printk("trig pin is 15\n");
            gpio_num_1 = 50;
            if( gpio_request(50, "gpio_out_50") != 0 )  printk("gpio_out_50 error!\n");
            HCSR_devp->conf.trigPin = 50;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            break;
        case 16:
            //printk("trig pin is 16\n");
            gpio_num_1 = 52;
            if( gpio_request(52, "gpio_out_52") != 0 )  printk("gpio_out_52 error!\n");
            HCSR_devp->conf.trigPin = 52;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            break;
        case 17:
            //printk("trig pin is 17\n");
            gpio_num_1 = 54;
            if( gpio_request(54, "gpio_out_54") != 0 )  printk("gpio_out_54 error!\n");
            HCSR_devp->conf.trigPin = 54;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            break;
        case 18:
            //printk("trig pin is 18\n");
            gpio_num_1 = 56;
            gpio_num_2 = 60;
            gpio_num_3 = 78;
            if( gpio_request(56, "gpio_out_56") != 0 )  printk("gpio_out_56 error!\n");
            if( gpio_request(60, "pin_mux_60") != 0 )  printk("pin_mux_60 error!\n");
            if( gpio_request(78, "pin_mux_78") != 0 )  printk("pin_mux_78 error!\n");
            HCSR_devp->conf.trigPin = 56;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value_cansleep(60, 1);
            gpio_set_value_cansleep(78, 1);
            break;
        case 19:
            //printk("trig pin is 19\n");
            gpio_num_1 = 58;
            gpio_num_2 = 60;
            gpio_num_3 = 79;
            if( gpio_request(58, "gpio_out_58") != 0 )  printk("gpio_out_58 error!\n");
            if( gpio_request(60, "pin_mux_60") != 0 )  printk("pin_mux_60 error!\n");
            if( gpio_request(79, "pin_mux_79") != 0 )  printk("pin_mux_79 error!\n");
            HCSR_devp->conf.trigPin = 58;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value_cansleep(60, 1);
            gpio_set_value_cansleep(79, 1);
            break;
    }
 
    switch(pin_2){  //echo, input
    	int ret;
        case 0:    
            gpio_num_5 = 11;
            gpio_num_6 = 32;
            if( gpio_request(11, "gpio_in_11") != 0 )  printk("gpio_in_11 error!\n");
            if( gpio_request(32, "dir_in_32") != 0 )  printk("dir_in_32 error!\n");
            irq_num = gpio_to_irq(gpio_num_5);
            if(irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", 0);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 11;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(32,1);
            break;
        case 1:
            gpio_num_5 = 12;
            gpio_num_6 = 28;
            gpio_num_7 = 45;
            if( gpio_request(12, "gpio_in_12") != 0 )  printk("gpio_in_12 error!\n");
            if( gpio_request(28, "dir_in_28") != 0 )  printk("dir_in_28 error!\n");
            if( gpio_request(45, "pin_mux_45") != 0 )  printk("pin_mux_45 error!\n");
            irq_num = gpio_to_irq(gpio_num_5);
            if(irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", 0);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 12;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(28,1);
            gpio_set_value_cansleep(45, 0);
            break;
        case 2:
            gpio_num_5 = 13;
            gpio_num_6 = 34;
            gpio_num_7 = 77;
            if( gpio_request(13, "gpio_in_13") != 0 )  printk("gpio_in_13 error!\n");
            if( gpio_request(34, "dir_in_34") != 0 )  printk("dir_in_34 error!\n");
            if( gpio_request(77, "pin_mux_77") != 0 )  printk("pin_mux_77 error!\n");
            irq_num = gpio_to_irq(gpio_num_5);
            if(irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", 0);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 13;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(34,1);
            gpio_set_value_cansleep(77, 0);
            break;
        case 3:
            gpio_num_5 = 14;
            gpio_num_6 = 16;
            gpio_num_7 = 76;
            gpio_num_8 = 64;
            if( gpio_request(14, "gpio_in_14") != 0 )  printk("gpio_in_14 error!\n");
            if( gpio_request(16, "dir_in_16") != 0 )  printk("dir_in_16 error!\n");
            if( gpio_request(76, "pin_mux_76") != 0 )  printk("pin_mux_76 error!\n");
            if( gpio_request(64, "pin_mux_64") != 0 )  printk("pin_mux_64 error!\n");
            irq_num = gpio_to_irq(gpio_num_5);
            if(irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(irq_num, (irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", 0);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 14;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(16,1);
            gpio_set_value_cansleep(76, 0);
            gpio_set_value_cansleep(64, 0);
            break;
        case 4:
            if( gpio_request(6, "gpio_in_6") != 0 )  printk("gpio_in_6 error!\n");
            HCSR_devp->conf.echoPin = 6;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            irq_num= gpio_to_irq(HCSR_devp->conf.echoPin);
			if(irq_num<0)
			{printk("IRQ NUMBER ERROR\n");return -1;}
			ret = request_irq(irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", 0);
			if (ret < 0) 
			{printk("Error in request_irq\n");return -1;}
            break;
        case 5:
            gpio_num_5 = 0;
            gpio_num_6 = 18;
            gpio_num_7 = 66;
            if( gpio_request(0, "gpio_in_0") != 0 )  printk("gpio_in_0 error!\n");
            if( gpio_request(18, "dir_in_18") != 0 )  printk("dir_in_18 error!\n");
            if( gpio_request(66, "pin_mux_66") != 0 )  printk("pin_mux_66 error!\n");
            irq_num = gpio_to_irq(gpio_num_5);
            if(irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", 0);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 0;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(18, 1);
            gpio_set_value_cansleep(66, 0);
            break;
        case 6:
            gpio_num_5 = 1;
            gpio_num_6 = 20;
            gpio_num_7 = 68;
            if( gpio_request(1, "gpio_in_1") != 0 )  printk("gpio_in_1 error!\n");
            if( gpio_request(20, "dir_in_20") != 0 )  printk("dir_in_20 error!\n");
            if( gpio_request(68, "pin_mux_68") != 0 )  printk("pin_mux_68 error!\n");
            irq_num = gpio_to_irq(gpio_num_5);
            if(irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", 0);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 1;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(20,1);
            gpio_set_value_cansleep(68, 0);
            break;
        case 7:
            gpio_num_5 = 38;
            if( gpio_request(38, "gpio_in_38") != 0 )  printk("gpio_in_38 error!\n");
            irq_num = gpio_to_irq(gpio_num_5);
            if(irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", 0);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 38;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            break;
        case 8:
            gpio_num_5 = 40;
            if( gpio_request(40, "gpio_in_40") != 0 )  printk("gpio_in_40 error!\n");
            irq_num = gpio_to_irq(gpio_num_5);
            if(irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", 0);
            if (ret < 0)   printk("Error in request_irq\n");
            gpio_direction_input(HCSR_devp->conf.echoPin);
            HCSR_devp->conf.echoPin = 40;
            break;
        case 9:
            gpio_num_5 = 4;
            gpio_num_6 = 22;
            gpio_num_7 = 70;
            if( gpio_request(4, "gpio_in_4") != 0 )  printk("gpio_in_4 error!\n");
            if( gpio_request(22, "dir_in_22") != 0 )  printk("dir_in_22 error!\n");
            if( gpio_request(70, "pin_mux_70") != 0 )  printk("pin_mux_70 error!\n");
            irq_num = gpio_to_irq(gpio_num_5);
            if(irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", 0);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 4;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(22,1);
            gpio_set_value_cansleep(70, 0);
            break;
        case 10:
            gpio_num_5 = 10;
            gpio_num_6 = 26;
            gpio_num_7 = 74;
            if( gpio_request(10, "gpio_in_10") != 0 )  printk("gpio_in_10 error!\n");
            if( gpio_request(26, "dir_in_26") != 0 )  printk("dir_in_26 error!\n");
            if( gpio_request(74, "pin_mux_74") != 0 )  printk("pin_mux_74 error!\n");
            irq_num = gpio_to_irq(gpio_num_5);
            if(irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(irq_num, (irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", 0);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 10;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(26,1);
            gpio_set_value_cansleep(74, 0);
            break;
        case 11:
            //printk("echo pin is 11\n");
            gpio_num_5 = 5;
            gpio_num_6 = 24;
            gpio_num_7 = 44;
            gpio_num_8 = 72;
            if( gpio_request(5, "gpio_in_5") != 0 )  printk("gpio_in_5 error!\n");
            if( gpio_request(24, "dir_in_24") != 0 )  printk("dir_in_24 error!\n");
            if( gpio_request(44, "pin_mux_44") != 0 )  printk("pin_mux_44 error!\n");
            if( gpio_request(72, "pin_mux_72") != 0 )  printk("pin_mux_72 error!\n");
            irq_num = gpio_to_irq(gpio_num_5);
            if(irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", 0);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 5;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(24,1);
            gpio_set_value_cansleep(44, 0);
            gpio_set_value_cansleep(72, 0);
            break;
        case 12:
            gpio_num_5 = 15;
            gpio_num_6 = 42;
            if( gpio_request(15, "gpio_in_15") != 0 )  printk("gpio_in_15 error!\n");
            if( gpio_request(42, "dir_in_42") != 0 )  printk("dir_in_42 error!\n");
            irq_num = gpio_to_irq(gpio_num_5);
            if(irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(irq_num, (irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", 0);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 15;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(42,1);
            break;
        case 13:
            gpio_num_5 = 7;
            gpio_num_6 = 30;
            gpio_num_7 = 46;
            if( gpio_request(7, "gpio_in_7") != 0 )  printk("gpio_in_7 error!\n");
            if( gpio_request(30, "dir_in_30") != 0 )  printk("dir_in_30 error!\n");
            if( gpio_request(46, "pin_mux_46") != 0 )  printk("pin_mux_46 error!\n");
            irq_num = gpio_to_irq(gpio_num_5);
            if(irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", 0);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 7;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(30,1 );
            gpio_set_value_cansleep(46, 0);
            break;
        case 14:
            gpio_num_5 = 48;
            if( gpio_request(48, "gpio_in_48") != 0 )  printk("gpio_in_48 error!\n");
            irq_num = gpio_to_irq(gpio_num_5);
            if(irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(irq_num, (irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", 0);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 48;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            break;
        case 15:
            gpio_num_5 = 50;
            if( gpio_request(50, "gpio_in_50") != 0 )  printk("gpio_in_50 error!\n");
            irq_num = gpio_to_irq(gpio_num_5);
            if(irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", 0);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 50;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            break;
        case 16:
            gpio_num_5 = 52;
            if( gpio_request(52, "gpio_in_52") != 0 )  printk("gpio_in_52 error!\n");
            irq_num = gpio_to_irq(gpio_num_5);
            if(irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", 0);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 52;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            break;
        case 17:
            gpio_num_5 = 54;
            if( gpio_request(54, "gpio_in_54") != 0 )  printk("gpio_in_54 error!\n");
            irq_num = gpio_to_irq(gpio_num_5);
            if(irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", 0);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 54;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            break;
        case 18:
            gpio_num_5 = 56;
            gpio_num_6 = 60;
            gpio_num_7 = 78;
            if( gpio_request(56, "gpio_in_56") != 0 )  printk("gpio_in_56 error!\n");
            if( gpio_request(60, "pin_mux_60") != 0 )  printk("pin_mux_60 error!\n");
            if( gpio_request(78, "pin_mux_78") != 0 )  printk("pin_mux_78 error!\n");
            irq_num = gpio_to_irq(gpio_num_5);
            if(irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", 0);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 56;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(60, 1);
            gpio_set_value_cansleep(78, 1);
            break;
        case 19:
            gpio_num_5 = 58;
            gpio_num_6 = 60;
            gpio_num_7 = 79;
            if( gpio_request(58, "gpio_in_58") != 0 )  printk("gpio_in_58 error!\n");
            if( gpio_request(60, "pin_mux_60") != 0 )  printk("pin_mux_60 error!\n");
            if( gpio_request(79, "pin_mux_79") != 0 )  printk("pin_mux_79 error!\n");
            irq_num = gpio_to_irq(gpio_num_5);
            if(irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", 0);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 58;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(60, 1);
            gpio_set_value_cansleep(79, 1);
            break;
    }
}
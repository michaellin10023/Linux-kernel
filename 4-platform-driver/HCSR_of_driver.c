/*
 * A sample program to show the binding of platform driver and device.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include "HCSR_of_device.h"
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
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
#include <linux/kthread.h>

#define DRIVER_NAME	 "HCSR_of_driver"
#define CLASS_NAME  "HCSR"
#define MAX 15
/*
 * Prototypes 
 */

static struct class *HCSR_class;
static dev_t HCSR_dev =1;
static short int num_devices = 0;
static struct P_chip *pchip[MAX];

//************************************************ from HCSR_drv.c *********************************************
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


	int cmd_table(struct HCSR_dev *,int, int);			// The pin-mux table function
	struct Cmd{
		char cmd_name[20];
		int para_1;
		int para_2;
	};


	
	static struct hrtimer hrt;
	int ultrasonic_speed= 346;    // 346 m/s
	long long unsigned int distance = 0;
	int trigger_flag=0;    
	int expire_time=5;
	int data_available=0;
	static int irq_num;


	void Triggering (struct HCSR_dev *HCSR_devp)  // this function triggers the sensor thus causing ultra sound to be transmitted.
	{	

		trigger_flag=1;
		gpio_set_value_cansleep(HCSR_devp->conf.trigPin,0);
		udelay(2);
		gpio_set_value_cansleep(HCSR_devp->conf.trigPin,1);
		udelay(20);
		//mdelay(2000);
		gpio_set_value_cansleep(HCSR_devp->conf.trigPin,0);
		trigger_flag=0;
		//printk("Ultrasonic has been triggerd\n");
		
	}

	void write_buffer(unsigned long long int tsc, unsigned long long int val,struct HCSR_dev *HCSR_devp) // This function is used to write into the fifo buffer
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

	int read_buffer(struct fifo_buffer *fb,struct HCSR_dev *HCSR_devp) // This function reads from the fifo buffer. this reads the first of the latest 5
	{
		int next;
		//printk("read buffer\n");
		if (HCSR_devp->head == HCSR_devp->curr)
			return -1;
		//printk()
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
	static irq_handler_t handling_irq(unsigned int irq, void* d) // interrupt handler
	{
		
		struct HCSR_dev *HCSR_devp;
		long long int t;
		int val;
		HCSR_devp = d;
		
		val=gpio_get_value(HCSR_devp->conf.echoPin);

		if(val==1)												//Handling for rising edge
		{
		
		ref1=ktime_set(0,0);
		ref1= ktime_get();
		irq_set_irq_type(irq_num,IRQ_TYPE_EDGE_FALLING);
		}

	 	else					// falling edge

	 	{

		ref2=ktime_set(0,0);				// performing the calculations and fiding the distance.
		ref2=ktime_get();
		t = ktime_to_us(ktime_sub(ref2,ref1));  
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
	void measure(struct HCSR_dev *HCSR_devp)			// makes m+2 measurement ever delta milliseconds
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
		 		
		 		i--;
		 		continue;
		 	}
		 	Triggering(HCSR_devp);
		 	mdelay(1);
		 	sum+=distance;
		 	if(distance > max)
		 		max=distance;
		 	if(distance<min)
		 		min=distance;

		 exp_t = ktime_set(0,(HCSR_devp->conf.delta)*1000*1000);
		hrt.function = &timer_int_handler_callback;
		hrtimer_start( &hrt, exp_t, HRTIMER_MODE_REL);					//hr_timer used to implement delay
		delay_flag=0;
		 }
		 
		 sum = sum - max;
		 sum = sum - min;
		 do_div(sum,HCSR_devp->conf.m);
		 write_buffer(rdtsc(),sum,HCSR_devp);
		 HCSR_devp->measurement_flag=0;
		 HCSR_devp->latest_distance=distance;
		}

	// it is used to configuring parametes
	void configure_parameters(struct HCSR_dev *HCSR_devp,int m , int d)
	{
		HCSR_devp->conf.m =m;//3;
		HCSR_devp->conf.delta =d;//20;
	}
	/*
	* Open HCSR driver
	*/
	int HCSR_driver_open(struct inode *inode, struct file *file)
	{
		int i=0;
		struct P_chip *pchp;
		int minor_num = iminor(inode);
		pchp=(struct P_chip * ) kmalloc (sizeof(struct P_chip), GFP_KERNEL);
		for (i=0;i<num_devices;i++)
		{
			if(minor_num==pchip[i]->my_misc.minor)
				pchp=pchip[i];
		}
		file->private_data = pchp->HCSR_devp;
		printk("Opening Device %s",pchp->HCSR_devp->name);
		return 0;
	}

	/*
	 * Release HCSR driver
	 */
	int HCSR_driver_release(struct inode *inode, struct file *file)
	{
		struct HCSR_dev *hdp= file->private_data;
		
		printk("\n%s is closing\n", hdp->name);
		
		return 0;
	}

	/*
	 * Write to HCSR driver
	 */
	ssize_t HCSR_driver_write(struct file *file, int * T)
	{
		struct HCSR_dev *HCSR_devp = file->private_data;
		int write_parameter;
		get_user(write_parameter,T);
		
		if(HCSR_devp->measurement_flag==1) // if there is a measurement going return EINVAL
			{printk("error measurement flag set");return -EINVAL;}
		else 
		{
			if(write_parameter!=0) // if the parameter is non-zero clear the buffer else make the measurement.
			{
				
				HCSR_devp->curr=0;
				HCSR_devp->head=0;
			}
			measure(HCSR_devp); // make the actual measurement.
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
		while(read_buffer(tmp,HCSR_devp)==-1); // wait until data is written
		copy_to_user(f,tmp,sizeof(struct fifo_buffer)); // copy to user space.
		kfree(tmp);
		return bytes_read;

	}


	int configure_pins(struct HCSR_dev *HCSR_devp,int t , int e) // used to configure the pins
	{
		if(t < 0 || t > 19){  // checking  if its valid
	           return -EINVAL;
	        }
	 
	        if(e < 0 || e > 19 || e ==7 || e == 8) { // checking if echo is valid
	           return -EINVAL;
	        }
	 
	     cmd_table( HCSR_devp,t, e);
	     return 0;
	}
	int HCSR_ioctl(struct file *file,  struct Cmd * cmd)   // pin and parameter configuration
	{
	    struct HCSR_dev *HCSR_devp = file->private_data;
	    
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
	 
	     cmd_table( HCSR_devp,cmd_buf->para_1, cmd_buf->para_2);  // pin mux table
	               
	    }
	 
	    else if(strcmp(cmd_buf->cmd_name, "SET_PARAMETERS") == 0){   // configuring parameters
	        configure_parameters(HCSR_devp,cmd_buf->para_1,cmd_buf->para_2);        

	        
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


	int measure_thread(void * t)  //  a kthread is spawned for the measurement process
	{

		struct HCSR_dev *HCSR_devp;
		HCSR_devp=t;
		while(!kthread_should_stop())
		{
		 measure(HCSR_devp);
		 }
		 return 0;

	}
	static ssize_t trigger_show(struct device *dev,struct device_attribute *attr,char *buf) // used to display the trigger pin 
	{     
        struct P_chip *p = dev_get_drvdata(dev);
        return snprintf(buf, PAGE_SIZE, "%d\n",p->HCSR_devp->trig_copy);
        
		}


	static ssize_t trigger_store(struct device *dev,struct device_attribute *attr,const char *buf,size_t count) //Used to set the trigger pin
	{
        int tmp;
        struct P_chip *pc =dev_get_drvdata(dev);
        sscanf(buf, "%d", &tmp);
        
        pc->HCSR_devp->conf.trigPin=tmp;
        pc->HCSR_devp->trig_copy = tmp;
        printk("Trigger Pin has been Set to %d \n",pc->HCSR_devp->conf.trigPin);
        return PAGE_SIZE;
	}

	static ssize_t echo_show(struct device *dev,struct device_attribute *attr,char *buf) // used to display the echo pin
	{     
        struct P_chip *p = dev_get_drvdata(dev);
        return snprintf(buf, PAGE_SIZE, "%d\n",p->HCSR_devp->echo_copy);
        
		}


	static ssize_t echo_store(struct device *dev,struct device_attribute *attr,const char *buf,size_t count) //used to set the echo pin
	{
        int tmp;
        struct P_chip *pc =dev_get_drvdata(dev);
        sscanf(buf, "%d", &tmp);
        
        pc->HCSR_devp->conf.echoPin=tmp;
        pc->HCSR_devp->echo_copy=tmp;
        printk("Echo Pin has been Set to %d \n",pc->HCSR_devp->conf.echoPin);
        return PAGE_SIZE;
	}

	static ssize_t number_samples_show(struct device *dev,struct device_attribute *attr,char *buf) //used to get the number of samples
	{     
        struct P_chip *p = dev_get_drvdata(dev);
        return snprintf(buf, PAGE_SIZE, "%d\n",p->HCSR_devp->conf.m);
        
		}


	static ssize_t number_samples_store(struct device *dev,struct device_attribute *attr,const char *buf,size_t count)  //used to set the number of samples
	{
        int tmp;
        struct P_chip *pc =dev_get_drvdata(dev);
        sscanf(buf, "%d", &tmp);
        
        pc->HCSR_devp->conf.m=tmp;
        printk("Number of samples has been Set to %d \n",pc->HCSR_devp->conf.m);
        return PAGE_SIZE;
	}

	static ssize_t sampling_period_show(struct device *dev,struct device_attribute *attr,char *buf) //used to displays the sampling period
	{     
        struct P_chip *p = dev_get_drvdata(dev);
        return snprintf(buf, PAGE_SIZE, "%d\n",p->HCSR_devp->conf.delta);
        
		}


	static ssize_t sampling_period_store(struct device *dev,struct device_attribute *attr,const char *buf,size_t count) //used to set the sampling period
	{
        int tmp;
        struct P_chip *pc =dev_get_drvdata(dev);
        sscanf(buf, "%d", &tmp);
        
        pc->HCSR_devp->conf.delta=tmp;
        printk("Sampling period has been Set to %d \n",pc->HCSR_devp->conf.delta);
        return PAGE_SIZE;
	}

	static ssize_t  enable_show(struct device *dev,struct device_attribute *attr,char *buf) //used to display the value of enable
	{     
        struct P_chip *p = dev_get_drvdata(dev);
        return snprintf(buf, PAGE_SIZE, "%d\n",p->HCSR_devp->enable);
        
		}


	static ssize_t enable_store(struct device *dev,struct device_attribute *attr,const char *buf,size_t count) // used to set the measuring thread or disabling it
	{
        int tmp;
        struct P_chip *pc =dev_get_drvdata(dev);
        sscanf(buf, "%d", &tmp);
        
        pc->HCSR_devp->enable=tmp;
        if(tmp==1)
        	{printk("measurement_enabled\n ");    

        	 configure_pins(pc->HCSR_devp,pc->HCSR_devp->conf.trigPin,pc->HCSR_devp->conf.echoPin);  // configure pins
        	 configure_parameters(pc->HCSR_devp,pc->HCSR_devp->conf.m,pc->HCSR_devp->conf.delta);  //configure the parameters
        	 pc->HCSR_devp->thread=kthread_run(measure_thread, (void *) pc->HCSR_devp , pc->HCSR_devp->name);//spawn a thread to measure the process


        	}

        if(tmp==0)
        	{printk("Measurement Disabled\n");
        		kthread_stop(pc->HCSR_devp->thread);  //kill the thread
        	}

        
        printk("Enable values is %d \n",tmp);
        return PAGE_SIZE;
	}
	
	static ssize_t  distance_show(struct device *dev,struct device_attribute *attr,char *buf) //used to display the distanc
	{     
        struct P_chip *p = dev_get_drvdata(dev);
        //unsigned long long int* di = distance;// p->HCSR_devp->buff[0].value;     
        return snprintf(buf, PAGE_SIZE, "%lld\n",p->HCSR_devp->latest_distance);                      
	}


	// sets each attribute to its corresponding store and show functions

	static DEVICE_ATTR(trigger, S_IRUSR | S_IWUSR, trigger_show, trigger_store);
	static DEVICE_ATTR(echo, S_IRUSR | S_IWUSR, echo_show, echo_store);
	static DEVICE_ATTR(number_samples, S_IRUSR | S_IWUSR, number_samples_show, number_samples_store);
	static DEVICE_ATTR(sampling_period, S_IRUSR | S_IWUSR, sampling_period_show, sampling_period_store);
	static DEVICE_ATTR(enable, S_IRUSR | S_IWUSR, enable_show, enable_store);
	static DEVICE_ATTR(distance, S_IRUSR , distance_show, NULL);
	
	
	
	int HCSR_driver_init(struct P_chip* p)   //initilize HCSR driver
	{	
		char name_dev[20];
		int retval;
		printk("Number of devices : %d\n",num_devices+1);
		
		/* Registering the Miscellaneous drivers */
		
		
		pchip[num_devices]=p;
		pchip[num_devices]->HCSR_devp = (struct HCSR_dev *)kmalloc(sizeof(struct HCSR_dev), GFP_KERNEL);
		//initializing HCSR structure
		pchip[num_devices]->HCSR_devp->head=0;
		pchip[num_devices]->HCSR_devp->curr=0;
		pchip[num_devices]->HCSR_devp->count=0;
		pchip[num_devices]->HCSR_devp->measurement_flag=0;

		pchip[num_devices]->HCSR_devp->trig_mux1=-1;
		pchip[num_devices]->HCSR_devp->trig_mux2=-1;
		pchip[num_devices]->HCSR_devp->echo_mux1=-1;
		pchip[num_devices]->HCSR_devp->echo_mux2=-1;
		pchip[num_devices]->HCSR_devp->trig_level=-1;
		pchip[num_devices]->HCSR_devp->echo_level=-1;
		pchip[num_devices]->HCSR_devp->irq_num=-1;

		pchip[num_devices]->HCSR_devp->enable=0;
		pchip[num_devices]->HCSR_devp->latest_distance=0;

		hrtimer_init(&hrt,CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		
		if (!pchip[num_devices]->HCSR_devp) 
			{printk("Bad Kmalloc HCSR-DEVP\n"); return -ENOMEM;}
		if(num_devices==0){
		HCSR_class=class_create(THIS_MODULE,CLASS_NAME);
		if(!HCSR_class)
			{printk("Error creating class \n"); return -EFAULT;}
	    }
		// registering misc devices
		pchip[num_devices]->my_misc.minor =MISC_DYNAMIC_MINOR;
		sprintf(name_dev,pchip[num_devices]->name);
		sprintf(pchip[num_devices]->HCSR_devp->name,name_dev);
		pchip[num_devices]->my_misc.name=name_dev;
		pchip[num_devices]->my_misc.fops=&HCSR_fops;
		printk("Registering %s\n", pchip[num_devices]->my_misc.name);
		retval = misc_register(&(pchip[num_devices]->my_misc));
		if(retval <0)
			{printk("Error registering the misc device");
		return -EFAULT;}
		// creating the devices HCSR_0 or HCSR_1
		pchip[num_devices]->HCSR_device = device_create(HCSR_class, NULL, HCSR_dev, NULL, pchip[num_devices]->name);
        if (!pchip[num_devices]->HCSR_device) {
                printk("ERROR Device cannot be created\n");
                return -EFAULT;
        }
        dev_set_drvdata(pchip[num_devices]->HCSR_device,(void *) pchip[num_devices]);
        HCSR_dev+=1;	

        // Creates the trigger attribute
        retval = device_create_file(pchip[num_devices]->HCSR_device, &dev_attr_trigger);
        if (retval < 0) {
                printk("error : Cannot create trigger show attribute\n");
                return -EFAULT;
        }
        //Creates the echo attribute
        retval = device_create_file(pchip[num_devices]->HCSR_device, &dev_attr_echo);
        if (retval < 0) {
                printk("error : Cannot create echo show attribute\n");
                return -EFAULT;
        }
    	//Creates the number of samples of attribute
        retval = device_create_file(pchip[num_devices]->HCSR_device, &dev_attr_number_samples);
        if (retval < 0) {
                printk("error : Cannot create number_samples show attribute\n");
                return -EFAULT;
        }
        // Creates the Sampling period attribute
        retval = device_create_file(pchip[num_devices]->HCSR_device, &dev_attr_sampling_period);
        if (retval < 0) {
                printk("error : Cannot create sampling period show attribute\n");
                return -EFAULT;
        }
        //Creates the enable  attribute
        retval = device_create_file(pchip[num_devices]->HCSR_device, &dev_attr_enable);
        if (retval < 0) {
                printk("error : Cannot create Enable show attribute\n");
                return -EFAULT;
        }
        //Creates the Distance attribute
        retval = device_create_file(pchip[num_devices]->HCSR_device, &dev_attr_distance);
        if (retval < 0) {
                printk("error : Cannot create Distance show attribute\n");
                return -EFAULT;
        }

      
		return 0;
	}
	/* Driver Exit */
	void HCSR_driver_exit(void)
	{
		// Free resources.
		 
		num_devices-=1;
		HCSR_dev-=1;
		if(pchip[num_devices]->HCSR_devp->trig_mux1!=-1)
			gpio_free(pchip[num_devices]->HCSR_devp->trig_mux1);
		if(pchip[num_devices]->HCSR_devp->trig_mux2!=-1)
			gpio_free(pchip[num_devices]->HCSR_devp->trig_mux2);
		if(pchip[num_devices]->HCSR_devp->echo_mux1!=-1)
			gpio_free(pchip[num_devices]->HCSR_devp->echo_mux1);
		if(pchip[num_devices]->HCSR_devp->echo_mux2!=-1)
			gpio_free(pchip[num_devices]->HCSR_devp->echo_mux2);
		if(pchip[num_devices]->HCSR_devp->trig_level!=-1)
			gpio_free(pchip[num_devices]->HCSR_devp->trig_level);
		if(pchip[num_devices]->HCSR_devp->echo_level!=-1)
			gpio_free(pchip[num_devices]->HCSR_devp->echo_level);
		if(pchip[num_devices]->HCSR_devp->irq_num!=-1)
			free_irq(pchip[num_devices]->HCSR_devp->irq_num,NULL);
		kfree(pchip[num_devices]->HCSR_devp);
		


		device_destroy(HCSR_class, HCSR_dev);


		misc_deregister(&(pchip[num_devices]->my_misc));
		if(num_devices==0)
		{class_unregister(HCSR_class);
        class_destroy(HCSR_class);
    hrtimer_cancel( &hrt);}	
	}




static const struct platform_device_id P_id_table[] = {            // Mentions the devices present
         { "HCSR_0", 0 },
         { "HCSR_1", 0 },
        
	 { },
};

static int P_driver_probe(struct platform_device *dev_found)		
{
	struct P_chip *pchip;
	
	pchip = container_of(dev_found, struct P_chip, plf_dev);
	
	printk(KERN_ALERT "Found the device -- %s  %d \n", pchip->name, pchip->dev_no);

	HCSR_driver_init(pchip); 		// initialize and register
	num_devices+=1;
	
			
	return 0;
};

static int P_driver_remove(struct platform_device *pdev)
{
	HCSR_driver_exit();
	return 0;
};

static struct platform_driver P_driver = {
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= P_driver_probe,
	.remove		= P_driver_remove,
	.id_table	= P_id_table,
};

module_platform_driver(P_driver);
MODULE_LICENSE("GPL");




int cmd_table(struct HCSR_dev *HCSR_devp,int pin_1, int pin_2){  // 1: output, 2:input

 

    switch(pin_1){  //trig, output

        case 0:    

            HCSR_devp->conf.trigPin = 11;

            HCSR_devp->trig_level = 32;

            if( gpio_request(HCSR_devp->conf.trigPin, "gpio_out_11") != 0 )  printk("gpio_out_11 error!\n");

            if( gpio_request(HCSR_devp->trig_level, "dir_out_32") != 0 )  printk("dir_out_32 error!\n");

            gpio_direction_output(HCSR_devp->trig_level, 0);

            gpio_direction_output(HCSR_devp->conf.trigPin, 0);

            break;

        case 1:

            HCSR_devp->conf.trigPin = 12;

            HCSR_devp->trig_level = 28;

            HCSR_devp->trig_mux1 = 45;

            if( gpio_request(HCSR_devp->conf.trigPin, "gpio_out_12") != 0 )  printk("gpio_out_12 error!\n");

            if( gpio_request(HCSR_devp->trig_level, "dir_out_28") != 0 )  printk("dir_out_28 error!\n");

            if( gpio_request(HCSR_devp->trig_mux1, "pin_mux_45") != 0 )  printk("pin_mux_45 error!\n");

            gpio_direction_output(HCSR_devp->trig_level, 0);

            gpio_direction_output(HCSR_devp->trig_mux1, 0);

            gpio_direction_output(HCSR_devp->conf.trigPin, 0);

            break;

        case 2:

            HCSR_devp->conf.trigPin = 13;

            HCSR_devp->trig_level = 34;

            HCSR_devp->trig_mux1 = 77;

            if( gpio_request(HCSR_devp->conf.trigPin, "gpio_out_13") != 0 )  printk("gpio_out_13 error!\n");

            if( gpio_request(HCSR_devp->trig_level, "dir_out_34") != 0 )  printk("dir_out_34 error!\n");

            if( gpio_request(HCSR_devp->trig_mux1, "pin_mux_77") != 0 )  printk("pin_mux_77 error!\n");

            gpio_direction_output(HCSR_devp->trig_level, 0);

            gpio_direction_output(HCSR_devp->trig_mux1, 0);

            gpio_direction_output(HCSR_devp->conf.trigPin, 0);

            break;

        case 3:

            HCSR_devp->conf.trigPin = 14;

            HCSR_devp->trig_level = 16;

            HCSR_devp->trig_mux1 = 76;

            HCSR_devp->trig_mux2 = 64;

            if( gpio_request(HCSR_devp->conf.trigPin, "gpio_out_14") != 0 )  printk("gpio_out_14 error!\n");

            if( gpio_request(HCSR_devp->trig_level, "dir_out_16") != 0 )  printk("dir_out_16 error!\n");

            if( gpio_request(HCSR_devp->trig_mux1, "pin_mux_76") != 0 )  printk("pin_mux_76 error!\n");

            if( gpio_request(HCSR_devp->trig_mux2, "pin_mux_64") != 0 )  printk("pin_mux_64 error!\n");

            gpio_direction_output(HCSR_devp->trig_level, 0);

            gpio_direction_output(HCSR_devp->trig_mux1, 0);

            gpio_direction_output(HCSR_devp->trig_mux2, 0);

            gpio_direction_output(HCSR_devp->conf.trigPin, 0);

            break;

        case 4:

            HCSR_devp->conf.trigPin = 6;

            HCSR_devp->trig_level = 36;

            if( gpio_request(HCSR_devp->conf.trigPin, "gpio_out_6") != 0 )  printk("gpio_out_6 error!\n");

            if( gpio_request(HCSR_devp->trig_level, "dir_out_36") != 0 )  printk("dir_out_36 error!\n");

            gpio_direction_output(HCSR_devp->trig_level, 0);

            gpio_direction_output(HCSR_devp->conf.trigPin, 0);

            break;

        case 5:

            HCSR_devp->conf.trigPin = 0;

            HCSR_devp->trig_level = 18;

            HCSR_devp->trig_mux1 = 66;

            if( gpio_request(HCSR_devp->conf.trigPin, "gpio_out_0") != 0 )  printk("gpio_out_0 error!\n");

            if( gpio_request(HCSR_devp->trig_level, "dir_out_18") != 0 )  printk("dir_out_18 error!\n");

            if( gpio_request(HCSR_devp->trig_mux1, "pin_mux_66") != 0 )  printk("pin_mux_66 error!\n");

            gpio_direction_output(HCSR_devp->trig_level, 0);

            gpio_direction_output(HCSR_devp->trig_mux1, 0);

            gpio_direction_output(HCSR_devp->conf.trigPin, 0);

            break;

        case 6:

            HCSR_devp->conf.trigPin = 1;

            HCSR_devp->trig_level = 20;

            HCSR_devp->trig_mux1 = 68;

            if( gpio_request(HCSR_devp->conf.trigPin, "gpio_out_1") != 0 )  printk("gpio_out_1 error!\n");

            if( gpio_request(HCSR_devp->trig_level, "dir_out_20") != 0 )  printk("dir_out_20 error!\n");

            if( gpio_request(HCSR_devp->trig_mux1, "pin_mux_68") != 0 )  printk("pin_mux_68 error!\n");

            gpio_direction_output(HCSR_devp->trig_level, 0);

            gpio_direction_output(HCSR_devp->trig_mux1, 0);

            gpio_direction_output(HCSR_devp->conf.trigPin, 0);

            break;

        case 7:

            HCSR_devp->conf.trigPin = 38;

            if( gpio_request(HCSR_devp->conf.trigPin, "gpio_out_38") != 0 )  printk("gpio_out_38 error!\n");

            gpio_direction_output(HCSR_devp->conf.trigPin, 0);

            break;

        case 8:

            HCSR_devp->conf.trigPin = 40;

            if( gpio_request(HCSR_devp->conf.trigPin, "gpio_out_40") != 0 )  printk("gpio_out_40 error!\n");

            gpio_direction_output(HCSR_devp->conf.trigPin, 0);

            break;

 

        case 9:

            HCSR_devp->conf.trigPin = 4;

            HCSR_devp->trig_level = 22;

            HCSR_devp->trig_mux1 = 70;

            if( gpio_request(HCSR_devp->conf.trigPin, "gpio_out_4") != 0 )  printk("gpio_out_4 error!\n");

            if( gpio_request(HCSR_devp->trig_level, "dir_out_22") != 0 )  printk("dir_out_22 error!\n");

            if( gpio_request(HCSR_devp->trig_mux1, "pin_mux_70") != 0 )  printk("pin_mux_70 error!\n");

            gpio_direction_output(HCSR_devp->trig_level, 0);

            gpio_direction_output(HCSR_devp->trig_mux1, 0);

            gpio_direction_output(HCSR_devp->conf.trigPin, 0);

            break;

        case 10:

            HCSR_devp->conf.trigPin = 10;

            HCSR_devp->trig_level = 26;

            HCSR_devp->trig_mux1 = 74;

            if( gpio_request(HCSR_devp->conf.trigPin, "gpio_out_10") != 0 )  printk("gpio_out_10 error!\n");

            if( gpio_request(HCSR_devp->trig_level, "dir_out_26") != 0 )  printk("dir_out_26 error!\n");

            if( gpio_request(HCSR_devp->trig_mux1, "pin_mux_74") != 0 )  printk("pin_mux_74 error!\n");

            gpio_direction_output(HCSR_devp->trig_level, 0);

            gpio_direction_output(HCSR_devp->trig_mux1, 0);

            gpio_direction_output(HCSR_devp->conf.trigPin, 0);

            break;

        case 11:

            HCSR_devp->conf.trigPin = 5;

            HCSR_devp->trig_level = 24;

            HCSR_devp->trig_mux1 = 44;

            HCSR_devp->trig_mux2 = 72;

            if( gpio_request(HCSR_devp->conf.trigPin, "gpio_out_5") != 0 )  printk("gpio_out_5 error!\n");

            if( gpio_request(HCSR_devp->trig_level, "dir_out_24") != 0 )  printk("dir_out_24 error!\n");

            if( gpio_request(HCSR_devp->trig_mux1, "pin_mux_44") != 0 )  printk("pin_mux_44 error!\n");

            if( gpio_request(HCSR_devp->trig_mux2, "pin_mux_72") != 0 )  printk("pin_mux_72 error!\n");

            gpio_direction_output(HCSR_devp->trig_level, 0);

            gpio_direction_output(HCSR_devp->trig_mux1, 0);

            gpio_direction_output(HCSR_devp->trig_mux2, 0);

            gpio_direction_output(HCSR_devp->conf.trigPin, 0);

            break;

        case 12:

            HCSR_devp->conf.trigPin = 15;

            HCSR_devp->trig_level = 42;

            if( gpio_request(HCSR_devp->conf.trigPin, "gpio_out_15") != 0 )  printk("gpio_out_15 error!\n");

            if( gpio_request(HCSR_devp->trig_level, "dir_out_42") != 0 )  printk("dir_out_42 error!\n");

            gpio_direction_output(HCSR_devp->trig_level, 0);

            gpio_direction_output(HCSR_devp->conf.trigPin, 0);

            gpio_set_value_cansleep(42, 0);

            break;

        case 13:

            HCSR_devp->conf.trigPin = 7;

            HCSR_devp->trig_level = 30;

            HCSR_devp->trig_mux1 = 46;

            if( gpio_request(HCSR_devp->conf.trigPin, "gpio_out_7") != 0 )  printk("gpio_out_7 error!\n");

            if( gpio_request(HCSR_devp->trig_level, "dir_out_30") != 0 )  printk("dir_out_30 error!\n");

            if( gpio_request(HCSR_devp->trig_mux1, "pin_mux_46") != 0 )  printk("pin_mux_46 error!\n");

            gpio_direction_output(HCSR_devp->trig_level, 0);

            gpio_direction_output(HCSR_devp->conf.trigPin, 0);

            gpio_direction_output(HCSR_devp->trig_mux1, 0);

            gpio_set_value_cansleep(30, 0);

            gpio_set_value_cansleep(46, 0);

            break;

        case 14:

            HCSR_devp->conf.trigPin = 48;

            if( gpio_request(HCSR_devp->conf.trigPin, "gpio_out_48") != 0 )  printk("gpio_out_48 error!\n");

            gpio_direction_output(HCSR_devp->conf.trigPin, 0);

            break;

        case 15:

            HCSR_devp->conf.trigPin = 50;

            if( gpio_request(HCSR_devp->conf.trigPin, "gpio_out_50") != 0 )  printk("gpio_out_50 error!\n");

            gpio_direction_output(HCSR_devp->conf.trigPin, 0);

            break;

        case 16:

            HCSR_devp->conf.trigPin = 52;

            if( gpio_request(HCSR_devp->conf.trigPin, "gpio_out_52") != 0 )  printk("gpio_out_52 error!\n");

            gpio_direction_output(HCSR_devp->conf.trigPin, 0);

            break;

        case 17:

            HCSR_devp->conf.trigPin = 54;

            if( gpio_request(HCSR_devp->conf.trigPin, "gpio_out_54") != 0 )  printk("gpio_out_54 error!\n");

            gpio_direction_output(HCSR_devp->conf.trigPin, 0);

            break;

        case 18:

            HCSR_devp->conf.trigPin = 56;

            HCSR_devp->trig_mux1 = 60;

            HCSR_devp->trig_mux2 = 78;

            if( gpio_request(HCSR_devp->conf.trigPin, "gpio_out_56") != 0 )  printk("gpio_out_56 error!\n");

            if( gpio_request(HCSR_devp->trig_mux1, "pin_mux_60") != 0 )  printk("pin_mux_60 error!\n");

            if( gpio_request(HCSR_devp->trig_mux2, "pin_mux_78") != 0 )  printk("pin_mux_78 error!\n");

            gpio_direction_output(HCSR_devp->trig_mux1, 0);

            gpio_direction_output(HCSR_devp->trig_mux2, 0);

            gpio_direction_output(HCSR_devp->conf.trigPin, 0);

            break;

        case 19:

            HCSR_devp->conf.trigPin = 58;

            HCSR_devp->trig_mux1 = 60;

            HCSR_devp->trig_mux2 = 79;

            if( gpio_request(HCSR_devp->conf.trigPin, "gpio_out_58") != 0 )  printk("gpio_out_58 error!\n");

            if( gpio_request(HCSR_devp->trig_mux1, "pin_mux_60") != 0 )  printk("pin_mux_60 error!\n");

            if( gpio_request(HCSR_devp->trig_mux2, "pin_mux_79") != 0 )  printk("pin_mux_79 error!\n");

            gpio_direction_output(HCSR_devp->trig_mux1, 0);

            gpio_direction_output(HCSR_devp->trig_mux2, 0);

            gpio_direction_output(HCSR_devp->conf.trigPin, 0);

            break;

    }

 

    switch(pin_2){  //echo, input

    int ret;

        case 0:    

            HCSR_devp->conf.echoPin = 11;

            HCSR_devp->echo_level = 32;

            if( gpio_request(11, "gpio_in_11") != 0 )  printk("gpio_in_11 error!\n");

            if( gpio_request(32, "dir_in_32") != 0 )  printk("dir_in_32 error!\n");

            gpio_direction_output(HCSR_devp->echo_level,1);

            HCSR_devp->irq_num = gpio_to_irq(HCSR_devp->conf.echoPin);

            if(HCSR_devp->irq_num<0)  {printk("IRQ NUMBER ERROR\n");return -1;}

            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void *) HCSR_devp);

            if (ret < 0)   {printk("in request_irq\n");return -1;}

            gpio_direction_input(HCSR_devp->conf.echoPin);

            break;

        case 1:

            HCSR_devp->conf.echoPin = 12;

            HCSR_devp->echo_level = 28;

            HCSR_devp->echo_mux1 = 45;

            if( gpio_request(12, "gpio_in_12") != 0 )  printk("gpio_in_12 error!\n");

            if( gpio_request(28, "dir_in_28") != 0 )  printk("dir_in_28 error!\n");

            if( gpio_request(45, "pin_mux_45") != 0 )  printk("pin_mux_45 error!\n");

            gpio_direction_output(HCSR_devp->echo_level,1);

            gpio_direction_output(HCSR_devp->echo_mux1,0);

            HCSR_devp->irq_num = gpio_to_irq(HCSR_devp->conf.echoPin);

            if(HCSR_devp->irq_num<0)  {printk("IRQ NUMBER ERROR\n");return -1;}

            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void *) HCSR_devp);

            if (ret < 0)   {printk("in request_irq\n");return -1;}

            gpio_direction_input(HCSR_devp->conf.echoPin);

            break;

        case 2:

            HCSR_devp->conf.echoPin = 13;

            HCSR_devp->echo_level = 34;

            HCSR_devp->echo_mux1 = 77;

            if( gpio_request(HCSR_devp->conf.echoPin, "gpio_in_13") != 0 )  printk("gpio_in_13 error!\n");

            if( gpio_request(HCSR_devp->echo_level, "dir_in_34") != 0 )  printk("dir_in_34 error!\n");

            if( gpio_request(HCSR_devp->echo_mux1, "pin_mux_77") != 0 )  printk("pin_mux_77 error!\n");

            gpio_direction_output(HCSR_devp->echo_level,1);

            gpio_direction_output(HCSR_devp->echo_mux1,0);

            HCSR_devp->irq_num = gpio_to_irq(HCSR_devp->conf.echoPin);

            if(HCSR_devp->irq_num<0)  {printk("IRQ NUMBER ERROR\n");return -1;}

            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void *) HCSR_devp);

            if (ret < 0)   {printk("in request_irq\n");return -1;}

            gpio_direction_input(HCSR_devp->conf.echoPin);

            break;

        case 3:

            HCSR_devp->conf.echoPin = 14;

            HCSR_devp->echo_level = 16;

            HCSR_devp->echo_mux1 = 76;

            HCSR_devp->echo_mux2 = 64;

            if( gpio_request(HCSR_devp->conf.echoPin, "gpio_in_14") != 0 )  printk("gpio_in_14 error!\n");

            if( gpio_request(HCSR_devp->echo_level, "dir_in_16") != 0 )  printk("dir_in_16 error!\n");

            if( gpio_request(HCSR_devp->echo_mux1, "pin_mux_76") != 0 )  printk("pin_mux_76 error!\n");

            if( gpio_request(HCSR_devp->echo_mux2, "pin_mux_64") != 0 )  printk("pin_mux_64 error!\n");

            gpio_direction_output(HCSR_devp->echo_level,1);

            gpio_direction_output(HCSR_devp->echo_mux1,0);

            gpio_direction_output(HCSR_devp->echo_mux2,0);

            HCSR_devp->irq_num = gpio_to_irq(HCSR_devp->conf.echoPin);

            if(HCSR_devp->irq_num<0)  {printk("IRQ NUMBER ERROR\n");return -1;}

            ret = request_irq(HCSR_devp->irq_num, (irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void *) HCSR_devp);

            if (ret < 0)   {printk("in request_irq\n");return -1;}

            gpio_direction_input(HCSR_devp->conf.echoPin);

            break;

        case 4:

            HCSR_devp->conf.echoPin=6;

        HCSR_devp->echo_level=36;

            if( gpio_request(HCSR_devp->conf.echoPin, "gpio_in_6") != 0 )  printk("gpio_in_6 error!\n");

            if( gpio_request(HCSR_devp->echo_level, "dir_in_36") != 0 )  printk("dir_in_36 error!\n");

            gpio_direction_output(HCSR_devp->echo_level,1);

            HCSR_devp->irq_num= gpio_to_irq(HCSR_devp->conf.echoPin);

if(HCSR_devp->irq_num<0) {printk("IRQ NUMBER ERROR\n");return -1;}

ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void *) HCSR_devp);

if (ret < 0) {printk("in request_irq\n");return -1;}

gpio_direction_input(HCSR_devp->conf.echoPin);

            break;

        case 5:

            HCSR_devp->conf.echoPin=0;

        HCSR_devp->echo_mux1=66;

        HCSR_devp->echo_level=18;

            if( gpio_request(HCSR_devp->conf.echoPin, "gpio_in_0") != 0 )  printk("gpio_in_0 error!\n");

            if( gpio_request(HCSR_devp->echo_level, "dir_in_18") != 0 )  printk("dir_in_18 error!\n");

            if( gpio_request(HCSR_devp->echo_mux1, "pin_mux_66") != 0 )  printk("pin_mux_66 error!\n");

            gpio_direction_output(HCSR_devp->echo_level,1);

            gpio_direction_output(HCSR_devp->echo_mux1,0);

            HCSR_devp->irq_num = gpio_to_irq(HCSR_devp->conf.echoPin);

            if(HCSR_devp->irq_num<0)  {printk("IRQ NUMBER ERROR\n"); return -1;}

            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void *) HCSR_devp);

            if (ret < 0)   {printk("in request_irq\n");return -1;}

            gpio_direction_input(HCSR_devp->conf.echoPin);

            break;

            

        case 6:

            HCSR_devp->conf.echoPin = 1;

            HCSR_devp->echo_level = 20;

            HCSR_devp->echo_mux1 = 68;

            if( gpio_request(HCSR_devp->conf.echoPin, "gpio_in_1") != 0 )  printk("gpio_in_1 error!\n");

            if( gpio_request(HCSR_devp->echo_level, "dir_in_20") != 0 )  printk("dir_in_20 error!\n");

            if( gpio_request(HCSR_devp->echo_mux1, "pin_mux_68") != 0 )  printk("pin_mux_68 error!\n");

            gpio_direction_output(HCSR_devp->echo_level,1);

            gpio_direction_output(HCSR_devp->echo_mux1,0);

            HCSR_devp->irq_num = gpio_to_irq(HCSR_devp->conf.echoPin);

            if(HCSR_devp->irq_num<0)  {printk("IRQ NUMBER ERROR\n");return -1;}

            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void *) HCSR_devp);

            if (ret < 0)   {printk("in request_irq\n");return -1;}

            gpio_direction_input(HCSR_devp->conf.echoPin);

            break;

        case 7:

            HCSR_devp->conf.echoPin = 38;

            if( gpio_request(HCSR_devp->conf.echoPin, "gpio_in_38") != 0 )  printk("gpio_in_38 error!\n");

            HCSR_devp->irq_num = gpio_to_irq(HCSR_devp->conf.echoPin);

            if(HCSR_devp->irq_num<0)  {printk("IRQ NUMBER ERROR\n");return -1;}

            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void *) HCSR_devp);

            if (ret < 0)   {printk("in request_irq\n");return -1;}

            gpio_direction_input(HCSR_devp->conf.echoPin);

            break;

        case 8:

            HCSR_devp->conf.echoPin = 40;

            if( gpio_request(HCSR_devp->conf.echoPin, "gpio_in_40") != 0 )  printk("gpio_in_40 error!\n");

            HCSR_devp->irq_num = gpio_to_irq(HCSR_devp->conf.echoPin);

            if(HCSR_devp->irq_num<0)  {printk("IRQ NUMBER ERROR\n");return -1;}

            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void *) HCSR_devp);

            if (ret < 0)   {printk("in request_irq\n");return -1;}

            gpio_direction_input(HCSR_devp->conf.echoPin);

            break;

        case 9:

            HCSR_devp->conf.echoPin = 4;

            HCSR_devp->echo_level = 22;

            HCSR_devp->echo_mux1 = 70;

            if( gpio_request(HCSR_devp->conf.echoPin, "gpio_in_4") != 0 )  printk("gpio_in_4 error!\n");

            if( gpio_request(HCSR_devp->echo_level, "dir_in_22") != 0 )  printk("dir_in_22 error!\n");

            if( gpio_request(HCSR_devp->echo_mux1, "pin_mux_70") != 0 )  printk("pin_mux_70 error!\n");

            gpio_direction_output(HCSR_devp->echo_level,1);

            gpio_direction_output(HCSR_devp->echo_mux1,0);

            HCSR_devp->irq_num = gpio_to_irq(HCSR_devp->conf.echoPin);

            if(HCSR_devp->irq_num<0)  {printk("IRQ NUMBER ERROR\n");return -1;}

            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void *) HCSR_devp);

            if (ret < 0)   {printk("in request_irq\n");return -1;}

            gpio_direction_input(HCSR_devp->conf.echoPin);

            break;

        case 10:

            HCSR_devp->conf.echoPin = 10;

            HCSR_devp->echo_level = 26;

            HCSR_devp->echo_mux1 = 74;

            if( gpio_request(HCSR_devp->conf.echoPin, "gpio_in_10") != 0 )  printk("gpio_in_10 error!\n");

            if( gpio_request(HCSR_devp->echo_level, "dir_in_26") != 0 )  printk("dir_in_26 error!\n");

            if( gpio_request(HCSR_devp->echo_mux1, "pin_mux_74") != 0 )  printk("pin_mux_74 error!\n");

            gpio_direction_output(HCSR_devp->echo_level,1);

            gpio_direction_output(HCSR_devp->echo_mux1,0);

            HCSR_devp->irq_num = gpio_to_irq(HCSR_devp->conf.echoPin);

            if(HCSR_devp->irq_num<0)  {printk("IRQ NUMBER ERROR\n");return -1;}

            ret = request_irq(HCSR_devp->irq_num, (irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void *) HCSR_devp);

            if (ret < 0)   {printk("in request_irq\n");return -1;}

            gpio_direction_input(HCSR_devp->conf.echoPin);

            break;

        case 11:

            HCSR_devp->conf.echoPin = 5;

            HCSR_devp->echo_level = 24;

            HCSR_devp->echo_mux1 = 44;

            HCSR_devp->echo_mux2 = 72;

            if( gpio_request(HCSR_devp->conf.echoPin, "gpio_in_5") != 0 )  printk("gpio_in_5 error!\n");

            if( gpio_request(HCSR_devp->echo_level, "dir_in_24") != 0 )  printk("dir_in_24 error!\n");

            if( gpio_request(HCSR_devp->echo_mux1, "pin_mux_44") != 0 )  printk("pin_mux_44 error!\n");

            if( gpio_request(HCSR_devp->echo_mux2, "pin_mux_72") != 0 )  printk("pin_mux_72 error!\n");

            gpio_direction_output(HCSR_devp->echo_level,1);

            gpio_direction_output(HCSR_devp->echo_mux1,0);

            gpio_direction_output(HCSR_devp->echo_mux2,0);

            HCSR_devp->irq_num = gpio_to_irq(HCSR_devp->conf.echoPin);

            if(HCSR_devp->irq_num<0)  {printk("IRQ NUMBER ERROR\n");return -1;}

            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void *) HCSR_devp);

            if (ret < 0)   {printk("in request_irq\n");return -1;}

            gpio_direction_input(HCSR_devp->conf.echoPin);

            break;

        case 12:

            HCSR_devp->conf.echoPin = 15;

            HCSR_devp->echo_level = 42;

            if( gpio_request(HCSR_devp->conf.echoPin, "gpio_in_15") != 0 )  printk("gpio_in_15 error!\n");

            if( gpio_request(HCSR_devp->echo_level, "dir_in_42") != 0 )  printk("dir_in_42 error!\n");

            gpio_direction_output(HCSR_devp->echo_level,1);

            HCSR_devp->irq_num = gpio_to_irq(HCSR_devp->conf.echoPin);

            if(HCSR_devp->irq_num<0)  {printk("IRQ NUMBER ERROR\n");return -1;}

            ret = request_irq(HCSR_devp->irq_num, (irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", 0);

            if (ret < 0)   {printk("in request_irq\n");return -1;}

            gpio_direction_input(HCSR_devp->conf.echoPin);

            break;

        case 13:

            HCSR_devp->conf.echoPin = 7;

            HCSR_devp->echo_level = 30;

            HCSR_devp->echo_mux1 = 46;

            if( gpio_request(HCSR_devp->conf.echoPin, "gpio_in_7") != 0 )  printk("gpio_in_7 error!\n");

            if( gpio_request(HCSR_devp->echo_level, "dir_in_30") != 0 )  printk("dir_in_30 error!\n");

            if( gpio_request(HCSR_devp->echo_mux1, "pin_mux_46") != 0 )  printk("pin_mux_46 error!\n");

            gpio_direction_output(HCSR_devp->echo_level,1);

            gpio_direction_output(HCSR_devp->echo_mux1,0);

            HCSR_devp->irq_num = gpio_to_irq(HCSR_devp->conf.echoPin);

            if(HCSR_devp->irq_num<0)  {printk("IRQ NUMBER ERROR\n");return -1;}

            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void *) HCSR_devp);

            if (ret < 0)   {printk("in request_irq\n");return -1;}

            gpio_direction_input(HCSR_devp->conf.echoPin);

            break;

        case 14:

            HCSR_devp->conf.echoPin = 48;

            if( gpio_request(HCSR_devp->conf.echoPin, "gpio_in_48") != 0 )  printk("gpio_in_48 error!\n");

            HCSR_devp->irq_num = gpio_to_irq(HCSR_devp->conf.echoPin);

            if(HCSR_devp->irq_num<0)  {printk("IRQ NUMBER ERROR\n");return -1;}

            ret = request_irq(HCSR_devp->irq_num, (irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void *) HCSR_devp);

            if (ret < 0)   printk("Error in request_irq\n");

            gpio_direction_input(HCSR_devp->conf.echoPin);

            break;

        case 15:

            HCSR_devp->conf.echoPin = 50;

            if( gpio_request(HCSR_devp->conf.echoPin, "gpio_in_50") != 0 )  printk("gpio_in_50 error!\n");

            HCSR_devp->irq_num = gpio_to_irq(HCSR_devp->conf.echoPin);

            if(HCSR_devp->irq_num<0)  {printk("IRQ NUMBER ERROR\n");return -1;}

            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void *) HCSR_devp);

            if (ret < 0)   {printk("in request_irq\n");return -1;}

            gpio_direction_input(HCSR_devp->conf.echoPin);

            break;

        case 16:

            HCSR_devp->conf.echoPin = 52;

            if( gpio_request(HCSR_devp->conf.echoPin, "gpio_in_52") != 0 )  printk("gpio_in_52 error!\n");

            HCSR_devp->irq_num = gpio_to_irq(HCSR_devp->conf.echoPin);

            if(HCSR_devp->irq_num<0)  {printk("IRQ NUMBER ERROR\n");return -1;}

            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void *) HCSR_devp);

            if (ret < 0)   {printk("in request_irq\n");return -1;}

            gpio_direction_input(HCSR_devp->conf.echoPin);

            break;

        case 17:

            HCSR_devp->conf.echoPin = 54;

            if( gpio_request(HCSR_devp->conf.echoPin, "gpio_in_54") != 0 )  printk("gpio_in_54 error!\n");

            HCSR_devp->irq_num = gpio_to_irq(HCSR_devp->conf.echoPin);

            if(HCSR_devp->irq_num<0)  {printk("IRQ NUMBER ERROR\n");return -1;}

            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void *) HCSR_devp);

            if (ret < 0)   {printk("in request_irq\n");return -1;}

            gpio_direction_input(HCSR_devp->conf.echoPin);

            break;

        case 18:

            HCSR_devp->conf.echoPin = 56;

            HCSR_devp->echo_mux1 = 60;

            HCSR_devp->echo_mux2 = 78;

            if( gpio_request(HCSR_devp->conf.echoPin, "gpio_in_56") != 0 )  printk("gpio_in_56 error!\n");

            if( gpio_request(HCSR_devp->echo_mux1, "pin_mux_60") != 0 )  printk("pin_mux_60 error!\n");

            if( gpio_request(HCSR_devp->echo_mux2, "pin_mux_78") != 0 )  printk("pin_mux_78 error!\n");

            gpio_direction_output(HCSR_devp->echo_mux1,1);

            gpio_direction_output(HCSR_devp->echo_mux2,1);

            HCSR_devp->irq_num = gpio_to_irq(HCSR_devp->conf.echoPin);

            if(HCSR_devp->irq_num<0)  {printk("IRQ NUMBER ERROR\n");return -1;}

            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void *) HCSR_devp);

            if (ret < 0)   {printk("in request_irq\n");return -1;}

            gpio_direction_input(HCSR_devp->conf.echoPin);

            break;

        case 19:

            HCSR_devp->conf.echoPin = 58;

            HCSR_devp->echo_mux1 = 60;

            HCSR_devp->echo_mux2 = 79;

            if( gpio_request(HCSR_devp->conf.echoPin, "gpio_in_58") != 0 )  printk("gpio_in_58 error!\n");

            if( gpio_request(HCSR_devp->echo_mux1, "pin_mux_60") != 0 )  printk("pin_mux_60 error!\n");

            if( gpio_request(HCSR_devp->echo_mux2, "pin_mux_79") != 0 )  printk("pin_mux_79 error!\n");

            gpio_direction_output(HCSR_devp->echo_mux1,1);

            gpio_direction_output(HCSR_devp->echo_mux2,1);

            HCSR_devp->irq_num = gpio_to_irq(HCSR_devp->conf.echoPin);

            if(HCSR_devp->irq_num<0)  {printk("IRQ NUMBER ERROR\n");return -1;}

            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void *) HCSR_devp);

            if (ret < 0)   {printk("in request_irq\n");return -1;}

            gpio_direction_input(HCSR_devp->conf.echoPin);

            break;

    }

}


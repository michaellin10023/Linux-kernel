#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>

#ifndef __PLATFORM_H__


  
#define __PLATFORM_H__
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
    int trig_mux1;
    int trig_mux2;
    int echo_mux1;
    int echo_mux2;
    int trig_level;
    int echo_level;
    int irq_num;
    int trig_copy;
    int echo_copy;
    int enable;
    unsigned long long int latest_distance;
    int measurement_flag;
    struct task_struct *thread;
} *HCSR_devp;



struct P_chip {
		char 			*name;
		int			dev_no;
		struct platform_device 	plf_dev;
		struct HCSR_dev *HCSR_devp;
	    struct device *HCSR_device;
	    struct miscdevice my_misc;
};



#endif /* __GPIO_FUNC_H__ */

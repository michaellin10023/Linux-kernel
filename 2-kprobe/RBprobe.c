
#include<linux/version.h>

#include<linux/init.h>
//#include<asm/kprobes.h>
//#include <asm/ptrace.h>

#include <linux/kprobes.h>
#include <linux/ptrace.h>
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
#include <asm/tsc.h>
#include<linux/init.h>
#include<linux/moduleparam.h>
#include <linux/sched.h>
#include <asm/current.h>
#define DEVICE_NAME                 "RBprobe"  // device name to be created and registered

#if defined(__i386__)

// function definition for reading the X86 TSC
static __inline__ unsigned long long rdtsc(void)
{
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


/* per device structure */
struct RBprobe_dev {
	struct cdev cdev;               /* The cdev structure */
	char name[20];                  /* Name of device*/
	char in_string[256];			/* buffer for the input string */
	int current_write_pointer;
} *RBprobe_devp;


static dev_t RBprobe_dev_number;        /* Allotted device number */
struct class *RBprobe_dev_class;          /* Tie with the device model */
static struct device *RBprobe_dev_device;
static struct kprobe kp;

int RBprobe_driver_open(struct inode *inode, struct file *file)
{
	struct RBprobe_dev *RBprobe_devp;
//	printk("\nopening\n");

	/* Get the per-device structure that contains this cdev */
	RBprobe_devp = container_of(inode->i_cdev, struct RBprobe_dev, cdev);


	/* Easy access to cmos_devp from rest of the entry points */
	file->private_data = RBprobe_devp;
	//printk("\n%s is openning \n", DEVICE_NAME);
	return 0;
}


int RBprobe_driver_release(struct inode *inode, struct file *file)
{
	struct RBprobe_dev *RBprobe_devp = file->private_data;
	
	//printk("\n%s is closing\n", RBprobe_devp->name);
	
	return 0;
}

/*
 write: to register or unregister a kprobe. The location (offset) of the kprobe is passed in the buffer
*buf along with an integer flag. A kprobe is registered if the flag is 1, or unregistered if 0.
*/
ssize_t RBprobe_driver_write(struct file *file,const char *buf,ssize_t count)   //const struct probe_data *tmp)
{
	struct RBprobe_dev *RBprobe_devp = file->private_data;
	
	char flag;
	//int count=*cnt;
	char in_string[256];

	int current_write_pointer=0;   
	int stl=count;
	while (count) {	
		get_user(in_string[current_write_pointer], buf++);  // read the data from user space
		count--;
		if(count){
			current_write_pointer++;
			if( current_write_pointer == 256)
				current_write_pointer = 0;
		}
	}
	flag=in_string[0];
	int i=0;
	for(i=0;i<stl-1;i++)
	in_string[i]=in_string[i+1];	
	if(flag=='1')   
	{
		printk("Registering the Kprobe\n");
		printk("%lu\n",kallsyms_lookup_name(in_string));
		kp.addr = (kprobe_opcode_t *)kallsyms_lookup_name(in_string);
		register_kprobe(&kp);
	}
	if(flag=='0')
	{
	//printk("Unregistering the Kprobe\n");
	unregister_kprobe(&kp);
	//printk("module removed\n ");
	}
	return 0;
}


static struct data_send {   // the data structure including: the address of the kprobe, 
                            //the pid of the running process that hits the probe, time stamp (x86TSC),
	unsigned long int kp_addr;
	unsigned long long int tsc;
	int pid;
}*data_buffer;
int is_empty=1;  //is_empty check if the buffer is empty

/*read: to retrieve the trace data items collected in a probe hit and saved in the buffer. If the buffer
is empty, -1 is returned and errno is set to EINVAL*/

ssize_t RBprobe_driver_read(struct file *file, struct data_send* b)
{

	//printk("Reading --\n");
	if(is_empty==1)
		return -EINVAL;
	else
	copy_to_user(b,data_buffer,sizeof(struct data_send));
	/* 
	 * Most read functions return the number of bytes put into the buffer
	 */
	return 0;

}

static struct file_operations RBprobe_fops = {
    .owner		= THIS_MODULE,           /* Owner */
    .open		= RBprobe_driver_open,        /* Open method */
    .release		= RBprobe_driver_release,     /* Release method */
    .write		= RBprobe_driver_write,       /* Write method */
    .read		= RBprobe_driver_read,        /* Read method */
};

//**********************************************************************************************



int Pre_Handler(struct kprobe *p, struct pt_regs *regs){
    //printk("Pre_Handler:\n");
    //printk(KERN_INFO "pre_handler: p->addr = 0x%u\n ",p->addr);
    //unsigned int *t =(kprobe_opcode_t *)kallsyms_lookup_name("tsc_khz");
    unsigned long long int x=rdtsc();
    data_buffer->kp_addr=p->addr;
    data_buffer->tsc=x;
    char s[50];
    sprintf(s,"%llu",x);
    //printk(KERN_INFO "\n %s \n",s);
    //printk("Timestamp = %llx \n",x);
    return 0;
}


void Post_Handler(struct kprobe *p, struct pt_regs *regs, unsigned long flags){
    //printk("Post_Handler is ");
	struct task_struct *task=get_current();
//(kprobe_opcode_t *)kallsyms_lookup_name("current_task");
	read_lock(&tasklist_lock);
	int tmp= task->pid;
	data_buffer->pid=tmp;
	//for_each_process(task) {
		char s[50];
    		sprintf(s,"%x",tmp);
		//printk(KERN_INFO "pid =%s \n ", s);
		//printk(KERN_INFO "pid = %x \n",tmp);
	//}
	is_empty=0;
	read_unlock(&tasklist_lock);
}	


 

 
int myinit(void)
{
	//printk("module inserted\n ");
	kp.pre_handler = Pre_Handler;
	kp.post_handler = Post_Handler;
	
	data_buffer=(struct data_send*)kmalloc(sizeof(struct data_send),GFP_KERNEL) ;
	int ret;
	int time_since_boot;
	
	/* Request dynamic allocation of a device major number */
	if (alloc_chrdev_region(&RBprobe_dev_number, 0, 1, DEVICE_NAME) < 0) {
			printk(KERN_DEBUG "Can't register device\n"); return -1;
	}

	/* Populate sysfs entries */
	RBprobe_dev_class = class_create(THIS_MODULE, DEVICE_NAME);

	/* Allocate memory for the per-device structure */
	RBprobe_devp = kmalloc(sizeof(struct RBprobe_dev), GFP_KERNEL);
	
	if (!RBprobe_devp) {
		printk("Bad Kmalloc\n"); return -ENOMEM;
	}

	/* Request I/O region */
	sprintf(RBprobe_devp->name, DEVICE_NAME);

	/* Connect the file operations with the cdev */
	cdev_init(&RBprobe_devp->cdev, &RBprobe_fops);
	RBprobe_devp->cdev.owner = THIS_MODULE;

	/* Connect the major/minor number to the cdev */
	ret = cdev_add(&RBprobe_devp->cdev, (RBprobe_dev_number), 1);

	if (ret) {
		printk("Bad cdev\n");
		return ret;
	}

	/* Send uevents to udev, so it'll create /dev nodes */
	RBprobe_dev_device = device_create(RBprobe_dev_class, NULL, MKDEV(MAJOR(RBprobe_dev_number), 0), NULL, DEVICE_NAME);		
	// device_create_file(RBprobe_dev_device, &dev_attr_xxx);

	//memset(RBprobe_devp->in_string, 0, 256);

	//time_since_boot=(jiffies-INITIAL_JIFFIES)/HZ;//since on some systems jiffies is a very huge uninitialized value at boot and saved.
	//sprintf(RBprobe_devp->in_string, "Hi %s, this machine has been on for %d seconds", user_name, time_since_boot);

	//	RBprobe_devp->current_write_pointer = 0;

	//printk("RBprobe driver initialized.\n");// '%s'\n",RBprobe_devp->in_string);
	return 0;

}
 
void myexit(void)
{   // device_remove_file(RBprobe_dev_device, &dev_attr_xxx);
	/* Release the major number */
	unregister_chrdev_region((RBprobe_dev_number), 1);

	/* Destroy device */
	device_destroy (RBprobe_dev_class, MKDEV(MAJOR(RBprobe_dev_number), 0));
	cdev_del(&RBprobe_devp->cdev);
	kfree(RBprobe_devp);
	kfree(data_buffer);
	/* Destroy driver_class */
	class_destroy(RBprobe_dev_class);
	unregister_kprobe(&kp);
	//printk("RBprobe driver removed.\n");
	
}
 
module_init(myinit);
module_exit(myexit);
MODULE_LICENSE("GPL");

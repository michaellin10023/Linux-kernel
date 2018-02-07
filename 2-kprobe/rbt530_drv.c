/*
CSE 530 Embedded Operating System Internal -- Fall 2017
Assignment 1. RB Tree and Dynamic Probe in Linux Kernel (200 points)
	  Part 1: Accessing a kernel RB tree via device file interface
Edited by: Kausic ????last name please, Beibei Liu (team 20) 
Created: 09/18/2017
Last edited: 09/22/2017
Emails: ?????

============================== rbt530_drv ==============================

*/

#include <linux/rbtree.h>
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
#include <linux/ioctl.h>
#include<linux/init.h>
#include<linux/moduleparam.h>
#include<asm/errno.h>
#include<asm-generic/ioctl.h>

#define DEVICE_NAME      "rbt_530"       /* device name to be created and registered */

/* per node structure */
int READ_BREAKPOINT=0;
int WRITE_BREAKPOINT=0;
struct values{
	int key;
	int data;
}val;

/* the object to be added to or removed from the RB tree */

struct rb_object{
	
	struct rb_node node;  
	
	/* with components key and value */
	struct values val;
}rb_object_t;

/* per device structure */

struct rbt_dev {
	/* the cdev structure */
	struct cdev cdev;   
	
	/* structure of the root for the RB tree */
	struct rb_root root;
} *rbt_devp;

/* set_end: command to set which object is to be read. If the argument is 0, read calls retrieve the first object in the tree.
If it is 1, read calls get the last object. Otherwise, -1 is returned and errno is set to EINVAL. */

int set_end;

static dev_t rbt_dev_number;      /* Allotted device number */
struct class *rbt_dev_class;          /* Tie with the device model */
static struct device *rbt_dev_device;



/*
* Open  driver
*/
int rbt_driver_open(struct inode *inode, struct file *file)
{
	struct rbt_dev *rbt_devp;
	rbt_devp->root=RB_ROOT;
	/* Get the per-device structure that contains this cdev */
	rbt_devp = container_of(inode->i_cdev, struct rbt_dev, cdev);
	/* Easy access to cmos_devp from rest of the entry points */
	file->private_data = rbt_devp;
	//printk("\n Driver is openning \n");
	return 0;
}

/*
 * Release rbt driver
 */
int rbt_driver_release(struct inode *inode, struct file *file)
{
	//struct rbt_dev *rbt_devp = file->private_data;
	
	//printk("\n Driver is closing \n");
	return 0;
}

// inserting the element.
int my_insert(struct rb_root *root, struct rb_object *data)
  { 
  	struct rb_node **new = &(root->rb_node), *parent = NULL;

  	// Figure out where to put new node
  	while (*new) {
  		struct rb_object *this = container_of(*new, struct rb_object, node);
  		int result = data->val.key - this->val.key;

		parent = *new;
  		if (result < 0) new = &((*new)->rb_left);
  		else if (result > 0) new = &((*new)->rb_right);
  		else return 0;
  	}

  	// Add new node and rebalance tree.
  	rb_link_node(&(data->node), parent, new);
  	rb_insert_color(&(data->node), root);

	return 1;
  }

// search function to search for a particular node with a particular key
struct rb_object *my_search(struct rb_root *root, int key)
{
      struct rb_node *node = root->rb_node;

      while (node) {
              struct rb_object *data = container_of(node, struct rb_object, node);
              int result;

              result = key - data->val.key;

              if (result < 0)
                      node = node->rb_left;
              else if (result > 0)
                      node = node->rb_right;
              else
                      return data;
      }
      return NULL;
}


ssize_t rbt_driver_write(struct file *file, const struct values *tmp )
{	int r;
	struct rb_object *obj;
	struct rb_object *obj2;
	struct rbt_dev *rbt_devp = file->private_data;
	
	obj=(struct rb_object *) kmalloc (sizeof(struct rb_object),GFP_KERNEL);
	// transferring to kernel space
	get_user(obj->val.key,&(tmp->key));
	get_user(obj->val.data,&(tmp->data));
	obj2=my_search(&(rbt_devp->root),obj->val.key);
// if the object is existing replace it .
	if(obj2!=NULL)
		{
		if(obj->val.data!=0)
			rb_replace_node(obj2,obj,&(rbt_devp->root));
		 else
			{
rb_erase(&(obj2->node),&(rbt_devp->root));    // if the data field value =0 delete it. 
}
}
	else
		r=my_insert(&(rbt_devp->root),obj);
	
	WRITE_BREAKPOINT=r;


	//printk("\n %d is the return value\n",r);
	//printk("\nWriting -- %d %d \n",obj->val.key, obj->val.data);
	return 0;
}

ssize_t rbt_driver_read(struct file *file, struct values *tmp)
{
	struct rb_object *obj;
	struct rb_object *next;
	struct rb_object *prev;
	struct rbt_dev *rbt_devp = file->private_data;

	
	if(rbt_devp->root.rb_node==NULL)
	{return -1;}
	/* Reading the data according the read direction */
	if(set_end==0){
		obj= container_of(rb_first(&(rbt_devp->root)), struct rb_object, node);
		if(&(obj->node)==rbt_devp->root.rb_node)
		{	//printk("Yes , root is object , key: %d",obj->val.key);
			next=rb_next(&(obj->node));
			if(next==NULL)
				{ 
				//printk("\n\n insidee  ======== NULLLl %d \n",obj->val.key);
				put_user(obj->val.key, &(tmp->key));
				put_user(obj->val.data, &(tmp->data));
				rb_erase(&(obj->node),&(rbt_devp->root));
				kfree(obj);
				rbt_devp->root.rb_node=NULL;}
			else
				{//printk("\n\n insidee  ~~~~~~~~~~~ NULLL %d \n",obj->val.key);
				put_user(obj->val.key, &(tmp->key));
				put_user(obj->val.data, &(tmp->data));
				rbt_devp->root.rb_node=next;
				rb_erase(&(obj->node),&(rbt_devp->root));
				kfree(obj);}
		}
		else 
			{
			put_user(obj->val.key, &(tmp->key));
			put_user(obj->val.data, &(tmp->data));
			rb_erase(&(obj->node),&(rbt_devp->root));
			kfree(obj);	
		}}

	if(set_end==1){
		obj= container_of(rb_last(&(rbt_devp->root)), struct rb_object, node);
		
		/* if the current node is root */
		
		if(&(obj->node)==rbt_devp->root.rb_node)
		{	
			//printk("Yes , root is object , key: %d",obj->val.key);
			prev=rb_prev(&(obj->node));
			
			/* if the previous node of the root is NULL, then sent this only node to user and set the root as NULL. */
			
			if(prev==NULL){ 
				//printk("\n\n insidee  ======== NULLLl %d \n",obj->val.key);
				put_user(obj->val.key, &(tmp->key));
				put_user(obj->val.data, &(tmp->data));
				rb_erase(&(obj->node),&(rbt_devp->root));
				kfree(obj);
				rbt_devp->root.rb_node=NULL;
			}
				
			/* if the previous node of the root is not NULL, then let the previous node to be root and erase the current root*/
			
			else{
				//printk("\n\n insidee  ~~~~~~~~~~~ NULLL %d \n",obj->val.key);
				put_user(obj->val.key, &(tmp->key));
				put_user(obj->val.data, &(tmp->data));
				rbt_devp->root.rb_node=prev;
				rb_erase(&(obj->node),&(rbt_devp->root));
				kfree(obj);}
		}
		
		/* if the current node is not root */
		
		else {
			put_user(obj->val.key, &(tmp->key));
			put_user(obj->val.data, &(tmp->data));
			rb_erase(&(obj->node),&(rbt_devp->root));
			kfree(obj);	
		}
	}
	
	READ_BREAKPOINT=0;	

	return sizeof(*obj);

}
// setting the direction of the read
 int rbt_ioctl(struct file *f,  int cmd)
{
	switch (cmd){
		
	case 0:
		return 0;
		break;

	case 1:
		return 0;
		break;

	default:
		return -EINVAL;
	}
}

/* File operations structure. Defined in linux/fs.h */
static struct file_operations rbt_fops = {
    .owner		= THIS_MODULE,           /* Owner */
    .open		= rbt_driver_open,        /* Open method */
    .release	= rbt_driver_release,     /* Release method */
    .write		= rbt_driver_write,       /* Write method */
   .read		= rbt_driver_read,        /* Read method */
   .unlocked_ioctl              = rbt_ioctl,
};

/*
Driver Initialization
*/
int __init rbt_driver_init(void)
{
	int ret;
	
	/* Request dynamic allocation of a device major number */
	if (alloc_chrdev_region(&rbt_dev_number, 0, 1, DEVICE_NAME) < 0) {
			printk(KERN_DEBUG "Can't register device\n"); return -1;
	}

	/* Populate sysfs entries */
	rbt_dev_class = class_create(THIS_MODULE, DEVICE_NAME);

	/* Allocate memory for the per-device structure */
	rbt_devp = kmalloc(sizeof(struct rbt_dev), GFP_KERNEL);
		
	if (!rbt_devp) {
		printk("Bad Kmalloc\n"); return -ENOMEM;
	}

	/* Connect the file operations with the cdev */
	cdev_init(&rbt_devp->cdev, &rbt_fops);
	rbt_devp->cdev.owner = THIS_MODULE;

	/* Connect the major/minor number to the cdev */
	ret = cdev_add(&rbt_devp->cdev, (rbt_dev_number), 1);

	if (ret) {
		printk("Bad cdev\n");
		return ret;
	}

	/* Send uevents to udev, so it'll create /dev nodes */
	rbt_dev_device = device_create(rbt_dev_class, NULL, MKDEV(MAJOR(rbt_dev_number), 0), NULL, DEVICE_NAME);		
	printk("rbt driver initialized.\n");
	return 0;
}

/* Driver Exit */

void __exit rbt_driver_exit(void)
{
	unregister_chrdev_region((rbt_dev_number), 1);

	/* Destroy device */
	device_destroy (rbt_dev_class, MKDEV(MAJOR(rbt_dev_number), 0));
	cdev_del(&rbt_devp->cdev);
	kfree(rbt_devp);
	
	/* Destroy driver_class */
	class_destroy(rbt_dev_class);
	printk("rbt driver removed.\n");
}
EXPORT_SYMBOL(WRITE_BREAKPOINT);
EXPORT_SYMBOL(READ_BREAKPOINT);
module_init(rbt_driver_init);
module_exit(rbt_driver_exit);
MODULE_LICENSE("GPL v2");

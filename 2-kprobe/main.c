/*
CSE 530 Embedded Operating System Internal -- Fall 2017
Assignment 1. RB Tree and Dynamic Probe in Linux Kernel (200 points)
	  Part 1: Accessing a kernel RB tree via device file interface
Edited by: Kausic Gunasekar, Beibei Liu (team 20) 
Created: 09/18/2017
Last edited: 09/22/2017
Emails: ?????

============================== main ==============================

*/

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <linux/sched.h>
#include <pthread.h>
#include <linux/ioctl.h>
#include <time.h>

/* ---------- parameters -------------- */
int totalOperations=101;          /* total number of read() and write() operations */
pthread_mutex_t mut;         

struct parameters{                /* data structure of the objects to be added to or removed from the RB tree */
	int key;
	int data;
}*par_t;

static struct probeData {       // data structure that users want to know
	unsigned long int kp_addr;
	unsigned long long int tsc;
	int pid;
}*data;

/*
void * Populate (void* file_d) populates (by calling write operation) the RB tree.
The threads are set with different real-time priorities and consecutive file operations are invoked after a random delay. 
*/

void * Populate (void* file_d){
	
	/* set iterating parameters for populating the RB tree for total 40 times. */
	printf("Populating the tree \n");
	
	int i=0,count=10;
	
	/* get the thread id of the current thread calling this function. */
	
	pthread_t this_thread=pthread_self();
	
	/* set the real-time priorities of threads. */
	
	struct sched_param params; 
	params.sched_priority =rand()%100; 
	pthread_setschedparam(this_thread, SCHED_RR, &params);
	
	/* set the mutex lock. */
	
	pthread_mutex_lock(&mut);
	
	/* assign values to key and data of the object ("par_t") to be populated into RB tree. */
	int *fd;
	fd=(int *) file_d;
	for (i =0;i<count ;i++){
		par_t->key=rand() % 10001;
		par_t->data=rand() % 10001;
		//printf("\n Populating the Tree :Key : %d data : %d\n",par_t->key,par_t->data);
		
		/* write the object "par_t". */
		
		write(fd,par_t);
	}

	/* unlock mutex. */
	
	pthread_mutex_unlock(&mut);
return 0;

}

/*
void * Randomly (void* file_d) invokes read() and write() randomly for a total of 100 read() and write() operations.
*/

void * Randomly (void* file_d){
	int *fd;
	fd=(int *) file_d;
	
	/* get the thread id of the current thread calling this function. */
	
	pthread_t this_thread=pthread_self();
	
	/* set the real-time priorities of threads. */   
	

	struct sched_param params; 
	params.sched_priority =rand()%100; 
	pthread_setschedparam(this_thread, SCHED_RR, &params);
	
	/* set the mutex lock. */
	
	pthread_mutex_lock(&mut);     
	/* call the ioctl() to set the read direction of the RB tree. */
	
	ioctl(fd,rand()%2);
	
	/* unlock mutex. */
	
	pthread_mutex_unlock(&mut);   
	
	while(1){
		pthread_mutex_lock(&mut);
		
		/* if all the read() and write() operations are finished, then unlock the mutex and jump out of the while loop. */
		
		if(totalOperations==0)
{
			pthread_mutex_unlock(&mut);
			break;
		}
		
		/* randomly call read() and write(), each time the function is excuted, the counter "totalOperations" reduces one. */
		
		int choice=rand()%2;
		totalOperations--;
		
		/* read() */
		if(choice==0){
			read(fd,par_t);
		//printf("Thread : '%u' op count : %d Reading randomly :  Key : %d data : %d\n",(unsigned int)pthread_self(),totalOperations,par_t->key,par_t->data);
		}
		
		/* write() */
		else{
			par_t->key=rand() % 10001;
			par_t->data=rand() % 10001;
			write(fd,par_t);
			//printf("Thread : '%u' op count : %d Writing randomly :  Key : %d data : %d\n",(unsigned int)pthread_self(),totalOperations,par_t->key,par_t->data);
}
			//sleep(rand()%3);
			
			/* Unlock the mutex */
			pthread_mutex_unlock(&mut);
		
	}
return 0;
}
int start=0;
void *probing (void* filep)
{
char buff[256];//="rbt_driver_write"; 
int *fpp;
	fpp=(int *) filep;
	printf("\n Enter the location probe entry\n");
	scanf("%s",buff);
	//printf("\n chosen %s\n",buff);
	char argv1[]="1";
	memset(buff, 0, 256);
	sprintf(buff,"%s", argv1);  //WRITE_BREAKPOINT
	sprintf(buff,"%s%s", argv1,"rbt_driver_write"); //specify the function you would like to insert kprobe
	int str_len=strlen(buff)+1; 
	write(fpp,buff,str_len); //registering kprobe and setting entry point at the function / symbol name.
	pthread_mutex_lock(&mut);
	start=1;
	pthread_mutex_unlock(&mut);
	
	while(1){
		int ret=read(fpp,data);   
		if(ret==-1)
		printf("Buffer empty\n");
		else{
			printf("\n PID : %d   TSC : %llu PROBE ADDRESS : 0x%lu   \n",data->pid,data->tsc,data->kp_addr);
		}
	}
return 0;
}

/*
int main(int argc, char **argv) creates 4 threads, populates the RB tree with a total of 40 objects.
Then it invokes the randomly() for a total of 100 read() and write() operations.

*/
int main(int argc, char **argv)
{
     /* Declare the four threads and the result of the thread creation. */
	 
	pthread_t thread1,thread2,thread3,thread4,probe_thread;
     int  iret1, iret2,iret3,iret4,iret5;

	 
	 /* allocate memory for the object passed from kernel space. */
	data=(struct probeData*)malloc(sizeof(struct probeData));
	 
     par_t=(struct parameters *)malloc(sizeof(struct parameters));
	 
	 /* open the device rbt_rbt530. */
	 
	int *fd,*fd2;
     fd = open("/dev/rbt_530", O_RDWR);
     fd2=open("/dev/RBprobe",O_RDWR); 


	 /* create four threads for populating the RB tree. */

     iret5 = pthread_create(&probe_thread,NULL,probing,fd2);  // create a separate thread
	
     //pthread_join(probe_thread,NULL);
     while(start==0);
     iret1 = pthread_create( &thread1, NULL, Populate, fd);
     iret2 = pthread_create( &thread2, NULL, Populate, fd);
     iret3 = pthread_create( &thread3, NULL, Populate, fd);
     iret4 = pthread_create( &thread4, NULL, Populate, fd);
     
    if(iret1 || iret2 || iret3 || iret4 || iret5){
        fprintf(stderr,"Error - pthread_create() return code: %d %d %d %d\n",iret1,iret2,iret3,iret4);
        exit(EXIT_FAILURE);
    }

	 /* wait for the threads to terminate. */
	 
    pthread_join( thread1, NULL);
    pthread_join( thread2, NULL);
    pthread_join( thread3, NULL);
    pthread_join( thread4, NULL);

	/* create four threads for reading and writing randomly. */

	iret1 = pthread_create( &thread1, NULL, Randomly, fd);
	iret2 = pthread_create( &thread2, NULL, Randomly, fd);
    iret3 = pthread_create( &thread3, NULL, Randomly, fd);
    iret4 = pthread_create( &thread4, NULL, Randomly, fd);
printf("\n\nReading and Writing Randomly\n");
    if(iret1 || iret2 || iret3 || iret4){
        fprintf(stderr,"Error - pthread_create() return code: %d %d %d %d\n",iret1,iret2,iret3,iret4);
        exit(EXIT_FAILURE);
    }
	
	/* wait for the threads to terminate. */
	
	pthread_join( thread1, NULL);
    pthread_join( thread2, NULL);
    pthread_join( thread3, NULL);
    pthread_join( thread4, NULL);


	/* set a counter to count the number of tree nodes. */
	
	int counter=0;
	
	/* read the objects from the tree and print out the key and data. */
	
	printf("\n\n\nThe tree Has the following data");

	while(1){
		if (read(fd,par_t)==-1)
			break;
		printf("\n Key : %d  Data : %d \n",par_t->key,par_t->data);
		counter++;
	}
	printf("\n Done");
	printf("\n Total objects = %d \n",counter);
	sleep(1);
	char c[5]="0";
	write(fd2,c,strlen(c)+1);
	pthread_cancel(probe_thread);
	close(fd);
	close(fd2);
	
	return 0;
}



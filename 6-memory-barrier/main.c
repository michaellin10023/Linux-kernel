#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>	
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <sys/wait.h>
#define TIMEOUT 3000
struct thread_data // data to be passed on to threads
{
	int *fd;
	unsigned int * bid;
 	int iters;
 	int sleep_time;
};
int error_val=0;
void * thred_func(void *t) // The thread function that all threads call
{
	struct thread_data *t_inf;
	t_inf=(struct thread_data *) t;
	int i;
	unsigned int ev=0;
	
	int thread_id=syscall(SYS_gettid);  // getting thread ID
	int process_id =getpid();	    // getting process ID
	for(i=0;i<t_inf->iters;i++){
	
	printf("Round %d | thread ID : %d | process ID : %d | Barrier ID : %d | \n",i+1,thread_id,process_id,*t_inf->bid);

	usleep(rand()%t_inf->sleep_time);
	ev= syscall(361, *t_inf->bid); //barrie_wait
	error_val=ev;
	if(error_val==-1)
	{
		printf("Timed out\n");	
		error_val=0;
		
	}}
	
	sleep(1);
	
return 0;
} 
// This routine tests according to the assignment requirements
void barrier_tester(unsigned int * barrier_1,unsigned int * barrier_2,int st)
{
		struct thread_data *tdata=(struct thread_data *)malloc(sizeof(struct thread_data));
		printf("Testing Barrier for all synchronization rounds and all threads\n");
	   
	 	tdata->bid=barrier_1;
	 	tdata->iters=100;
	 	tdata->sleep_time=st;
		printf("Excercising the first barrier");
		sleep(2);
		int i=0;
		pthread_t thread_[20];
		
		for ( i=0;i<5;i++)
		pthread_create( &thread_[i], NULL, thred_func, tdata);  // creating and running the five threads
		for (i=0;i<5;i++) 
	    pthread_join( thread_[i], NULL);
    
	    printf("Excercising the second barrier");
	    sleep(2);
		
		tdata->bid=barrier_2;
		tdata->iters=100;
		tdata->sleep_time=st;
		for (i=0;i<20;i++)
		pthread_create( &thread_[i], NULL, thred_func, tdata); // creating and running  the 20 threads
		for (i=0;i<20;i++)
	    pthread_join( thread_[i], NULL);
	   
}

// destroy the barriers 
void barrier_destroy_tester(unsigned int * barrierID,int st)
{
		printf("THIS TESTS THE BARRIER DESTROY FUNCTIONALITY\n");
		printf("barrier id received %d",*barrierID);
		struct thread_data *tdata=(struct thread_data *)malloc(sizeof(struct thread_data));
		sleep(3);
		syscall(362, *barrierID);     // system call of the destroy function
		tdata->bid=barrierID;
		tdata->iters=10;
		tdata->sleep_time=st;
		int i=0;
		pthread_t thread_[5];
		
		for ( i=0;i<5;i++)  
		pthread_create( &thread_[i], NULL, thred_func, tdata);   // creating and running the 5 threads
		for (i=0;i<5;i++)   
	    	pthread_join( thread_[i], NULL);
	    
}

// test the timeout cases
void barrier_timeout_tester(unsigned int * barrierID,int st)
{
		printf("NOW THIS TESTS THE BARRIER TIMEOUT FUNCTIONALITY\n");
		struct thread_data *tdata=(struct thread_data *)malloc(sizeof(struct thread_data));
		
		tdata->bid=barrierID;
		tdata->iters=10;
		tdata->sleep_time=st;
		int i=0;
		pthread_t thread_[20];
		
		for ( i=0;i<20;i++)
		pthread_create( &thread_[i], NULL, thred_func, tdata);    // 20 threads being created and run
		for (i=0;i<20;i++) 
	    pthread_join( thread_[i], NULL);
	    
}



void child_process(int st)
{  // this function is run by both children
		 
		 unsigned int thread_count=5;
		 int timeout=TIMEOUT;
		 unsigned int *barrier_1 = (unsigned int *)malloc(sizeof(unsigned int));
		 syscall(360, thread_count,barrier_1,timeout);  // barrier init
		 printf("Barrier has been initialized with id %d\n",*barrier_1);

		 thread_count=20;
		 timeout=0;
		 unsigned int *barrier_2 = (unsigned int *)malloc(sizeof(unsigned int));
		 syscall(360, thread_count,barrier_2,timeout); // barrrier init
		 printf("Barrier has been initialized with id %d\n",*barrier_2);
		 printf("Testing our barrier\n");
		 barrier_tester(barrier_1,barrier_2,st);

		 printf("Now demonstrating with barrier destroy\n\n");
		 sleep(2);
		 barrier_destroy_tester(barrier_1,st); // barrier destroy
		 printf("Now demonstrating Timeout functionality\n\n");
		 sleep(2);
		 unsigned int *barrier_3 = (unsigned int *)malloc(sizeof(unsigned int));
		 thread_count=20;
		 timeout=30;
		 syscall(360, thread_count,barrier_3,timeout);
		 printf("Barrier has been initialized with id %d\n",*barrier_3);
		 barrier_timeout_tester(barrier_3,st);
}

int main()
{
	int pid;
	int status;
	int st;
	printf("This program demonstrates the various barrior synchronization attributes of our method\n\n");
	printf("Enter average sleep value:");
	scanf("%d",&st);

	pid=fork();
	printf("pid values after forking %d \n",pid);
	if(pid==0){    // ================ first child process ================
		child_process(st);
		printf("Done executing 1st child\n");
		
	}
	else        // parent process
		{
		child_process(st); // second child
		printf("Done executing 2nd child\n");
		}
		

    pid=wait(&status);
    pid=wait(&status);
 
	sleep(1);

return 0;
}


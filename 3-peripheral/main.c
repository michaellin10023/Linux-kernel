#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
struct fifo_buffer{
	unsigned long long int time_stamp;
	unsigned long long int value;
} *buff;

struct Cmd{     		 // configuration structure
	char cmd_name[20];
	int para_1;
	int para_2;
};

void read_and_print(int fd) // this function reads data from kernel and prints it.
{
	buff = (struct fifo_buffer*)malloc(sizeof(struct fifo_buffer));
	int ret=read(fd,buff);
	printf("Time stamp : %llu \n",buff->time_stamp);
	printf("Distance   : %llu cm \n",buff->value);
}

void * read_write(void * file_d)    // thread function 
{
	int *fd;
	int i,j;
	fd=(int * )file_d;
	
	for (i=0;i<50;i++)
	{		for (j=0;j<rand()%4;j++)
				write(*fd,i%3);

		if(i%2==0)
			read_and_print(*fd);
	}
}
int main(int argc, char **argv) 
{
	int fd1,fd2,res;
	pthread_t t1,t2;
	int r1,r2;

	printf("This is a user test program to demonstrated the functionality of the HSCR driver\n");
	
	//struct Cmd  *cmd2 = (struct Cmd *) malloc
	struct Cmd  *cmd1 = (struct Cmd *) malloc (sizeof(struct Cmd));
	/* open devices */
	fd1 = open("/dev/HCSR_0", O_RDWR);
	if (fd1< 0){
		printf("Can not open device file.\n");		
		return 0;
	}
	fd2 = open("/dev/HCSR_1", O_RDWR);
	if (fd2< 0){
		printf("Can not open device file.\n");		
		return 0;
	}
	sleep(1);
	// ********* configuring  sensor 1*******
	strcpy(cmd1->cmd_name,"CONFIG_PINS");
	cmd1->para_1 = 7;//40 ;
	cmd1->para_2 = 5;//6 ;
	ioctl(fd1,cmd1);

	strcpy(cmd1->cmd_name,"SET_PARAMETERS");
	cmd1->para_1 = 3 ;
	cmd1->para_2 = 25 ;
	ioctl(fd1,cmd1);

	
   // *********** configuring sensor 2***********8//
	strcpy(cmd1->cmd_name,"CONFIG_PINS");
	cmd1->para_1 = 8;//40 ;
	cmd1->para_2 = 4;//6 ;
	ioctl(fd1,cmd1);

	strcpy(cmd1->cmd_name,"SET_PARAMETERS");
	cmd1->para_1 = 3 ;
	cmd1->para_2 = 25 ;
	ioctl(fd1,cmd1);

	r1=pthread_create(&t1,NULL,read_write,&fd1);
	r2=pthread_create(&t2,NULL,read_write,&fd2);
	if(r1 || r2)
	{
		printf("Error in pthread create\n");
		return -1;
	}

	pthread_join(t1,NULL);
	pthread_join(t2,NULL);
	
	sleep(1);
	close(fd1);
	close(fd2);
	return 0;
}

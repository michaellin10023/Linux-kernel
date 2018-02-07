/* 
Assignment 3 A SPI-based 1-wire Device Driver for LED Strip
this is a user space program for spi driver
by team 20: Kausic (1213203730), beibei (1210467866)
*/

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#define RESET 0
struct Info 
{
	int start_led;
	int val;
	int color;	
};

//int open(int,int);
int close(int);
long ioctl(int , char *,unsigned long);
ssize_t write(int, const struct Info *);
int sleep(int);
int main()
{
	int f,i,l;
	int n_pixels=0;
	int count=0;
	int color_choice=0;
	int loops=3;
	int flag=0;
	struct Info *in_buf = (struct Info *) malloc(sizeof(struct Info));

	f=open("/dev/WS2812", O_RDWR);
	if(f<0)
		printf("Error opening the file\n");

	//----------------- user input number of leds "n" -----------
	while(1)
	{
		printf("Enter the n_pixel data : ");
		scanf("%d",&n_pixels);
		if(n_pixels>0 && n_pixels<=16)
			break;
		else
			printf("Please enter a valid number \n");
		}

	//----------------- user input color -------------------
	while(1)
	{
		printf("Enter choice of color : \n 0 -> green \n 1 -> red \n 2 -> blue 3 -> random \n");
		scanf("%d",&color_choice);
		if(color_choice>=0 && color_choice<=3)
			break;
		else
			printf("please enter a valid choice \n");
	}

	//----------------- user input loops of circulating -------------------
	printf("Enter the number of times you would want the display to circulate : ");
	scanf("%d",&loops);

	//********************************** RESET and configure *********************************
	ioctl(f,RESET,0);
	
	// ************************* Circulating display routine **********************************
	if(loops==0)
		{loops=1;flag=1;}
	do {
	for (l=0;l<loops*16;l++)
	{  
		for (i=l;i<l+n_pixels;i++)
		{
			in_buf->start_led=i%16;
			in_buf->val=255;
			if(color_choice==3)
				in_buf->color=count%3;
			else
				in_buf->color=color_choice;
			count++;
			write(f,in_buf);

		}
	sleep(1);
	for (i=l;i<l+n_pixels;i++)
	{
		in_buf->start_led=i%16;
		in_buf->val=0;
		write(f,in_buf);

	}
	}	
	}while(flag==1);

	//***************************************  display routine end *******************************
	close(f);
	return 0;
}

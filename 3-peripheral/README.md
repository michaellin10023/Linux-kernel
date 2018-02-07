<h1> Assignment 2 Part 1 :  A Device driver  for HCSR04 Sensor </h1>


This document goes into detail about the driver program hcsr_drv.c which is the kernel driver for the HCSR04 ultrasonic sensor. The repository additionally contains a test program main.c and the corresponding make file for the program.

<u>Contents:</u>


main.c - The user test program to test various scenarios.

HCSR_drv.c - The driver program for the HCSR sensor.

Makefile - Make file to compile the program.

Readme - readme file

Once you download the files, navigate into the directory that contains the four files. Now it is assumed that the sdk is already installed in your computer. The default directory for the set up is /opt/iot-devkit/1.7.2/sysroots/ which will contain the kernel and the i586-poky-linux-gcc. If you have installed your sdk in some other directory then make sure the path variable leads to your i586-poky-linux-gcc compiler and the make is given the right directory to the kernel. 

If that has been set, open the terminal and type in <b><i> make</i></b>
You should be able to see some files that are created in the same directory.

The items of interest are [HCSR_drv.ko, test]

Once you get these files, You will have to transfer the files to the galileo board. 

Type in the following command.

<i> sudo scp HCSR_drv.ko root@192.168.1.5:/home/ </i>

<i> sudo scp test root@192.168.1.5:/home/ </i>


Replace the ip address with the static IP you have assigned to your board. 
Now that you have the files, you can install the module in. Type in the following command from the directory where you have the hcsr_drv.ko and the test binary in the galileo.

<i> insmod HCSR_drv.ko num_devices=2 </i>

The "2" should be replaced with the number of drivers you want to install. After installing, you should be able to see the devices listed in the /dev directory.

You can check this out by typing in :

<i> ls /dev/HCSR* </i>

In this case it should list out /dev/HCSR_0, /dev/HCSR_1

hardware connections:

we connect the vcc pins of both sensors to 5v, trig pins are connected to IO 7 and 8 on Galileo board, echo pins are connected to IO 5 and 4 on Galileo board, GND pins of both are connected to GND. 



<h2> The driver program </h2>

There are five functions in the driver module:

1.ioctl:

The ioctl function is used to either configure the parameters or configure pins. It accepts a file pointer and a pointer to the  structure with a string and 2 integers from user space. The string could be either "CONFIG_PINS"  or " SET_PARAMETERS". Each mode takes two arguements which are integers. The former takes the trigger pin and the echo pin number while the latter takes the number of samples (m) value and the sampling period (delta).

In the "CONFIG_PINS" mode, the pin numbers are checked if they are valid and the echo pin is checked if it has interrupts also. Then, the trigger and echo pins are passed through a pin mux table to set up the multiplexing. This could be seen in the driver program in a function called cmd_table. The pins are initialized accordingly. In this case, two HCSR devices are tested. IO 7 and IO 8 on Galileo board are used as two trig pins, while IO 5 and IO 4 are used as two echo pins, respectively. 

In the "SET_PARAMETERS" mode, user sets the sampling period (delta) and the number of samples (m) in the per_device structure. 

2. write:

The write function calls measure() function which takes the average of m median values after taking in m+2 readings. hrtimer is implemented to measure the duration of the echo pulse. The averaged value of distance readings is then entered into the fifo_buffer that we had implemented. Along with the distance reading in centimeters the corresponding timestamp is also entered. This is done only when there is no on-going measurement which is checked using the measurement flag. The fifo_buffer pointers are reset if the paramter to write function is non-zero.

3. read:

The read function reads from the fifo buffer implemented and transfers the content to the user space. The data is obtained using a similar structure.
 
4. open:

open the device.

5. release:

close the descriptor of the opened file.


init and exit functions:

1. init:

In init function, the misc device is registered. FIFO buffer is initialized. hrtimer is initialized. 

2. exit:

In exit function, the misc device is unregistered. hrtimer is cancelled. GPIOs and irqs are freed.


**Measurement:**

The measure() functions calls the trigger function which sets the trigger pin high for 10us and then gets the data that is calculated in the interrupt handler. 

You can run the test program ./test to see the working of the ultrasonic driver. It will print out the averaged distance and the time stamp of each device, respectively. 


<i> the test program (main.c) </i>

In the test program, 

It opens the device. Then it calls the ioctl function to configure the trig pin, echo pin, the number of samples (m), and the sampling period (delta). Then multiple (2) threads are created. Each thread calls the read and write functions for each of the device. The write and read functions happen a number of times and after reading the values are printed to the console.









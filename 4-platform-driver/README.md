<h1> Assignment 2 Part 2 :  Platform device driver and sysfs interface for HCSR04 Sensor </h1>

team 20: Kausic (1213203730), beibei (1210467866)

This document goes into detail about the process of platform device registration and driver-device match.
Check the hardware connection side heading for the hardware that is connected.


<u>Contents:</u>
The repository contains 6 files:
1) HCSR_of_device.c - the device (HCSR sensor) module showing the registration of the platform devices. 
2) HCSR_of_driver.c - the driver module for platform devices showing the process of platform devices matched with platform driver. 
3) HCSR_of_device.h - header file for platform device module, defining the data structures.
4) main.c - the test program working for device file interface to test various scenarios.
5) Test_script.sh - the bash file for configuring the parameters (trigger/echo pins, number of samples, sampling period,and enable) and showing the distance measurement output.
5) Makefile - Make file to compile the program.
6) README - read me file


<u>Compile Instructions:</u>

Once you download the files, navigate into the directory that contains the 6 files. Now it is assumed that the sdk is already installed in your computer. The default directory for the set up is /opt/iot-devkit/1.7.2/sysroots/ which will contain the kernel and the i586-poky-linux-gcc. If you have installed your sdk in some other directory then make sure the path variable leads to your i586-poky-linux-gcc compiler and the make is given the right directory to the kernel. 

If that has been set, open the terminal and type in <b><i> make</i></b>
You should be able to see some files that are created in the same directory.

The items of interest are [HCSR_of_device.ko, HCSR_of_driver.ko, Test_script.sh]

Once you get these files, You will have to transfer the files to the galileo board. 

Type in the following command.

<i> sudo scp HCSR_of_driver.ko root@192.168.1.5:/home/ </i>

<i> sudo scp HCSR_of_device.ko root@192.168.1.5:/home/ </i>

<i> sudo scp test root@192.168.1.5:/home/ </i>

<i> sudo scp Test_script.sh root@192.168.1.5:/home/ </i>

Replace the ip address with the static IP you have assigned to your board. 
Now that you have the files, you can install the module in. Type in the following command from the directory where you have the HCSR_of_driver.ko and the test binary in the galileo.

<i> insmod HCSR_of_driver.ko </i>

<i> insmod HCSR_of_device.ko </i>

After installing, you should be able to see the devices listed in the /dev directory. By default, there are two HCSR devices installed.
You can check this out by typing in :

<i> ls /dev/HCSR* </i>

In this case it should list out:
<i> /dev/HCSR_0 </i>
<i> /dev/HCSR_1 </i>

From the sysfs interface, you should be able to see the interfaces listed in the /sys/class directory. In our case, there is 1 class (HCSR), 2 devices (HCSR_0, HCSR_1), and 6 attributes (trigger pin, echo pin, number of samples, sampling period, enable and distance) .
You can check the class out by typing in :
<i> ls /sys/class/HCSR </i>

You can check the devices out by typing in :
<i> ls /sys/class/HCSR/HCSR* </i>

In this case it should list out:
/sys/class/HCSR/HCSR_0:
distance         number_samples   subsystem
echo             power            trigger
enable           sampling_period  uevent

/sys/class/HCSR/HCSR_1:
distance         number_samples   subsystem
echo             power            trigger
enable           sampling_period  uevent


Before you run the bash file "Test_script.sh", change the permission by typing in :
<i> chmod 777 Test_script.sh </i>

Then, make sure you are at the root directory of the "Test_script.sh" file, 
If you want to log all the output from terminal, type:
<i> bash -x Test_script.sh | tee log.file </i>
A log file "log.file" will be generated in the current directory.

Otherwise, type:
<i> ./Test_script.sh </i>

In the "log.file" (or console), you will be able to see:
1. the trig/echo pin number, number of samples, and sampling period configurations
2. the distance outputs of HCSR_0 and HCSR_1 (two threads are writing and reading concurently, therefore the outcomes are generated concurently)

<i> hardware connections:</i>

we connect the vcc pins of both sensors to 5v, trig pins are connected to IO 7 and 8 on Galileo board, echo pins are connected to IO 5 and 4 on Galileo board, GND pins of both are connected to GND. 

<h2> The device program (HCSR_of_device.c)</h2>

The device program initializes two chips (P1_chip, P2_chip) of the data structure "P_chip", which contains a platform device data structure "plf_dev". Then when "insmod HCSR_of_device.ko" command is issued, the "p_device_init()" function is called to register the platform devices in each P_chip, respectively. When "rmmod HCSR_of_device.ko" command is issued, the "p_device_exit" function is called to unregister the platform devices.


<h2> The driver program (HCSR_of_driver.c)</h2>

The driver program performs the major file operations. 

It uses the following functions from the device driver "HCSR_drv.c" in assignment 2 part 1:

Triggering(): set values for trig pin
handling_irq(): callback function of the interrupt, waiting for rising/falling edges
measure(): make m+2 measurement ever delta milliseconds
configure_parameters(): set value for number of samples and sampling period
configure_pins(): Configures the pins according to the pin mux table

HCSR_driver_open(): open the device and check the minor number in each misc device
HCSR_driver_release(): close the device
HCSR_driver_write(): if the "measurement_flag" is 1, start a new measurment by calling "measure()" function
HCSR_driver_read(): read the distance and time stamp from FIFO buffer and send to user space
HCSR_ioctl(): accept the command ("CONFIG_PINS" / " SET_PARAMETERS"), trig/echo pins, value for number of samples, and sampling period from user space, doing configurations.


The driver program creates the following new functions different from the part 1:

Each interface has attributes "show" and "store":

trigger_show(): show trig pin by command "cat".
trigger_store(): set trig pin by command "echo".
echo_show(): show echo pin by command "cat".
echo_store(): set echo pin by command "echo".
number_samples_show(): show number of samples by command "cat".
number_samples_store(): set number of samples by command "echo".
sampling_period_show(): show sampling peiod in millisecond by command "cat".
sampling_period_store(): set sampling peiod in millisecond by command "echo".
enable_show(): show enable status by command "cat".
enable_store(): set enable status by command "echo".

interface "distance" only has attribute "show":
distance_show(): show the most recent distance measurement.

cmd_table(): realized the pinmux table. For each given trig pin number, it requests the corresponding gpios, sets the value, direction, level shifter, and pin mux. For each echo pin number, apart from the settings it does with trig pins, it also requests the irq, calling the callback function to wait for rising/falling edges.


<h2> init and exit functions: </h2>

As a prerequite, a global counter "num_devices" is defined, which has the initial value 0. Every time the "P_driver_probe()" function is called, it increases one; every time "P_driver_remove" function is called, it decreases one. In this way, each init function will have the index reference for which device it is working with.


1. init:

<i> P_driver_probe(): </i> 

When "insmod HCSR_of_driver.ko" command is issued, "P_driver_probe()" function is triggered. 
"P_driver_probe()" function finds the address of the data structure "P_chip" by address of the platform device data structure "plf_dev" (using container_of). Then it calls the function "HCSR_driver_init()".

<i> HCSR_driver_init(): </i> 

"HCSR_driver_init()" function initializes the FIFO buffer pointers(head, curr, count) to the start point 0, initilize the gpios to -1, register misc devices, initialize the hr timer, and set the attributes of intefaces through "device_create_file()" functions.

2. exit:

<i> P_driver_remove(): </i>
When "rmmod HCSR_of_driver.ko" command is issued, "P_driver_remove()" function is triggered. 
"P_driver_remove()" function calls the function "HCSR_driver_exit()".

<i> HCSR_driver_exit(): </i>
"HCSR_driver_exit()" function frees the resources. It frees the gpios and irqs, unregisters the misc device, cancels the hrtimer, and destroy the classes and devices.



<i> the test program (main.c) </i>
The test program for assignment 2 part 1 should also work for assignment 2 part 2.
In the test program, it opens the device. Then it calls the ioctl function to configure the trig pin, echo pin, the number of samples (m), and the sampling period (delta). Then multiple (2) threads are created. Each thread calls the read and write functions for each of the device. The write and read functions happen a number of times and after reading the values are printed to the console.


<i> ./test </i>

This should run the test program using the /dev/ interface.









<h1> Assignment 3 A SPI-based 1-wire Device Driver for LED Strip </h1>

team 20: Kausic (1213203730), beibei (1210467866)

This document goes into detail about the process of implementing a SPI device driver which can accept pixel information from users and light up 16 RGB LEDs of a LED ring.


<u>Contents:</u>
part 1: The repository contains 5 files:
1) ring_device.c - the device (WS2812) module showing the registration of the spi devices. 
2) ring_driver.c - the driver module for SPI devices and driver match and perform write, ioctl functions.
3) main.c - the test program working for device file interface to test various scenarios.
4) Makefile - Make file to compile the program.
5) README - read me file

<i> hardware connections:</i>

5V pins is connected to 5v on Galileo board, DI (Data_In) pin is connected to IO 1 on Galileo board, GND pin is connected to GND on Galileo board.

<u>Compile Instructions:</u>

Once you download the files, navigate into the directory that contains the 5 files. Now it is assumed that the sdk is already installed in your computer. The default directory for the set up is /opt/iot-devkit/1.7.2/sysroots/ which will contain the kernel and the i586-poky-linux-gcc. If you have installed your sdk in some other directory then make sure the path variable leads to your i586-poky-linux-gcc compiler and the make is given the right directory to the kernel. 

If that has been set, open the terminal and type in <b><i> make</i></b>
You should be able to see some files that are created in the same directory.

The items of interest are [ring_device.ko, ring_driver.ko, test]

Once you get these files, You will have to transfer the files to the galileo board. 

Type in the following command:

<i> sudo scp ring_driver.ko root@192.168.1.5:/home/ </i>

<i> sudo scp ring_device.ko root@192.168.1.5:/home/ </i>

<i> sudo scp test root@192.168.1.5:/home/ </i>

Replace the ip address with the static IP you have assigned to your board. 
Now that you have the files, you can install the module in. Type in the following command from the directory where you have the ring_driver.ko and the test binary in the galileo.

<i> insmod ring_driver.ko </i>

<i> insmod ring_device.ko </i>

After installing, you should be able to see the device listed in the /dev directory. 
You can check this out by typing in :

<i> ls /dev/WS2812</i>

From the sysfs interface, you should be able to see the spi device listed under /sys/bus/spi/device directory. 
You can check it out by typing in :
<i> ls /sys/bus/spi/devices </i>

You should be able to see the spi driver listed under /sys/bus/spi/driver directory. 
You can check it out by typing in :
<i> ls /sys/bus/spi/drivers </i>

Also the interfaces are listed in the /sys/class directory.
You can check the class out by typing in :
<i> ls /sys/class/WS2812 </i>

You can check the devices out by typing in :
<i> ls /sys/class/WS2812/WS2812 </i>

Then, to run the user program by typing in:
<i> ./test </i>

You may be asked to input the number of Leds you want to light up. You can put any number from 1 to 16. After input, please put "enter".
Then you may be asked to input the color choice. 0 -> green, 1 -> red, 2 -> blue, 3 -> random. You can put any number from 0 to 3. After input, please put "enter".
Then you may be asked to input the number of times you would want the display to circulate. You can put any positive integer number. If you put 0, it will circulate forever. After input, please put "enter".

We recommend the following values: [**n=3, color choice =3  and loops =3**]
If you do choose zero you can kill the forever loop using CTRL+C since this is on the user side.

Then you can see the leds lighting in a circulating pattern as you indicated.


<h3> The device program (ring_device.c)</h3>

The device program initializes and register the spi device. In our case, the device data structure (spi_board_info) is "ring_info". 

<h4> init and exit functions: </h4>

1. init:
<i> initialize(): </i> 
When "insmod ring_device.ko" command is issued, the "initialize()" function is called to initialize and register the spi device.
2. exit:
<i> cleanup(): </i> 
When "rmmod ring_device.ko" command is issued, the "cleanup()" function is called to unregister the spi device.

<h3> The driver program (ring_driver.c)</h3>

The driver program performs the major file operations. 

<h4> open, release, ioctl, write functions: </h4>

open(): open the WS2812 device
release(): close the file descriptor
ioctl(): "RESET" or configure the GPIOs, setting SPI to a suitable mode
write(): put the data to SPI-MOSI for 1-wire transfer.

<h4> init and exit functions: </h4>

1. init:

When "insmod ring_driver.ko" command is issued, "ringLed_probe()" function is triggered. 
Then it calls the function "ringLed_spi_init()".

<i> ringLed_spi_init(): </i> 

"ringLed_spi_init()" function initializes the spi driver using the function spi_register_driver().

2. exit:

<i> ringLed_remove(): </i>
When "rmmod ring_driver.ko" command is issued, "ringLed_remove()" function is triggered. 
"ringLed_remove()" function calls the function "ringLed_spi_exit()".

<i> ringLed_spi_exit(): </i>
"ringLed_spi_exit()" function frees the resources. It frees the gpios, unregisters the spi device, and destroy the classes and devices.


<h4> the test program (main.c) </h4>
In the test program, it opens the device. Then it calls the ioctl function to reset the spi device. 
It will ask user to input number of leds "n", color, and loops of circulating. Then the display routine circulation will run. The user program is calling the write() function to let the kernel light up a given led with given color intensities, based on piexl values.


<i> ./test </i>
This should run the test program using the /dev/ interface.



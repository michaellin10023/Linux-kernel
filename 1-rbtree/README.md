<h1><center><u> Red Black Tree </u></center></h1>

---

This part deals with building a kernel module that creates an RB tree and use a test program to  test it. 

<h3><u> Compilation Instructions</u></h3>

The repository contains the following files.

main.c - This is source code for the test program that operates in user space.

rbt530_drv.c - This is the source sode for the kernel driver module.

Makefile - This is the makefile for compiling both the user program and the kernel module.

Once you download the files, navigate into the directory that contains the three files. Now it is assumed that the sdk is already installed in your computer. The default directory for the set up is /opt/iot-devkit/1.7.2/sysroots/ which will contain the kernel and the i586-poky-linux-gcc. If you have installed your sdk in some other directory then make sure the path variable leads to your i586-poky-linux-gcc compiler and the make is given the right directory to the kernel. 

If that has been set, open the terminal and type in <b><i> make</i></b>
You should be able to see some files that are created in the same directory. 

Among this the items of interest are , [rbt530_drv.ko , test].

Once you get these files, You will have to transfer the files to the galileo board. 

Type in the following command.

<i> sudo scp rbt530_drv.ko root@192.168.1.5:/home/ </i>

<i> sudo scp test root@192.168.1.5:/home/ </i>

Replace the ip address with the static IP you have assigned to your board. 

Now that you have the files, you can install the module in. Type in the following command from the directory where you have the rbt530_drv.ko and the test 

<i> insmod rbt530_drv.kp </i>

Now the driver module has been installed. you can check this by listing out dev.

<i>ls /dev/rbt* </i>

If it has been installed correctly you should see rbt530_dev device.

Now run the test programe.

<i> ./test </i>

<h3> <u><i> About The Program </u></i></h3>

The program has all the aspects of the assignment working in it.

<h4> open</h4>  The Device can be opened successfully from the user program using the open function in the user progam.This can be seen in the rbt_driver_open function.

<h4> release </h4> The Device can be released by the userprogram using the close function in the user program.This can be seen in the rbt_driver_release.

<h4> write </h4> You can write data to the device from the user program. If you give in a new key and data it will add it to the tree and rebalance it. If you're giving an existing key with a new data, it replace the existing key's data with the new data and if you give in an existing key with data=0 it deletes the data. To accomplish this you can see in the driver source code search and insert function being implemented. Erase and replace is available as part of the rb_tree api. This can be seen in the rbt_driver_write function.

<h4> ioctl </h4> Using the ioctl the user program can set the direction of the read in the kernel. If it 0 or 1 the ioctl in the user program returns 0 otherwise it returns -1 and the error no is set to EINVAL. In the kernel space it sets a global varibale set_end to do  the corresponding value. This can be seen in the  rbt_ioctl function int the driver program.

<h4> read </h4> The rbt_driver read returns either the first or the last node in the rb_tree depending on whether the global varibale set_end is 0 or 1 respectively. It returns the data from the kernel to user space.


In the user program the <b><i>Populate<i></b> function is passed to each of the 4 pthreads created. Each thread adds 10 random key and data to the rb_tree. 

After populating the tree with the 40 objects, the 4 threads are used to carry out 100 random operations of read and write concurrently. The function<i><b> Randomly</i></b> is passed to the four threads. A global variable totalOperations is used to keep track of the number of operations carried out. 

In both the functions the threads are set to a real time priority policy with a random priority value between 0 to 99.

The randomly function also uses ioctl to randomly determine the direction of the read for each thread.

Once the above two operations are carried out , the threads terminate and the contents are printed out as key and data. 




<h1><center><u> Assignment 1 Part 1 - Red Black Tree </u></center></h1>

---
This part uses Kprobe to get the diagnostic information from the kernel module rbt_driver_write.

<h3><u> Compilation Instructions</u></h3>

The repository contains the following files.

main.c - This is source code for the test program that operates in user space.

rbt530_drv.c - This is the source sode for the kernel driver module.

RBprobe.c - this is the RBprobe kernel module.

Makefile - This is the makefile for compiling both the user program and the kernel module.

Once you download the files, navigate into the directory that contains the three files. Now it is assumed that the sdk is already installed in your computer. The default directory for the set up is /opt/iot-devkit/1.7.2/sysroots/ which will contain the kernel and the i586-poky-linux-gcc. If you have installed your sdk in some other directory then make sure the path variable leads to your i586-poky-linux-gcc compiler and the make is given the right directory to the kernel. 

If that has been set, open the terminal and type in <b><i> make</i></b>
You should be able to see some files that are created in the same directory. 

Among this the items of interest are , [rbt530_drv.ko,RBprobe.ko , test].

Once you get these files, You will have to transfer the files to the galileo board. 

Type in the following command.

<i> sudo scp rbt530_drv.ko root@192.168.1.5:/home/ </i>

<i> sudo scp RBprobe.ko root@192.168.1.5:/home/</i>

<i> sudo scp test root@192.168.1.5:/home/ </i>

Replace the ip address with the static IP you have assigned to your board. 

Now that you have the files, you can install the module in. Type in the following command from the directory where you have the rbt530_drv.ko and the test 

<i> insmod rbt530_drv.ko </i>

<i> insmod RBprobe.ko </i>

Now the driver modules have been installed. you can check this by listing out dev.

<i>ls /dev/rbt* </i>

<i> ls /dev/RBprobe </i>

If it has been installed correctly you should see rbt530_dev device.

Now run the test programe.

<i> ./test </i>

After typing in the above command you should either type in <i> rbt_driver_write </i> or <i> rbt_driver_read </i>.
<h3> <u><i> About The Program </u></i></h3>



<h4> open</h4>  The Device can be opened successfully from the user program using the open function in the user progam.This can be seen in the kprobe_driver_open function.

<h4> release </h4> The Device can be released by the userprogram using the close function in the user program.This can be seen in the kprobe_driver_release.

<h4> write </h4> You can write data to the device from the user program. To register a device 1 is set followed by the function name or the symbol name. This is concatenated and converted to a string and is sent to the kprobe_driver_write function. sending in a 0 unregisters it.

<h4> read </h4> The kprobe_driver_read the stored trace data back into user space. the trace data is stored in a structure called data_send.


In the user program a seperate thread asks the user to enter the function name or symbol name where you want to register the kprobe. Now this is the point of kprobe entry. you can type in either  rbt_driver_write or rbt_driver_read according to how it is mentioned in the assignment. If you want to enter some other place. You can set a global varibale in the rbt kernel driver , export the symbol and set that symbol as your entry point. 

Once the function name has been entered it will start reading the data from rb_probe kernel and it will print it out. The PID , Kprobe address and the time stamp is printed out. 

 the <b><i>Populate<i></b> function is passed to each of the 4 pthreads created. Each thread adds 10 random key and data to the rb_tree. 

After populating the tree with the 40 objects, the 4 threads are used to carry out 100 random operations of read and write concurrently. The function<i><b> Randomly</i></b> is passed to the four threads. A global variable totalOperations is used to keep track of the number of operations carried out. 

In both the functions the threads are set to a real time priority policy with a random priority value between 0 to 99.

The randomly function also uses ioctl to randomly determine the direction of the read for each thread.

Once the above two operations are carried out , the threads terminate and the contents are printed out as key and data. 

At the end, the kprobe is unregistered. To check for successfull unregistering you can send rmmod RBprobe and then rmmod rbt530_drv and you should be able to see successfull uninstallation.



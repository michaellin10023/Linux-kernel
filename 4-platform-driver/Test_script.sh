#!/bin/bash

#Configuring Sensor 1 -> HCSR0

sleep 1

echo -n "7" > /sys/class/HCSR/HCSR_0/trigger
echo -n "5" > /sys/class/HCSR/HCSR_0/echo
echo -n "3" > /sys/class/HCSR/HCSR_0/number_samples
echo -n "50" > /sys/class/HCSR/HCSR_0/sampling_period

sleep 1

#Reading the Configured Values of HCSR0
echo ""
echo "Trigger value: " | cat - /sys/class/HCSR/HCSR_0/trigger
echo ""
echo "Echo value : " | cat - /sys/class/HCSR/HCSR_0/echo
echo ""
echo "Number of Samples : " | cat - /sys/class/HCSR/HCSR_0/number_samples
echo ""
echo "Sampling Period Delta : " | cat - /sys/class/HCSR/HCSR_0/sampling_period
echo ""

sleep 1
                                                                
#Configuring the second sensor HCSR1
                                                                
echo -n "8" > /sys/class/HCSR/HCSR_1/trigger
echo -n "4" > /sys/class/HCSR/HCSR_1/echo                       
echo -n "3" > /sys/class/HCSR/HCSR_1/number_samples
echo -n "50" > /sys/class/HCSR/HCSR_1/sampling_period           
                                            
sleep 1                                                       
                                                   
#Reading the Configured values of HCSR-1             
echo ""                                     
echo "Trigger value: " | cat - /sys/class/HCSR/HCSR_1/trigger
echo ""                                            
echo "Echo value : " | cat - /sys/class/HCSR/HCSR_1/echo
echo ""                                     
echo "Number of Samples : " | cat - /sys/class/HCSR/HCSR_1/number_samples
echo ""                                            
echo "Sampling Period Delta : " | cat - /sys/class/HCSR/HCSR_1/sampling_period
echo ""
                                                                
sleep 1

#Displaying the latest distance values of Both Sensors before enabling measurement

echo "HCSR_0 Distance : " | cat - /sys/class/HCSR/HCSR_0/distance
echo "HCSR_1 Distance : " | cat - /sys/class/HCSR/HCSR_1/distance
sleep 1

#Enabling measurement for sensor 1
echo -n "1" > /sys/class/HCSR/HCSR_0/enable
#Reading the enable for sensor 1
echo "Enable HCSR0 : " | cat - /sys/class/HCSR/HCSR_0/enable
sleep 1

#Displaying the latest distance values of Both Sensors after enabling measurement for sensor 1 

echo "HCSR_0 Distance : " | cat - /sys/class/HCSR/HCSR_0/distance
sleep 1
echo "HCSR_1 Distance : " | cat - /sys/class/HCSR/HCSR_1/distance
sleep 1

#Enabling measurement for sensor 2
echo -n "1" > /sys/class/HCSR/HCSR_1/enable
#Reading the enable for sensor 1
echo "Enable HCSR1 : " | cat - /sys/class/HCSR/HCSR_1/enable
sleep 2

count=0

#Reading the latest readings measured by both sensors 
while [ $count -lt 20 ] 
do
echo "HCSR_0 Distance : " | cat - /sys/class/HCSR/HCSR_0/distance
sleep 1
echo "HCSR_1 Distance : " | cat - /sys/class/HCSR/HCSR_1/distance 
let count+=1

done 

#Disabling measurement
echo -n "0" > /sys/class/HCSR/HCSR_0/enable
echo -n "0" > /sys/class/HCSR/HCSR_1/enable
sleep 1

	

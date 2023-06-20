#!/bin/sh
#https://www.analog.com/media/cn/technical-documentation/user-guides/AD9364_Register_Map_Reference_Manual_UG-672.pdf

ptton()
{
    #PTT on GPIO 0 AND GPIO 2 (GPIO 1 should be not touched)
	echo 0x27 0x50 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
	mosquitto_pub -t plutodvb/status/tx -m true
}

pttoff()
{
        echo 0x27 0x00 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
		mosquitto_pub -t plutodvb/status/tx -m false
}


echo manual_tx_quad > /sys/bus/iio/devices/iio:device0/calib_mode
#Manual GPIO
echo 0x26 0x10 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
pttoff

loop()
{
while :
do
inotifywait -e modify /sys/bus/iio/devices/iio:device0/out_altvoltage1_TX_LO_powerdown
txmute=$(cat /sys/bus/iio/devices/iio:device0/out_altvoltage1_TX_LO_powerdown)
if [ "$txmute" = "1" ] ; then
echo "SdrConsole PTT OFF"
pttoff
else
       
   echo "SdrConsole PTT ON"
   ptton   
fi
done
}

loop

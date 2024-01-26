
adphys="$(cat /sys/bus/iio/devices/iio:device0/name)"

if [ "$adphys" = "ad9361-phy" ] ; then
echo manual_tx_quad > /sys/bus/iio/devices/iio:device0/calib_mode
echo -89 > /sys/bus/iio/devices/iio:device0/out_voltage0_hardwaregain
else
echo manual_tx_quad > /sys/bus/iio/devices/iio:device1/calib_mode
echo -89 > /sys/bus/iio/devices/iio:device1/out_voltage0_hardwaregain
fi

devmem 0x43c00040 32 0x80000000
devmem 0x43c00044 32 0x00000000
devmem 0x43c00000 32 0x00000002
devmem 0x43c20040 32 0x00000001
devmem 0x43c20000 32 0x00000002

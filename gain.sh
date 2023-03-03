key=$(cat /sys/kernel/config/usb_gadget/composite_gadget/strings/0x409/serialnumber)
dt_root="dt/pluto/$key"
cmd_root="cmd/pluto/$key"


maxgain=-18



if [[ $(echo "($1) < ($maxgain)" |bc -l) -ge 1 ]]; then
#$(mosquitto_pub -t $cmd_root/tx/dvbs2/agcgain -m $1)
$(mosquitto_pub -t $cmd_root/tx/gain -m $1)
else
echo "Over max gain"
fi




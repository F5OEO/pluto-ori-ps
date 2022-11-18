key=$(cat /sys/kernel/config/usb_gadget/composite_gadget/strings/0x409/serialnumber)
dt_root="dt/pluto/$key"
cmd_root="cmd/pluto/$key"
duration=2

rollcoderatelong () {
for rate in 0 1 2 3 4 5 6 7 8 9 10
do    
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/coderate -m $rate)
    echo $rate
     $(mosquitto_pub -t $cmd_root/tx/dvbs2/frame -m long)
    sleep $duration
done    
}

rollcoderateshort () {
for rate in 0 1 2 3 4 5 6 7 8 9
do    
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/coderate -m $rate)
    echo $rate
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/frame -m short)
    sleep $duration
done    
}

for constel in "qpsk" "8psk"
do
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/constel -m $constel)
    echo $constel
   
    echo longframe
    rollcoderatelong
   
    echo shortframe
    rollcoderateshort
done



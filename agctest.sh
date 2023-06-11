key=$(fw_printenv -n call)
dt_root="dt/pluto/$key"
cmd_root="cmd/pluto/$key"
duration=5

$(mosquitto_pub -t $cmd_root/tx/stream/mode -m dvbs2-ts)
#$(mosquitto_pub -t $cmd_root/tx/stream/mode -m dvbs2-gse)

$(mosquitto_pub -t $cmd_root/tx/frequency -m 1255e6)
$(mosquitto_pub -t $cmd_root/tx/dvbs2/sr -m 333000)
$(mosquitto_pub -t $cmd_root/tx/dvbs2/agcgain -m -25.0)

rollcoderatelongqpsk () {
for rate in 14 13 25 12 35 23 34 45 56 89 910
#for rate in 0 1 3 4 5 6 8 10 
do  
     $(mosquitto_pub -t $cmd_root/tx/dvbs2/constel -m qpsk)
     $(mosquitto_pub -t $cmd_root/tx/dvbs2/frame -m long)
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/fec -m $rate)
    echo $rate
     
#    ./regs.sh
    sleep $duration
done    
}

rollcoderateshortqpsk () {
for rate in 14 13 25 12 35 23 34 45 56 89 
#for rate in  8 9
do    
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/fec -m $rate)
    echo $rate
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/frame -m short)
    sleep $duration
done    
}

rollcoderatelong8psk () {
for rate in 35 23 34 56 89 910 

do    
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/fec -m $rate)
    echo $rate
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/constel -m 8psk)
     $(mosquitto_pub -t $cmd_root/tx/dvbs2/frame -m long)
    sleep $duration
done    
}

rollcoderateshort8psk () {
for rate in 35 23 34 56 89 
do    
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/fec -m $rate)
    echo $rate
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/constel -m 8psk)
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/frame -m short)
    sleep $duration
done    
}

rollcoderatelong16apsk() {
for rate in 23 34 45 56 89 910 
do    
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/fec -m $rate)
    echo $rate
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/constel -m 16apsk)
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/frame -m long)
    sleep $duration
done    
}
rollcoderateshort16apsk() {
for rate in 23 34 45 56 89 
do    
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/fec -m $rate)
    echo $rate
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/constel -m 16apsk)
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/frame -m short)
    sleep $duration
done    
}

rollcoderatelong32apsk() {
for rate in 34 45 56 89 910 
do    
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/fec -m $rate)
    echo $rate
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/constel -m 32apsk)
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/frame -m long)
    sleep $duration
done    
}
rollcoderateshort32apsk() {
for rate in 34 45 56 89
do    
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/fec -m $rate)
    echo $rate
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/constel -m 32apsk)
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/frame -m short)
    sleep $duration
done    
}

rollall() {
    echo rollcoderatelongqpsk
    rollcoderatelongqpsk
    echo rollcoderateshortqpsk
    rollcoderateshortqpsk
   
}


    $(mosquitto_pub -t $cmd_root/tx/dvbs2/constel -m qpsk)
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/pilots -m 1)
    rollall
    


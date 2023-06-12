key=$(cat /sys/kernel/config/usb_gadget/composite_gadget/strings/0x409/serialnumber)
dt_root="dt/pluto/$key"
cmd_root="cmd/pluto/$key"
dt_longmynd="dt/longmynd"
duration=5
gain=-30

waitlock()
{
    while :
    do
    bch=$(mosquitto_sub -t $dt_longmynd/bch_uncorect -C 1)
    
    if [ "$bch" = "1" ]; then
        mer=$(mosquitto_sub -t $dt_longmynd/mer -C 1)
        fec=$(mosquitto_sub -t $dt_longmynd/fec -C 1)
        short=$(mosquitto_sub -t $dt_longmynd/short_frame -C 1)
         >&2 echo "Lock on mer $mer"
         totalmer=$mer;
        for i in `seq 1 9`;
        do
            mer=$(mosquitto_sub -t $dt_longmynd/mer -C 1)
            #>&2 echo "Averaging mer $mer"
            totalmer=$(echo "($totalmer) + ($mer)" |bc -l )
            done
        averagemer=$(echo "scale=1;($totalmer) / (10.0)" |bc -l )
        >&2 echo "Average mer $averagemer "
        if [ "$short" = "1" ]; then
            echo -n "$fec;short;$averagemer;"
        else
            echo -n "$fec;long;$averagemer;"
        fi
        
        gain=$(echo "($gain) - 3.0" |bc )
        ./gain.sh $gain
        #cat ";" > result.txt
        break;
    else
        gain=$(echo "($gain) + 0.5" |bc )
        
        ./gain.sh $gain
        #sleep 2
         
    fi    
    done
}

resetgain()
{
        gain=-30
     ./gain.sh $gain
     sleep 1
}

rollcoderatelongqpsk () {
for rate in 0 1 2 3 4 5 6 7 8 9 10
#for rate in 0 1 3 4 5 6 8 10 
do  
     $(mosquitto_pub -t $cmd_root/tx/dvbs2/constel -m qpsk)
     $(mosquitto_pub -t $cmd_root/tx/dvbs2/frame -m long)
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/coderate -m $rate)
    #echo "long FEC $rate"
    waitlock     

done    
}

rollcoderateshortqpsk () {
for rate in 0 1 2 3 4 5 6 7 8 9
#for rate in  8 9
do    
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/constel -m qpsk)
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/frame -m short)
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/coderate -m $rate)
    #echo "short FEC $rate"
    waitlock     
  
    sleep $duration
done    
}



$(mosquitto_pub -t $cmd_root/tx/gain -m $gain)
$(mosquitto_pub -t $cmd_root/tx/mute -m 0)

$(mosquitto_pub -t $cmd_root/tx/frequency -m 2404.750e6)
$(mosquitto_pub -t $cmd_root/tx/dvbs2/sr -m 333000)

$(mosquitto_pub -t $cmd_root/tx/stream/mode -m dvbs2-ts)
$(mosquitto_pub -t $cmd_root/tx/dvbs2/frame -m long)
$(mosquitto_pub -t $cmd_root/tx/dvbs2/constel -m qpsk)
$(mosquitto_pub -t $cmd_root/tx/dvbs2/coderate -m 0)

while :
    do
>&2 echo -n "Date $(date) ;"
echo -n "Date $(date);"
resetgain
rollcoderatelongqpsk >> test.csv
resetgain
rollcoderateshortqpsk >> test.csv   
echo >> test.csv 

done

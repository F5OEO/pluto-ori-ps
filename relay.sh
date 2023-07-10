key=$(fw_printenv -n call)
dt_longmynd="dt/longmynd"
cmd_root="cmd/pluto/$key"
dtrelay="dt/pluto/$key/relay"
cmdrelay="cmd/pluto/$key/relay"
waitlock()
{
    modulation=$(mosquitto_sub -t $dt_longmynd/modulation -C 1)
    while [ "$modulation" != "none" ]
    do
    station=$(mosquitto_sub -t $dt_longmynd/service_name -C 1)
    mer=$(mosquitto_sub -t $dt_longmynd/mer -C 1)
    fec=$(mosquitto_sub -t $dt_longmynd/fec -C 1)
    $(mosquitto_pub -t $dtrelay/status -m "lock")
    $(mosquitto_pub -t $dtrelay/station -m $station)
    $(mosquitto_pub -t $dtrelay/mer -m $mer)

    echo "lock with mer $mer"
    $(mosquitto_pub -t $cmd_root/tx/mute -m 0)
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/sdt -m $station"-via-")
    if [ "$fecmode" == "follow" ] && [ "$fec" != "none" ]
    then
        $(mosquitto_pub -t $cmd_root/tx/dvbs2/fec -m $fec)
    fi
    modulation=$(mosquitto_sub -t $dt_longmynd/modulation -C 1)
    done
    echo unlock
    $(mosquitto_pub -t $cmd_root/tx/mute -m 1)
}

scan()
{
    for sr in 125 250 333 500 1000
    do
        $(mosquitto_pub -t cmd/longmynd/sr -m $sr)
        $(mosquitto_pub -t $dtrelay/status -m "scan $sr")
        srfull=""$sr"000"
	echo $srfull
 $(mosquitto_pub -t $cmd_root/tx/dvbs2/sr -m $srfull)
        sleep 3
        modulation=$(mosquitto_sub -t $dt_longmynd/modulation -C 1)
        if [ "$modulation" != "none" ] ; then
            
            $(mosquitto_pub -t $cmd_root/tx/dvbs2/sr -m $srfull)
             
            waitlock
        fi
    done

}

inputfrequency()
{
    while :
    do
    frequency=$(mosquitto_sub -t $cmdrelay/infrequency -C 1)
    echo "$frequency relay"
    $(mosquitto_pub -t $dtrelay/infrequency -m $frequency)
    $(mosquitto_pub -t cmd/longmynd/frequency -m $frequency)
    done

}

FecMode()
{
    while :
    do
    fecmode=$(mosquitto_sub -t $cmdrelay/fecmode -C 1)
    if [ "$fecmode" == "follow" ]
    then
        $(mosquitto_pub -t $cmd_root/tx/dvbs2/fecmode -m fixed)
    else
        $(mosquitto_pub -t $cmd_root/tx/dvbs2/fec -m 1/4)
        $(mosquitto_pub -t $cmd_root/tx/dvbs2/fecmode -m variable)
    fi
    done    
}

exit_script() {
    trap - SIGINT SIGTERM # clear the trap
	$(mosquitto_pub -t $cmd_root/tx/mute -m 1)
    kill -- -$$ # Sends SIGTERM to child/sub processes
}

init()
{
    $(mosquitto_pub -t $cmd_root/tx/mute -m 1)
    $(mosquitto_pub -t cmdrelay/fecmode -m follow)
    $(mosquitto_pub -t cmd/longmynd/frequency -m 747750)
    $(mosquitto_pub -t cmd/longmynd/swport -m 0)
    $(mosquitto_pub -t cmd/longmynd/tsip -m 230.0.0.2)
}

trap exit_script SIGINT SIGTERM
fecmode=follow
FecMode &
inputfrequency &

while :
do
scan
done

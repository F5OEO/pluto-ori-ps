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
    $(mosquitto_pub -t $dtrelay/status -m "lock")
    $(mosquitto_pub -t $dtrelay/station -m $station)
    $(mosquitto_pub -t $dtrelay/mer -m $mer)

    echo "lock with mer $mer"
    $(mosquitto_pub -t $cmd_root/tx/mute -m 0)
    $(mosquitto_pub -t $cmd_root/tx/dvbs2/sdt -m "$station via")
    modulation=$(mosquitto_sub -t $dt_longmynd/modulation -C 1)
    done
    echo unlock
    $(mosquitto_pub -t $cmd_root/tx/mute -m 1)
}

scan()
{
    for sr in 125 250 333 500
    do
        $(mosquitto_pub -t cmd/longmynd/sr -m $sr)
        $(mosquitto_pub -t $dtrelay/status -m "scan $sr")
        echo  ""$sr"000"
        sleep 3
        modulation=$(mosquitto_sub -t $dt_longmynd/modulation -C 1)
        if [ "$modulation" != "none" ] ; then
            
            $(mosquitto_pub -t $cmd_root/dvbs2/sr -m ""$sr"000")
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

exit_script() {
    trap - SIGINT SIGTERM # clear the trap
    kill -- -$$ # Sends SIGTERM to child/sub processes
}

 echo unlock
$(mosquitto_pub -t $cmd_root/tx/mute -m 1)
$(mosquitto_pub -t cmd/longmynd/frequency -m 747750)
$(mosquitto_pub -t cmd/longmynd/swport -m 0)
$(mosquitto_pub -t cmd/longmynd/tsip -m 230.0.0.2)
trap exit_script SIGINT SIGTERM
inputfrequency &
while :
do
scan
done
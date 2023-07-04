key=$(fw_printenv -n call)
dt_longmynd="dt/longmynd"
cmd_root="cmd/pluto/$key"

waitlock()
{
    modulation=$(mosquitto_sub -t $dt_longmynd/modulation -C 1)
    while [ "$modulation" != "none" ]
    do
    mer=$(mosquitto_sub -t $dt_longmynd/mer -C 1)
    echo "lock with mer $mer"
    srfound=$(mosquitto_sub -t $dt_longmynd/symbolrate -C 1)
    $(mosquitto_pub -t $cmd_root/tx/mute -m 0)
    done
    echo unlock
    $(mosquitto_pub -t $cmd_root/tx/mute -m 1)
}

scan()
{
    for sr in 125 250 333 500
    do
        $(mosquitto_pub -t cmd/longmynd/sr -m $sr)
        sleep 2
        modulation=$(mosquitto_sub -t $dt_longmynd/modulation -C 1)
        if [ "$modulation" != "none" ] ; then
            waitlock
        fi
    done

}

$(mosquitto_pub -t cmd/longmynd/frequency -m 747250)
$(mosquitto_pub -t cmd/longmynd/swport -m 0)
$(mosquitto_pub -t cmd/longmynd/tsip -m 230.0.0.2)

while :
do
scan
done

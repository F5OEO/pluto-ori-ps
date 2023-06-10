key=$(fw_printenv -n call)
dt_root="dt/pluto/$key"
cmd_root="cmd/pluto/$key"


$(mosquitto_pub -t $cmd_root/tx/mute -m 1)

key=$(fw_printenv -n call)
dt_root="dt/pluto/$key"
cmd_root="cmd/pluto/$key"

echo "Setmodtx to $1"

$(mosquitto_pub -t $cmd_root/tx/stream/mode -m $1)

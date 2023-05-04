cmd_root="cmd/longmynd"


$(mosquitto_pub -t $cmd_root/frequency -m $1)
$(mosquitto_pub -t $cmd_root/sr -m $2)





cmd_root="cmd/pluto"

while :
do
param=$(mosquitto_sub -t $cmd_root/call -C 1)
fw_setenv call $param
done
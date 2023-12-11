key=$(fw_printenv -n call)
dt_root="dt/pluto/$key"
cmd_root="cmd/pluto/$key"

while :
do
param=$(mosquitto_sub -t $cmd_root/ip/route -C 1)
route $param
#route add -net 44.0.0.0/24 gw $param
done
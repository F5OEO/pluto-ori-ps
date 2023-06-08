key=$(fw_printenv -n call)
dt_root="dt/pluto/$key"
cmd_root="cmd/pluto/$key"

iptables -F
iptables -t nat -F

while :
do
param=$(mosquitto_sub -t $cmd_root/ip/tunadress -C 1)
#echo $param
#tunctl -t gse
ifconfig gse0 up
ifconfig gse0 $param
route add -net 44.0.0.0/24 gw $param
done
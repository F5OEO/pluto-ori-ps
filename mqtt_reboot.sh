key=$(fw_printenv -n call)
dt_root="dt/pluto/$key"
cmd_root="cmd/pluto/$key"

iptables -F
iptables -t nat -F

while :
do
param=$(mosquitto_sub -t $cmd_root/system/reboot -C 1)
#echo $param
reboot&
done
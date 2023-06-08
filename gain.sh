key=$(fw_printenv -n call)
dt_root="dt/pluto/$key"
cmd_root="cmd/pluto/$key"


maxgain=0



if [[ $(echo "($1) < ($maxgain)" |bc -l) -ge 1 ]]; then
#$(mosquitto_pub -t $cmd_root/tx/dvbs2/agcgain -m $1)
$(mosquitto_pub -t $cmd_root/tx/gain -m $1)
else
echo "Over max gain"
fi




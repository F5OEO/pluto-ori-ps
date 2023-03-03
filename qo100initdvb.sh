key=$(cat /sys/kernel/config/usb_gadget/composite_gadget/strings/0x409/serialnumber)
dt_root="dt/pluto/$key"
cmd_root="cmd/pluto/$key"

$(mosquitto_pub -t $cmd_root/tx/gain -m -25)
$(mosquitto_pub -t $cmd_root/tx/mute -m 0)

#$(mosquitto_pub -t $cmd_root/tx/frequency -m 2404.750e6)
$(mosquitto_pub -t $cmd_root/tx/frequency -m 2406.250e6)
$(mosquitto_pub -t $cmd_root/tx/dvbs2/sr -m 333000)

#$(mosquitto_pub -t $cmd_root/tx/stream/mode -m dvbs2-ts)
$(mosquitto_pub -t $cmd_root/tx/stream/mode -m dvbs2-gse)
$(mosquitto_pub -t $cmd_root/tx/dvbs2/frame -m long)
$(mosquitto_pub -t $cmd_root/tx/dvbs2/constel -m qpsk)
$(mosquitto_pub -t $cmd_root/tx/dvbs2/coderate -m 0)
#$(mosquitto_pub -t $cmd_root/tx/dvbs2/agcgain -m -30.0)
$(mosquitto_pub -t $cmd_root/tx/dvbs2/fecmode -m variable)

#sysctl -w net.ipv4.ip_forward=1
sleep 4
route add -net 44.0.0.0/24 gw 44.0.0.2

iptables -A FORWARD -i eno1 -o gse0 -d 44.0.0.2/24 -j ACCEPT
iptables -A FORWARD -i lo -o gse0 -d 44.0.0.2/24 -j ACCEPT
iptables -t nat -A POSTROUTING -o gse0  -j MASQUERADE
iptables -A FORWARD -i eth0 -o gse0 -m state --state ESTABLISHED,RELATED -j ACCEPT
iptables -t nat -A PREROUTING -p tcp --dport 8090 -j DNAT --to-destination 192.168.1.39:9090

#key=$(cat /sys/kernel/config/usb_gadget/composite_gadget/strings/0x409/serialnumber)
key=$(fw_printenv -n call)
dt_root="dt/pluto/$key"
cmd_root="cmd/pluto/$key"
#tun ip address PLuto
tunip="44.0.0.4"
#Default PC to forward
pcforward="192.168.1.39"
#Web server on an other PC
pcforward2="192.168.1.70"
#Mcast address to receive bbframe from longmynd
Mcast=230.0.0.2

$(mosquitto_pub -t $cmd_root/tx/gain -m -20)
$(mosquitto_pub -t $cmd_root/tx/mute -m 0)

$(mosquitto_pub -t $cmd_root/tx/frequency -m 1255e6)
$(mosquitto_pub -t $cmd_root/tx/dvbs2/sr -m 333000)
$(mosquitto_pub -t $cmd_root/tx/nco -m 000)


#$(mosquitto_pub -t $cmd_root/tx/stream/mode -m test)
#$(mosquitto_pub -t $cmd_root/tx/stream/mode -m dvbs2-ts)
$(mosquitto_pub -t $cmd_root/tx/stream/mode -m dvbs2-gse)

$(mosquitto_pub -t $cmd_root/tx/dvbs2/fec -m 5/6)
$(mosquitto_pub -t $cmd_root/tx/dvbs2/constel -m qpsk)
$(mosquitto_pub -t $cmd_root/tx/dvbs2/pilots -m 1)
$(mosquitto_pub -t $cmd_root/tx/dvbs2/frame -m short)
#$(mosquitto_pub -t $cmd_root/tx/dvbs2/frame -m long)
#$(mosquitto_pub -t $cmd_root/tx/dvbs2/fecmode -m variable)
#$(mosquitto_pub -t $cmd_root/tx/dvbs2/agcgain -m -25.0)


$(mosquitto_pub -t $cmd_root/rx/frequency -m 745e6)
$(mosquitto_pub -t $cmd_root/rx/gain -m 45.0)

# ****************************
# Receiver Lognmynd for F5OEO with mqtt
# ****************************
#./setrx.sh 744750 1000
$(mosquitto_pub -t $cmd_root/tx/dvbs2/rxbbframeip -m $Mcast:1234)
$(mosquitto_pub -t $cmd_root/ip/tunadress -m "$tunip")


#ifconfig gse0 $tunip
#route add -net 44.0.0.0/24 gw $tunip


$(mosquitto_pub -t $cmd_root/ip/iptables -m "-t nat -F")
sleep 0.1
$(mosquitto_pub -t $cmd_root/ip/iptables -m "-F")
sleep 0.1
$(mosquitto_pub -t $cmd_root/ip/iptables -m "-A FORWARD -p udp -o gse0 -s 44.0.0.0/24 -j DROP")
sleep 0.1
$(mosquitto_pub -t $cmd_root/ip/iptables -m "-t nat -A POSTROUTING -o gse0  -j MASQUERADE")
sleep 0.1
$(mosquitto_pub -t $cmd_root/ip/iptables -m "-t nat -A POSTROUTING -o eth0  -j MASQUERADE")
sleep 0.1
$(mosquitto_pub -t $cmd_root/ip/iptables -m "-t nat -A PREROUTING -p tcp -i gse0 --dport 80 -j DNAT --to-destination $pcforward2:80")
sleep 0.1
$(mosquitto_pub -t $cmd_root/ip/iptables -m "-t nat -A PREROUTING -p udp -i gse0 --dport 1000:10000 -j DNAT --to-destination $pcforward:1000-10000")




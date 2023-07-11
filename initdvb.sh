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

$(mosquitto_pub -t $cmd_root/tx/frequency -m 2404e6)
$(mosquitto_pub -t $cmd_root/tx/dvbs2/sr -m 333000)
$(mosquitto_pub -t $cmd_root/tx/nco -m 000)


#$(mosquitto_pub -t $cmd_root/tx/stream/mode -m test)
$(mosquitto_pub -t $cmd_root/tx/stream/mode -m dvbs2-ts)
#$(mosquitto_pub -t $cmd_root/tx/stream/mode -m dvbs2-gse)

#udp source
#$(mosquitto_pub -t $cmd_root/tx/dvbs2/tssourcemode -m 0)
#$(mosquitto_pub -t $cmd_root/tx/dvbs2/tssourceaddress -m 230.0.0.10:10000)
#filesource
#$(mosquitto_pub -t $cmd_root/tx/dvbs2/tssourcemode -m 1)
#$(mosquitto_pub -t $cmd_root/tx/dvbs2/tssourcefile -m /root/remote/pluto_dvb/f9zg.ts)
#$(mosquitto_pub -t $cmd_root/tx/dvbs2/tssourcefile -m /root/remote/pluto_dvb/bunny450.ts)
#$(mosquitto_pub -t $cmd_root/tx/dvbs2/tssourcefile -m /root/remote/pluto_dvb/dh1rk.ts)
#pattern
#$(mosquitto_pub -t $cmd_root/tx/dvbs2/tssourcemode -m 2)


$(mosquitto_pub -t $cmd_root/tx/dvbs2/fec -m 2/3)
$(mosquitto_pub -t $cmd_root/tx/dvbs2/constel -m qpsk)
$(mosquitto_pub -t $cmd_root/tx/dvbs2/pilots -m 1)
$(mosquitto_pub -t $cmd_root/tx/dvbs2/frame -m short)
#$(mosquitto_pub -t $cmd_root/tx/dvbs2/frame -m long)
$(mosquitto_pub -t $cmd_root/tx/dvbs2/fecmode -m variable)
#digital gain, max 3db
#$(mosquitto_pub -t $cmd_root/tx/dvbs2/digitalgain -m 2.0)

#$(mosquitto_pub -t $cmd_root/tx/dvbs2/fecrange -m 5)
#$(mosquitto_pub -t $cmd_root/tx/dvbs2/gainvariable -m 1)


$(mosquitto_pub -t $cmd_root/rx/frequency -m 2404e6)
$(mosquitto_pub -t $cmd_root/rx/gain -m 45.0)

#$(mosquitto_pub -t $cmd_root/rx/stream/mode -m pass)
#To see waterfall on the web interface
#$(mosquitto_pub -t $cmd_root/rx/stream/mode -m webfft)


# ****************************
# Receiver Lognmynd for F5OEO with mqtt
# ****************************

$(mosquitto_pub -t $cmd_root/system/longmynd -m on)
sleep 1
$(mosquitto_pub -t cmd/longmynd/frequency -m 2404000)
$(mosquitto_pub -t cmd/longmynd/sr -m 333)
$(mosquitto_pub -t cmd/longmynd/swport -m 0)
$(mosquitto_pub -t cmd/longmynd/tsip -m 230.0.0.2)

$(mosquitto_pub -t $cmd_root/tx/dvbs2/rxbbframeip -m $Mcast:1234)
$(mosquitto_pub -t $cmd_root/ip/tunadress -m "$tunip")

# RELAY MODE : COMMAND longmynd and ptt

#$(mosquitto_pub -t $cmd_root/relay/infrequency -m 74725)
#$(mosquitto_pub -t $cmd_root/relay/fecmode -m follow)

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




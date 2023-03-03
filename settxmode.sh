key=$(cat /sys/kernel/config/usb_gadget/composite_gadget/strings/0x409/serialnumber)
dt_root="dt/pluto/$key"
cmd_root="cmd/pluto/$key"

echo "Setmodtx to $1"

$(mosquitto_pub -t $cmd_root/tx/stream/mode -m $1)

# knockitoff
Trigger an event based on volume

On an orbi, at least, you can turn off internet to a computer with this command:

iptables -D FORWARD -m mac --mac-source <mac address> -j REJECT

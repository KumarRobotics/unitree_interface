# unitree_interface
hardware version: 2.0

software version: 1.0.29

unitree_ros2 commit: 66ae09858245ac3d2231c0cc209e36a88f8d7d03


## Network Configuration
1. Configure ethernet interface for Unitree
```
nmcli connection add type ethernet ifname <interface_name> con-name unitree_eth
nmcli connection modify unitree_eth ipv4.addresses 192.168.123.99/24
nmcli connection modify unitree_eth ipv4.gateway 192.168.123.1
nmcli connection modify unitree_eth ipv4.dns 8.8.8.8
nmcli connection modify unitree_eth ipv4.method manual
nmcli connection up unitree_eth
```

2. Configure ethernet interface for Ouster
```
nmcli connection add type ethernet ifname <interface_name> con-name os_eth
nmcli connection modify os_eth ipv4.addresses 192.168.100.10/24
nmcli connection modify os_eth ipv4.gateway 192.168.100.1
nmcli connection modify os_eth ipv4.dns 8.8.8.8
nmcli connection modify os_eth ipv4.method manual
nmcli connection up os_eth
```

# unitree_interface
hardware version: 2.0

software version: 1.0.29

unitree_ros2 commit: 66ae09858245ac3d2231c0cc209e36a88f8d7d03


## Network Configuration
1. Configure ethernet interface for Unitree
```
UNITREE_ETH_INTERFACE=<interface_name>
sudo nmcli connection add type ethernet ifname $UNITREE_ETH_INTERFACE con-name unitree_eth
sudo nmcli connection modify unitree_eth ipv4.addresses 192.168.123.99/24
sudo nmcli connection modify unitree_eth ipv4.gateway 192.168.123.1
sudo nmcli connection modify unitree_eth ipv4.dns 8.8.8.8
sudo nmcli connection modify unitree_eth ipv4.method manual
sudo nmcli connection up unitree_eth
```

2. Configure ethernet interface for Ouster
```
OS_ETH_INTERFACE=<interface_name>
sudo nmcli connection add type ethernet ifname $OS_ETH_INTERFACE con-name os_eth
sudo nmcli connection modify os_eth ipv4.addresses 192.168.100.10/24
sudo nmcli connection modify os_eth ipv4.gateway 192.168.100.1
sudo nmcli connection modify os_eth ipv4.dns 8.8.8.8
sudo nmcli connection modify os_eth ipv4.method manual
sudo nmcli connection up os_eth
```

## Launch
1. Launch unitree_teleop
```
ros2 launch unitree_teleop unitree_teleop_cyclonedds.launch.py
```
2. Launch Ouster and LIO
```
ros2 launch unitree_interface ouster128_rko_lio.launch.py
```
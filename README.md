## Description


**ROS 1** package for receiving CRSF (RC channels values) packets over serial port (UART).


**CRSF protocol** packet format [description](https://github.com/crsf-wg/crsf/wiki/Message-Format).


### Topics
    
- `rc/channels` - received rc channels values
- `rc/link` - connection statistics information


---

## Installation



### Dependencies:

Install `CppLinuxSerial` before:

```bash
git clone https://github.com/gbmhunter/CppLinuxSerial.git

cd CppLinuxSerial
mkdir build/
cd build
cmake ..
make
sudo make install
cd ../..
```


Let's assume that your ros workspace localized at `~/ros_ws/`.


### 1. Clone package from git:

```bash
cd ~/row2_ws/src

# Types dependency package:
git clone https://github.com/wanghuohuo0716/ros_crsf_receiver.git
```

### 2. Build

```bash
cd ~/ros_ws

colcon build --packages-select crsf_receiver_msg
colcon build --packages-select crsf_receiver
```

### 3. Re-source

```bash
source ~/ros_ws/devel/setup.bash
```

---



## Running


### Set up params:

**!!note!!**: **normally the receiver baund rate is 420000, but linux cutcom etc serial software can't resovle so fast baund rate data, so you have to change the receiver buand rate to a lower baund rate, now I set the baund rate 115200 for easy test.**

It's easy to set the receiver baund rate, don't worry!

1. Serial device name: `device`, default is `/dev/ttyUSB0`
2. Baud rate: `baud_rate`, default is `115200`
3. Enable / Disable link statistics info: `link_stats`, default is `false`
4. Receiver rate (hz): `receiver_rate`, default is `100`


### Run ros node:

```bash
# Run Node with default parameters
source devel/setup.bash
rosrun crsf_receiver crsf_receiver_node

# Or setup and run Node with custom parameters values:
rosrun crsf_receiver crsf_receiver_node --ros-args -p "device:=/dev/serial0" -p baud_rate:=420000  -p link_stats:=true
```

### Check

After correct setup and running without errors you can check topics:

```bash
# Check channels values
rostopic echo /rc/channels

# Check link statisics
rostopic echo /rc/link

# Check receiver rate
rostopic hz /rc/channels
```



###  Link statistics message fields:

- `uplink_rssi_ant1` - ( dBm * -1 )
- `uplink_rssi_ant2` - ( dBm * -1 )
- `uplink_status` - Package success rate / Link quality ( % )
- `uplink_snr` - ( db )
- `active_antenna` - Diversity active antenna ( enum ant. 1 = 0, ant. 2 )
- `rf_mode` - ( enum 4fps = 0 , 50fps, 150hz)
- `uplink_tx_power` - ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW )
- `downlink_rssi` - ( dBm * -1 )
- `downlink_status` - Downlink package success rate / Link quality ( % )
- `downlink_snr` - ( db )

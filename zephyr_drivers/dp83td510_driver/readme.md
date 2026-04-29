# DP83TD510 10BASE-T1L PHY Ethernet Zephyr Driver
  
This document describes how to setup and test DP83TD510E driver using Zephyr OS.
The driver was validated on Zephyr repository v4.4 and SDK 0.17.2 and 0.17.4 and 1.0.0
## 1. Repository Structure
```bash
driver_files/
├─ board_dts/
│   ├─ host_board.dts
│ 
├─ drivers/
│   └─ phy/
│       ├─ Kconfig
│       ├─ phy_ti_dp83td510.c
│       └─ CMakeLists.txt
│
├─ dts/
│   └─ bindings/
│       └─ phy/
│           └─ ti,dp83td510.yaml
└─
```

### 1.1 Key File Notes
  
Device Tree (*.dts): Defines tree integrated with Zephyr Driver enabled
Driver Source: Enables PHY interface with SoC
DTS Bindings: Defines required properties for the driver in the DTS

## 2. Features and Device Tree

### 2.1 Supported Features

This driver supports the following features:

- **10BASE-T1L Single Pair Ethernet**: Full support for IEEE 802.3cg single twisted pair communication
- **SMI Access**: MDIO/MDC access for register configuration
- **RMII Leader & Follower**: DTS configuration to enable RMII leader or follower mode.
- **Voltage Swing Negotiation & Config**: Driver API to retrieve and/or set voltage swing between 2.4V and 1.0V p2p 


**Device Tree Properties:**

| Property | Type | Description |
|----------|------|-------------|
| `compatible` | string | Device identification - must be "ti,dp83td510" |
| `int-gpios` | phandle | GPIO pin for interrupt signal (ACTIVE_LOW with pull-up recommended) |
| `rst-gpios` | phandle | GPIO pin for hardware reset (ACTIVE_LOW) |
| `ti,interface-type` | string | Which type of MAC interface the PHY is setup for |
  
## 3. Build Environment Setup on Ubuntu Host Machine

### 3.1 Install Dependencies
  
Ensure that the Ubuntu version is at least 20.04
  
Install dependencies using:
```bash
$ sudo apt install --no-install-recommends git cmake ninja-build gperf ccache
dfu-util device-tree-compiler wget python3-dev python3-pip python3-setuptools
python3-tk python3-wheel xz-utils file make gcc gcc-multilib g++-multilib
libsdl2-dev libmagic1
```
  
**### 3.2 Install Python Dependencies and Get Zephyr Source**
  
Run the following commands to install Python venv package and create/activate the virtual environment for Zephyr
  
```bash
$ sudo apt install python3-venv
$ python3 -m venv ~/zephyrproject/.venv
$ source ~/zephyrproject/.venv/bin/activate
```
Note: Remember to activate the virtual environment every time you start working.
  
Install west and get Zephyr source code:
```bash
$ pip install west
$ west init ~/zephyrproject
$ cd ~/zephyrproject
$ west update // Gets Zephyr Source Code
$ west zephyr-export // Exports Zephyr CMake Package
$ pip install -r ~/zephyrproject/zephyr/scripts/requirements.txt //  Installs the dependencies using pip
```
  
### 3.3 Install the Zephyr SDK
  
For this driver, Zephyr SDK v0.17.2 and v0.17.4 was used.
  
Follow the below script to download and install the SDK:
```bash
$ cd ~/zephyrproject
$ wget github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.17.2/zephyr-sdk-0.17.2_linux-x86_64.tar.xz
$ wget -O - https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.17.2/sha256.sum | shasum --check --ignore-missing
$ tar xvf zephyr-sdk-0.17.2_linux-x86_64.tar.xz
$ cd zephyr-sdk-0.17.2/
$ ./setup.sh
```
  
Install udev rules:
```bash
$ sudo cp ~/zephyr-sdk-0.17.2/sysroots/x86_64-pokysdk-linux/usr/share/openocd/contrib/60-openocd.rules /etc/udev/rules.d
$ sudo udevadm control --reload
```
  
# 4. Install DP83TD510E Driver Files

Place the driver files in the directories that is reflected by the Repository structure within zephyr/.
  
# 5. Test Hardware Platform

## 5.1 Host Board

### Hardware Requirements:
- Applicable Host board with Zephyr support
- Adapter for connecting 510E-EVM MAC connections to Host board
- DP83TD510E-EVM
- USB A Type to Micro USB B Type cable
  
### Hardware Setup
1. Connect DP83TD510E-EVM to an applicable Host board over RMII. Adapter is recommended to break-out SAMTEC <-> RMII connections
2. Power EVM over micro-USB, or by connecting power rails from Host board to external power jumpers
3. Power up the board by connecting the Micro USB B Type cable between the Debug USB port of the target platform and your host machine.
  
### Software Setup
1. Modify applicable dts file under  boards/../host_board.dts using host_board.dts reference under board_dts/ in this repo.
  
This dts file enables DP83TD510E for RMII leader mode. PHY expects 25M input on XI, with output 50M ref clock routed to applicable Host input ref clock.
  
### Build the Application
1. Compile zperf application
```bash
west build -b <host_board> samples/net/zperf/
```
2. Use menu configuration GUI to enable legacy UDP packets
```bash
$ west build -t menuconfig
```
Legacy UDP Packet is enabled by:
(Top) → Subsystems and OS Services → Networking → Link layer and networking (including IP) →
Network additional services → zperf library → Legacy iperf UDP header format
  
3. Save and close the configuration.
  
4. Rebuild the application.

# 6. Flash the Target Platform

1. Flash the target platform.
```bash
$ west flash
```
2. Open serial port console in separate terminal window.
```bash
$ sudo minicom -D /dev/ttyACM0
```
  
3. You can check the IP address in prj.conf file:
samples/net/zperf/prj.conf
  
# 7. Test the network performance

## 7.1 Setup

For internal testing during driver development, Host was connected to DP83TD510E-EVM using SAMTEC adapter to break-out RMII, SMI, and clock connections. This was then connected to another 510EVM media converter to complete T1L - TX link to another Linux PC.
  
## 7.2 Ping and zperf test

Ping can be done by:
```bash
$ net ping <destination ip address>
```
  
Zperf can be done by:
From node 0:
```bash
$ zperf udp download 5001
```
From node 1:
```bash
$ zperf udp upload <ip_address> 5001 <duration> <packet_size> <bandwidth>
```

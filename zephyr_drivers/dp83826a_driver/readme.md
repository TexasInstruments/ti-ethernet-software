# DP83826/A 100BASE-TX PHY Ethernet Zephyr Driver

This document describes how to setup and test DP83826x driver using Zephyr OS. 
The driver was validated on Zephyr repository v4.3 and SDK v1.0.
## 1. Repository Structure
```bash
driver_files/
├─ board_dts/
│   └─ host_board.dts
│
├─ drivers/
│   └─ ethernet/phy/
|		├─ Kconfig
|		├─ phy_ti_dp83826x.c 
│       └─ CMakeLists.txt
│
├─ dts/
│   └─ bindings/
│       └─ ethernet/phy/
│           └─ ti,dp83826x.yaml
└─ 
```
### 1.1 Key File Notes

Device Tree (*.dts): Defines tree integrated with Zephyr Driver enabled
Driver Source: Enables PHY with the MCU
DTS Bindings: Defines required properties for the driver in the DTS

## 2. Features and Device Tree

### 2.1 Supported Features

This driver supports the following features:

- **10/100BASE-T1X Standard Ethernet**: Full support for IEEE 802.3 10/100Base-TX communication
- **SMI Access**: MDIO/MDC access for register configuration
- **MII, RMII Leader, RMII Follower**: DTS configuration to enable MII, RMII Leader, or RMII follower modes
- **DAC Voltage Swing Tuning**: DTS configuration to tune DAC voltage swing.

**Device Tree Properties:**

| Property | Type | Description |
|----------|------|-------------|
| `compatible` | string | Device identification - must be "ti,dp83826x" |
| `int-gpios` | phandle | GPIO for interrupt signal indicating PHY state change |
| `reset-gpios` | phandle | GPIO pin for hardware reset (ACTIVE_LOW) |
| `ti,interface-type` | string | Which type of MAC interface the PHY is setup for |
| `ti,cfg-dac-minus-one-bp` | int | Sets the voltage ratio of logical level -1 for the MLT-3 encoded TX data |
| `ti,cfg-dac-plus-one-bp` | int | Sets the voltage ratio of logical level +1 for the MLT-3 encoded TX data |


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

## 3.2 Install Python Dependencies and Get Zephyr Source

Run the following commands to install Python venv package and create/activate the virtual environment for Zephyr

```bash
$ sudo apt install python3-venv
$ python3 -m venv ~/zephyrproject/.venv
$ source ~/zephyrproject/.venv/bin/activate
```
Note: Remember to activate the virtual environment every time you start working.

Install west and get Zephyr source code:
```bash
$ pip install west
$ west init ~/zephyrproject
$ cd ~/zephyrproject
$ west update // Gets Zephyr Source Code
$ west zephyr-export // Exports Zephyr CMake Package
$ pip install -r ~/zephyrproject/zephyr/scripts/requirements.txt //  Installs the dependencies using pip
```

## 3.3 Install the Zephyr SDK

For this driver, Zephyr SDK v1.0.0 and Zephyr v4.3 was used. 

Follow the below script to download and install the SDK:
```bash
$ cd ~
$ wget github.com/zephyrproject-rtos/sdk-ng/releases/download/v1.0.0/zephyr-sdk-1.0.0_linux-x86_64.tar.xz
$ wget -O - https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v1.0.0/sha256.sum | shasum --check --ignore-missing
$ tar xvf zephyr-sdk-1.0.0_linux-x86_64.tar.xz
$ cd zephyr-sdk-1.0.0/
$ ./setup.sh 
```

Install udev rules:
```bash
$ sudo cp ~/zephyr-sdk-1.0.0/sysroots/x86_64-pokysdk-linux/usr/share/openocd/contrib/60-openocd.rules /etc/udev/rules.d
$ sudo udevadm control --reload
```

# 4. Install DP83826X Driver Files
Place the driver files as described below:

Following files go into zephyr/drivers/ethernet/phy:
- Kconfig (Please overwrite the original Kconfig)
- phy_ti_dp83826x.c 
- CMakeLists.txt (Please overwrite the original Kconfig)

Following file goes into zephyr/dts/bindings/ethernet/phy:
- ti,dp83826x.yaml

# 5. Test Hardware Platform

## 5.1 Host Board
### Hardware Requirements:
- Applicable Host board with Zephyr support
- Adapter for connecting 826x-EVM MAC connections to Host board
- DP83826x-EVM or equivalent
- USB A Type to Micro USB B Type cable

### Hardware Setup
1. Connect DP883826x-EVM to an applicable Host board over MAC. Adapter may be required to break-out MAC connections between PHY EVM and Host.
2. Power EVM over micro-USB, or by connecting power rails from Host board to external power jumpers
3. Power up the board by connecting the Micro USB B Type cable between the Debug USB port of the target platform and your host machine.

### Software Setup
Modify applicable dts file under  boards/../host_board.dts using host_board.dts reference under board_dts/ in this repo.
This dts file enables DP83826x for RMII follower mode. PHY expects 50M on XI, provided by Host reference clock.

# 6. Test the network performance
## 6.1 Setup
Applicable Host board, such as BeaglePlay or TMDS64EVM with Networking support.
## 6.2 Ping and zperf test
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


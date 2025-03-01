## Point One Polaris Client for MAVLink
This application uses the [polaris](https://github.com/PointOneNav/polaris) library to interface with Point One's GNSS corrections service and send RTCM corrections via MAVLink.

You must first create a PointOne account and copy your API key to the config file. <br>
https://app.pointonenav.com/

The **config.toml** file is used to configure the program settings. <br>
https://github.com/ARK-Electronics/mavlink-to-lattice/blob/cfc8b5ad4350fdedb18006e8ecd1872272c747a0/config.toml#L1-L2

### Behavior
The application waits for a MAVSDK connection and once connected the Polaris client is created and runs asynchronously. The received [GPS_RAW_INT](https://mavlink.io/en/messages/common.html#GPS_RAW_INT) mavlink messages from the flight controller are converted from WSG to ECEF and sent to the Polaris server in order to begin receiving corrections. The corrections are binary RTCM3 and are published as [GPS_RTCM_DATA](https://mavlink.io/en/messages/common.html#GPS_RTCM_DATA) MAVLink messages.

### Build
Pre-requisites
```
sudo apt install libssl-dev libgflags-dev libgoogle-glog-dev libboost-all-dev
```
Install MAVSDK if you haven't already, the latest releases can be found at https://github.com/mavlink/MAVSDK/releases
```
sudo dpkg -i libmavsdk-dev_2.4.1_debian12_arm64.deb
```
Build
```
make
```
Run
```
./build/mavlink-to-lattice
```

### Issues
Some areas service is not available? I couldn't receive corrections for a Swiss or Alaskan location.
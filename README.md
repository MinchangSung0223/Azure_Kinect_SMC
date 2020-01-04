## Install Azure Kinect SDK for Ubuntu 18.04 (Bionic)
```bash
 export AZURE_HOME=$PWD
 sudo apt-get install curl
 sudo apt-get install libx11-dev
 sudo apt-get install xorg-dev libglu1-mesa-dev
 curl https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
 sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
 sudo apt-get update
 sudo apt install k4a-tools
 sudo apt install libk4a1.3-dev
 git clone https://github.com/ninja-build/ninja.git
 cd ninja
 ./configure
 ./bootstrap
 apt-get install ninja-build
 git clone https://github.com/microsoft/Azure-Kinect-Sensor-SDK.git
 rm -r Azure-Kinect-Sensor-SDK/examples
 cd $AZURE_HOME
 mv examples Azure-Kinect-Sensor-SDK/examples
 mv smc_examples Azure-Kinect-Sensor-SDK/smc_examples
 cd Azure-Kinect-Sensor-SDK
 cp scripts/99-k4a.rules /etc/udev/rules.d/
 mkdir build && cd build
 sudo rm /usr/lib/x86_64-linux-gnu/libGL.so
 sudo ln -s /usr/lib/libGL.so.1 /usr/lib/x86_64-linux-gnu/libGL.so
 sudo apt install libsoundio-dev
 sudo apt-get install uuid-dev
 sudo apt-get install libudev-dev
 sudo apt-get install libusb-1.0-0-dev
 cmake .. -G Ninja
 ninja
 ninja install
 cd bin
 cp libk4a* /usr/lib/
 k4aviewer
```
## Test Python code
```bash
 cd $AZURE_HOME
 cd Azure-Kinect-Sensor-SDK/smc_examples
 python3 capture.py
```


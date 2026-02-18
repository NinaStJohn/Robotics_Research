
# Robotics LTL-D* from simulation to implemtation #

This repository is deteicated to exploring LTL's applicaiton in robotics. 


### References ###

the spot github that was used, in Ubuntu
```
// required build dependencies
sudo apt update
sudo apt install -y \
  build-essential git pkg-config \
  autoconf automake libtool \
  flex bison \
  libbdd-dev libgmp-dev libboost-all-dev \
  python3-dev swig

// clone with submodules
cd ~
git clone --recurse-submodules https://gitlab.lre.epita.fr/spot/spot.git
cd spot

// bootstrap
autoreconf -vfi

// configure
./configure --prefix=/usr/local

// build
make -j$(nproc)

// install
sudo make install
sudo ldconfig

```
this is a warning for furture me reference
```
===================================================================
 This is a development version of Spot: Assertions and debuging
 code are enabled by default.  If you find this too slow or
 plan to do some benchmarking, run configure with --disable-devel.
===================================================================
```

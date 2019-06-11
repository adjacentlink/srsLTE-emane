srsLTE-emane
========

srsLTE-emane is a derivative project of [srsLTE](https://github.com/srsLTE). In conjunction with the
[EMANE LTE Model](https://github.com/adjacentlink/emane-model-lte.git), srsLTE-emane enables emulating
an LTE network in [EMANE](https://github.com/adjacentlink/emane.git) on a laptop, desktop or other
dedicated computer resources. Software Defined Radio (SDR) hardware is not required.

The srsLTE applications, `srsenb`, `srsue`, `srsepc` and `srsmbms`, are adapted to send RF traffic through
the EMANE emulation environment. The EMANE version of the applications are renamed `srsenb-emane`, `srsue-emane`,
`srsepc-emane` and `srsepc-emane`. `srsenb-emane` and `srsue-emane` contain an embedded EMANE instance
to replace lower levels of the LTE radio stack required to emulate the over the air communication effects. The new applications are also instrumented for data extraction using
[OpenStatistic](https://github.com/adjacentlink/openstatistic). Internal statistics can be queried manually
via the `ostatistic` application, or collected automatically using
[OpenTestPoint](https://github.com/adjacentlink/opentestpoint) and the
[OpenTestPoint LTE Probe](https://github.com/adjacentlink/opentestpoint-probe-lte).

srsLTE-emane is released under the AGPLv3 license. The current stable
version is 18.12.2, the second release based on srsLTE 18.12.

---
## Build Instructions

1. Install the latest [pre-built EMANE bundle](https://github.com/adjacentlink/emane/wiki/Install). EMANE version 1.2.3 or later is **required**.

2. Build and install the [EMANE LTE Model](https://github.com/adjacentlink/emane-model-lte.git).

3. Build and install srsLTE-emane:
   * [Centos 7](#centos-7)
   * [Fedora 28 and 29](#fedora-28-and-29)
   * [Ubuntu 16.04 and 18.04](#ubuntu-1604-and-1804)


### Centos 7

```
sudo yum install cmake fftw3-devel polarssl-devel lksctp-tools-devel libconfig-devel boost-devel redhat-lsb-core
git clone https://github.com/adjacentlink/srsLTE-emane.git
cd srsLTE-emane
mkdir build
cd build
cmake ..
make package
sudo yum install srslte-emane-18.12.1-x86_64.rpm
```

### Fedora 28 and 29

```
sudo dnf install cmake fftw3-devel polarssl-devel lksctp-tools-devel libconfig-devel boost-devel redhat-lsb-core
git clone https://github.com/adjacentlink/srsLTE-emane.git
cd srsLTE-emane
mkdir build
cd build
cmake ..
make package
sudo dnf install srslte-emane-18.12.1-x86_64.rpm
```

### Ubuntu 16.04 and 18.04

```
sudo apt-get install cmake libfftw3-dev libmbedtls-dev libboost-program-options-dev libconfig++-dev libsctp-dev lsb-release
git clone https://github.com/adjacentlink/srsLTE-emane.git
cd srsLTE-emane
mkdir build
cd build
cmake ..
make package
sudo dpkg -i srslte-emane-18.12.1-x86_64.deb; sudo apt-get install -f
```

---
## Demonstration

[EMANE LTE Model](https://github.com/adjacentlink/emane-model-lte.git) contains a demonstration for running
a small network with two UEs and one ENB.

---
## Configuration

Each of the emane version srsLTE applications take an input
configuration file, identical to the one used by the regular srsLTE
applications, with the following additional parameters.


```
[runtime]
daemonize = 1     # 0 - foreground, 1 - run as daemon

[mhal]
#statistic_service_endpoint=0.0.0.0:47100
#emane_configfile=emanelte.xml
```

The new `runtime` configuration section contains a `daemonize` parameter
that controls whether the application runs in the foreground or as a daemon.

The new `mhal` section contains a `statistic_service_endpoint`
parameter to set the OpenStatistic query address and port (all
applications), and the `emane_configfile` parameter takes the
name of the the configuration file fed to the embedded EMANE instance
(`srsenb-emane` and `srsue-emane` only). Default values shown.

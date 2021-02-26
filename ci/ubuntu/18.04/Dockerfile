FROM ubuntu:18.04

RUN mkdir -p /opt/built

# prevent failures due to interactive apt transactions
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get -y update
RUN apt-get -y upgrade

# Copy and install packages from the latest opentestpoint build
WORKDIR /opt
RUN mkdir -p dependencies
COPY --from=gcr.io/automated-builds-303920/ubuntu18.04.emane /opt/built/* dependencies/
COPY --from=gcr.io/automated-builds-303920/ubuntu18.04.opentestpoint /opt/built/* dependencies/
COPY --from=gcr.io/automated-builds-303920/ubuntu18.04.openstatistic /opt/built/* dependencies/
COPY --from=gcr.io/automated-builds-303920/ubuntu18.04.emane-model-lte /opt/built/* dependencies/
RUN dpkg -i dependencies/*; apt-get -y -f install

# Build tools and dependencies
RUN apt-get -y install git gcc g++ debhelper dh-python pkg-config python-setuptools \
                       protobuf-compiler libprotobuf-dev python-protobuf
RUN apt-get -y install libxml2-dev libpcap-dev libpcre3-dev uuid-dev
RUN apt-get -y install cmake libfftw3-dev libmbedtls-dev libboost-program-options-dev libconfig++-dev libsctp-dev lsb-release

RUN git clone https://github.com/adjacentlink/srsLTE-emane -b develop

WORKDIR srsLTE-emane
RUN mkdir build
WORKDIR build
RUN cmake ..
RUN make package
RUN cp *\.deb /opt/built
RUN dpkg -i /opt/built/*; apt-get -y -f install

RUN echo 'complete'


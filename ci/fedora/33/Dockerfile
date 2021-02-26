FROM fedora:33

RUN mkdir -p /opt/built

RUN dnf -y update

# Copy and install packages from the latest opentestpoint build
WORKDIR /opt
RUN mkdir -p dependencies
COPY --from=gcr.io/automated-builds-303920/fedora33.emane /opt/built/* dependencies/
COPY --from=gcr.io/automated-builds-303920/fedora33.opentestpoint /opt/built/* dependencies/
COPY --from=gcr.io/automated-builds-303920/fedora33.openstatistic /opt/built/* dependencies/
COPY --from=gcr.io/automated-builds-303920/fedora33.emane-model-lte /opt/built/* dependencies/
RUN dnf -y install dependencies/*\.rpm

# Build tools and depends
RUN dnf -y install git gcc-c++ make rpm-build
RUN dnf -y install libxml2-devel libpcap-devel pcre-devel libuuid-devel
RUN dnf -y install cmake fftw3-devel polarssl-devel lksctp-tools-devel libconfig-devel boost-devel redhat-lsb-core

RUN git clone https://github.com/adjacentlink/srsLTE-emane -b develop

WORKDIR srsLTE-emane
RUN mkdir build
WORKDIR build
RUN cmake ..
RUN make package
RUN cp "*\.rpm" /opt/built
RUN dnf -y install /opt/built/*

RUN dnf clean all

RUN echo 'complete'


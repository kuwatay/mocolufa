FROM ubuntu:14.04

LABEL Description="Docker for avr-gcc-embedded projects"

RUN dpkg-divert --local --rename --add /sbin/initctl
RUN ln -sf /bin/true /sbin/initctl
ENV DEBIAN_FRONTEND noninteractive

RUN dpkg --add-architecture i386 && \
    apt-get update && \
    apt-get -y install gcc-avr binutils-avr avr-libc gdb-avr avrdude  make git  sudo curl tree nano lrzsz

# clean cache
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

VOLUME ["/opt/avr"]
CMD ["/bin/bash"]

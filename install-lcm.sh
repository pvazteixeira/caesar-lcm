#!/bin/sh
set -ex
wget https://github.com/lcm-proj/lcm/archive/v1.3.1.tar.gz -O lcm.tar.gz
tar -xvf lcm.tar.gz
cd lcm-1.3.1 && ./bootstrap.sh && ./configure && make && sudo make install

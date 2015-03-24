# grl
Generic Reinforcement Learning Library

Copyright 2015 Wouter Caarls

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

# Introduction

GRL is a C++ reinforcement learning library that aims to easily allow
evaluating different algorithms through a declarative configuration
interface.

![Configurator](/doc/grl.png)

# Installation (Ubuntu 14.04)
```
sudo apt-get install git cmake g++ libboost-dev
git clone https://github.com/wcaarls/grl.git
```

For the locally linear approximation, additionally install
```
sudo apt-get install libeigen3-dev
```

For the visualization, additionally install
```
sudo apt-get install libgl1-mesa-dev-lts-utopic freeglut3-dev
```

For the odesim environment, additionally install
```
sudo apt-get install libqt4-opengl-dev libtinyxml-dev libmuparser-dev libode-dev
```

For the configurator, additionally install
```
sudo apt-get install python-yaml python-tk
```

# Setup
```
mkdir build
cd build
cmake ..
make
```

# Running

To directly perform an experiment, run

```
./grld ../cfg/pendulum/sarsa_tc.yaml
```

To start the configurator instead, run

```
cd ../bin
./grlc ../cfg/pendulum/sarsa_tc.yaml
```

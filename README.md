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

# Installation (Ubuntu 16.04)
```
sudo apt-get install git cmake g++ libeigen3-dev
git clone https://github.com/wcaarls/grl.git
```

For the visualization, additionally install
```
sudo apt-get install libgl1-mesa-dev freeglut3-dev libz-dev
```

For the odesim environment, additionally install
```
sudo apt-get install libqt4-opengl-dev libtinyxml-dev libmuparser-dev libode-dev
```

For the RBDL environment and MUSCOD integration, additionally install
```
sudo apt-get install liblua5.1-dev
```

For the configurator, additionally install
```
sudo apt-get install python-yaml python-tk
```

For the zeromq addon, additionally install
```
sudo apt-get install libprotoc-dev protobuf-compiler libsodium-dev
```
Then download a recent (> 4.0.0) version of zeromq and run './configure, make, make install'

For the tensorflow addon, additionall install

```
sudo apt-get install libprotoc-dev protobuf-compiler
```
Then download TensorFlow for C from https://www.tensorflow.org/install/install_c
in a place that can be found by the compiler and runtime environment.

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

# Visualizations

GRL comes with standard visualizations for value functions,
policies, and the integrated environments (e.g. pendulum
swing-up, cart-pole swing-up, compass walker)

![Visualizations](/doc/grl2.png)

# Further reading

See [grl.pdf](/doc/grl.pdf)

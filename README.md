# grl
[![Build # Status](https://github.com/wcaarls/grl/workflows/CMake/badge.svg)](https://github.com/wcaarls/grl/actions?query=workflow%3ACMake)

Generic Reinforcement Learning Library

Copyright 2015-2022 Wouter Caarls

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

# Installation

## Ubuntu 22.04
```
sudo apt install git cmake g++ libeigen3-dev libpython3-dev python3-distutils libz-dev
git clone https://github.com/wcaarls/grl.git
```

### For the visualization, additionally install
```
sudo apt-get install libgl1-mesa-dev freeglut3-dev
```

### For the configurator, additionally install
```
sudo apt-get install python3-yaml python3-tk
```

### For the tensorflow addon, additionall install
```
sudo apt-get install libprotobuf-dev protobuf-compiler

# Tensorflow C API 2.8.0
wget https://storage.googleapis.com/tensorflow/libtensorflow/libtensorflow-gpu-linux-x86_64-2.8.0.tar.gz
sudo tar zxvf libtensorflow-gpu-linux-x86_64-2.8.0.tar.gz -C /usr/local

# CUDA Toolkit 11.7
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/3bf863cc.pub
sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/ /"
sudo apt-get update
sudo apt-get install cuda

### cuDNN 8.2.1
wget https://anaconda.org/anaconda/cudnn/8.2.1/download/linux-64/cudnn-8.2.1-cuda11.3_0.tar.bz2
sudo tar -jxvf cudnn-8.2.1-cuda11.3_0.tar.bz2 -C /usr/local --wildcards "lib/*"
```

Then edit `~/.bashrc` to include the following, and open a new terminal
```
export PATH=$PATH:/usr/local/cuda-11/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib:/usr/local/cuda-11/lib64
```

Install Tensorflow 2 for Python

```
sudo -H python3 -m pip install tensorflow
```

If there are errors relating to Python loading
`libtensorflow_framework.so.2`, it is because it comes with its own version,
and it finds the version from the C API. Workaround:
```
sudo apt-get install patchelf
sudo patchelf --replace-needed libtensorflow_framework.so.2 libtensorflow_framework.so.2.8.0 /usr/local/lib/libtensorflow.so.2.8.0
sudo rm /usr/local/lib/libtensorflow_framework.so /usr/local/lib/libtensorflow_framework.so.2
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

# Visualizations

GRL comes with standard visualizations for value functions,
policies, and the integrated environments (e.g. pendulum
swing-up, cart-pole swing-up, compass walker)

![Visualizations](/doc/grl2.png)

# Further reading

See [grl.pdf](/doc/grl.pdf)

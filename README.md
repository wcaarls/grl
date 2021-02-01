# grl
[![Build # Status](https://github.com/wcaarls/grl/workflows/CMake/badge.svg)](https://github.com/wcaarls/grl/actions?query=workflow%3ACMake)

Generic Reinforcement Learning Library

Copyright 2015-2020 Wouter Caarls

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

## Ubuntu 20.04
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

#### Tensorflow 2

```
# Tensorflow C API 2.4.0
wget https://storage.googleapis.com/tensorflow/libtensorflow/libtensorflow-gpu-linux-x86_64-2.4.0.tar.gz
sudo tar zxvf libtensorflow-gpu-linux-x86_64-2.4.0.tar.gz -C /usr/local

# CUDA Toolkit 11.0
sudo apt-key adv --fetch-keys http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub
sudo bash -c 'echo "deb http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/cuda.list'
sudo bash -c 'echo "deb http://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/cuda_learn.list'
sudo apt update
sudo apt install cuda-11-0
sudo apt install libcudnn8
```

Then edit `~/.bashrc` to include the following, and open a new terminal
```
export PATH=$PATH:/usr/local/cuda-11.0/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-11.0/lib64
```

Install Tensorflow 2 for Python

```
sudo -H python3 -m pip install tensorflow
```

#### Tensorflow 1

Tensorflow 1 is deprecated. Consider swithing to Tensorflow 2 (see above).

```
# Tensorflow C API 1.14.0
wget https://storage.googleapis.com/tensorflow/libtensorflow/libtensorflow-gpu-linux-x86_64-1.14.0.tar.gz
sudo tar zxvf libtensorflow-gpu-linux-x86_64-1.14.0.tar.gz -C /usr/local

# CUDA Toolkit 10.0
wget https://developer.nvidia.com/compute/cuda/10.0/Prod/local_installers/cuda_10.0.130_410.48_linux
wget http://developer.download.nvidia.com/compute/cuda/10.0/Prod/patches/1/cuda_10.0.130.1_linux.run
chmod a+x *.run
sudo ./cuda_10.0.130_410.48_linux --override # When asked, skip installing graphics driver
sudo ./cuda_10.0.130.1_linux.run

# cuDNN 7.6.5 for CUDA 10.0
# First log in with NVIDIA developer account
# Download https://developer.nvidia.com/compute/machine-learning/cudnn/secure/7.6.5.32/Production/10.0_20191031/cudnn-10.0-linux-x64-v7.6.5.32.tgz
sudo tar zxvf cudnn-10.0-linux-x64-v7.6.5.32.tgz -C /usr/local
```

Then edit `~/.bashrc` to include the following, and open a new terminal
```
export PATH=$PATH:/usr/local/cuda-10.0/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-10.0/lib64
```

Install Python 3.7 from ppa or from source, e.g.
```
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt-get update
sudo apt-get install python3.7 python3.7-dev
```

Install Tensorflow for Python 1.14.0

```
sudo -H python3.7 -m pip install tensorflow==1.14.0 keras==2.2.4
```

Finally, edit the network files (`cfg/4layer_*.py`) to use the correct Python version if
necessary.

If there are errors relating to Python loading
`libtensorflow_framework.so.1`, it is because it comes with its own version,
and it finds the version from the C API. Consider adding the path of the
Python version to `LD_LIBRARY_PATH`, before the C version.


## Ubuntu 16.04
```
sudo apt install git cmake g++ libeigen3-dev libpython3-dev python3-distutils libz-dev
git clone https://github.com/wcaarls/grl.git
```

### For the visualization, additionally install
```
sudo apt-get install libgl1-mesa-dev freeglut3-dev
```

### For the odesim environment, additionally install
```
sudo apt-get install libqt4-opengl-dev libtinyxml-dev libmuparser-dev libode-dev
```

### For the RBDL environment and MUSCOD integration, additionally install
```
sudo apt-get install liblua5.1-dev
```

### For the configurator, additionally install
```
sudo apt-get install python-yaml python-tk
```

### For the zeromq addon, additionally install
```
sudo apt-get install libprotoc-dev protobuf-compiler libsodium-dev
```
Then download a recent (> 4.0.0) version of zeromq and run './configure, make, make install'

### For the tensorflow addon, additionally install

```
sudo apt-get install libprotoc-dev protobuf-compiler
```
Then download TensorFlow for C 1.14.0 from https://www.tensorflow.org/install/install_c
in a place that can be found by the compiler and runtime environment. To use
Python to build the networks, the same Tensorflow version (and Keras 2.2.4)
should be available under Python as well.

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

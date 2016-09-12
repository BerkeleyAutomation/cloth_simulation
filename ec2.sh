#!/bin/bash

sudo apt-get --assume-yes update
sudo apt-get --assume-yes upgrade

sudo apt-get --assume-yes install python-matplotlib python-numpy python-scipy cython ipython git python-qt4

git clone https://github.com/BerkeleyAutomation/cloth_simulation.git


sudo apt-get --assume-yes build-dep python-scipy python-pygame
sudo apt-get --assume-yes install swig
wget https://repo.continuum.io/archive/Anaconda2-4.1.1-Linux-x86_64.sh
bash Anaconda2-4.1.1-Linux-x86_64.sh
source ~/.bashrc

git clone https://github.com/rllab/rllab.git
cd rllab
bash scripts/setup_linux.sh

cd
source activate rllab
pip install cython
export PYTHONPATH=/home/ubuntu/rllab:$PYTHONPATH

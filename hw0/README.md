# pdm-f23-hw0

NYCU Perception and Decision Making 2023 Fall

Spec: https://docs.google.com/document/d/1DHr3-iiZNR1UNKYoBC-aF7qijACRdsjz/edit

## Introduction 
In this course, we are going to build an indoor navigation system in Habitat step by step during homework 1 ~ 3 and the final project. This homework 0 will help you to build the environment with essential packages.

## Requirements
- OS : Ubuntu Desktop 18.04, 20.04

    - Ubuntu on virtual machine is not recommended
    - MacOS may work but not guaranteed
- Python 3.7 ( You can use conda to create new environment )

## Installation
```bash
# Ensure the latest submodules
git submodule update --init --recursive
# Create a conda env
conda create -n habitat python=3.7
# Activate the conda env
conda activate habitat
# Install habitat-sim from source
conda install cmake=3.14.0
cd habitat-sim && pip install -r requirements.txt && python setup.py install --bullet --headless && cd ..
# Install habitat-lab
cd habitat-lab && pip install -r requirements.txt && python setup.py develop && cd ..
# Install requirements
pip install -r requirements.txt
```


## Tasks

1. Please download the replica dataset from this link: [Apartment_0](https://drive.google.com/file/d/1zHA2AYRtJOmlRaHNuXOvC_OaVxHe56M4/view)

2. Run the load.py to explore the scene and **record the video on screen (<30 sec)** at the same time to demonstrate that the simulation can be opened up successfully.

    (Note: You can change agent_state.position to set the agent in the first or second floor. (0, 0, 0) for first floor and (0, 1, 0) for second floor.)
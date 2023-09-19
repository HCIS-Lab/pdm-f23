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

### Clone the repo
`git clone git@github.com:HCIS-Lab/pdm-f23.git` to download the repo or create a new fork on you own GitHub account.

```bash
cd pdm-f23/hw0
# Ensure the latest submodules
git submodule update --init --recursive
# Create a conda env
conda create -n habitat python=3.7
# Activate the conda env
conda activate habitat
# Install requirements
pip install -r requirements.txt
# Install habitat-sim from source
cd habitat-sim && pip install -r requirements.txt && python setup.py install --bullet && cd ..
# Install habitat-lab
cd habitat-lab && pip install -r requirements.txt && python setup.py develop && cd ..
```

### Download dataset

Download dataset from [here](https://drive.google.com/file/d/1zHA2AYRtJOmlRaHNuXOvC_OaVxHe56M4/view)
and put the directory under `replica_v1/`

## Tasks

1. Run the `load.py` to explore the scene

    Use keyboard to control the agent
    ```
    w for go forward  
    a for turn left  
    d for trun right  
    f for finish and quit the program
    ```

    **Note**

    You can change agent_state.position to set the agent in the first or second floor. (0, 0, 0) for first floor and (0, 1, 0) for second floor.


2. **Record the video on screen (<30 sec)** at the same time to demonstrate that the simulation can be opened up successfully.

    

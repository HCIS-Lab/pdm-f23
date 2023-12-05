# pdm-f23
NYCU Perception and Decision Making 2023 Fall

# HW4 Robot Manipulation

## Installation

 Create and activate Conda environment, then install GCC and Python packages.

```shell
cd ravens
conda create --name pdm-hw4 python=3.7 -y
conda activate pdm-hw4
sudo apt-get update
sudo apt-get -y install gcc libgl1-mesa-dev
pip install -r requirements.txt
```

## Task 1

Implement your_fk function in fk.py

## Task 2

Implement your_ik function in ik.py

## Task 3

Test your ik implementation in the Transporter Networks 's frame work by inferencing "Block Insertion Task" on a mini set of 10 testing data

**Step 1.** Download dataset   at and put it under

**Step 2.** Download checkpoint at and put it under 

**Step 3.** Testing the model and your ik implementaton 

 ```shell
export CUDA_VISIBLE_DEVICES="" # No need to use gpu
python ravens/test.py --assets_root=./ravens/environments/assets/ --disp=True --task=block-insertion --agent=transporter --n_demos=1000 --n_steps=20000
 ```
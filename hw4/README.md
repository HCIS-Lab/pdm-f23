# pdm-f23-hw4
NYCU Perception and Decision Making 2023 Fall

# HW4 Robot Manipulation

## Installation

Follow the installation instruction in https://github.com/google-research/ravens

1. Create and activate Conda environment, then install GCC and Python packages.

```shell
cd ravens
conda create --name pdm-hw4 python=3.7 -y
conda activate pdm-hw4
sudo apt-get update
sudo apt-get -y install gcc libgl1-mesa-dev
pip install -r requirements.txt
```
2. Install GPU acceleration with NVIDIA CUDA 10.1 and cuDNN 7.6.5 for Tensorflow.
```bash
conda install cudatoolkit==10.1.243 -y
conda install cudnn==7.6.5 -y
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
# No need to use GPU
CUDA_VISIBLE_DEVICES=-1 python ravens/test.py --assets_root=./ravens/environments/assets/ --disp=True --task=block-insertion-easy --agent=transporter --n_demos=1000 --n_steps=20000
 ```

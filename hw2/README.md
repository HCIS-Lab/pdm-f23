# pdm-f23-hw2

NYCU Perception and Decision Making 2023 Fall

Spec: [Google Docs](https://drive.google.com/file/d/1LdzOZnM4sa_z1dcEKYHdXxHH_FsDKr_h/view?usp=sharing)

## Preparation
In your original dpm-f23 directory, `git pull` to get new `hw2` directory. 

If you want to use [semantic-segmentation-pytorch](https://github.com/CSAILVision/semantic-segmentation-pytorch), you may face the problem that the page of pretrain model are not accessible.
Please check the [issue](https://github.com/CSAILVision/semantic-segmentation-pytorch/issues/286) and follow the step here to download model checkpoints in the same way.

```
MODEL_NAME=ade20k-mobilenetv2dilated-c1_deepsup
ENCODER=$MODEL_NAME/encoder_epoch_20.pth
DECODER=$MODEL_NAME/decoder_epoch_20.pth

MODEL_NAME=ade20k-hrnetv2-c1
ENCODER=$MODEL_NAME/encoder_epoch_30.pth
DECODER=$MODEL_NAME/decoder_epoch_30.pth

MODEL_NAME=ade20k-resnet50dilated-ppm_deepsup
ENCODER=$MODEL_NAME/encoder_epoch_20.pth
DECODER=$MODEL_NAME/decoder_epoch_20.pth

```

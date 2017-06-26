# Simultaneous Hand-Eye Calibration and Reconstruction - shecar

We present a novel pipeline for hand-eye calibration which combines classical hand-eye calibration with 3D reconstruction. This work will be presented at IROS 2017.

The accompyniying dataset is located at: https://robotics.shanghaitech.edu.cn/datasets/shecar 


# Dependencies
1. [Theia]. This project highly depends on Theia. It is worthy mentioning that we are not using the mainstream of theia, to make it easy to reuse, we modify the original code a little, so please be sure you are using the code provided here. After you install Theia successfully, you should compile this project without any trouble.

# How to run
We have a sample dataset in 'data/', after you compile successfully, you can try this dataset by

`./bin/SHECAR --flagfile=hand_eye_calibration_flags.txt`


[Theia]: https://github.com/zhixy/TheiaSfM/tree/HandEye

Authors:

Code & paper: Zhi Xiangyang: https://robotics.shanghaitech.edu.cn/people/zhi 

Paper: Sören Schwertfeger: https://robotics.shanghaitech.edu.cn/people/soeren

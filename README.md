# 4d_association
Code for cvpr2020 "4D Association Graph for Realtime Multi-person Motion Capture Using Multiple Video Cameras"

### Introduction
Our work [4D Association Graph](http://www.liuyebin.com/4dassociation) contributes a novel realtime multi-personmotion capture algorithm using multiview video inputs. Due to the heavy occlusions and closely interacting motions in each view, joint optimization on the multiview images and multiple temporal frames is indispensable, which brings up the essential challenge of realtime efﬁciency. Our contributions include:
+ 4D Association Graph Representation: unify per-view parsing, cross-view matching, and temporal tracking into a single optimization framework that each dimension can be treated equally and simultaneously. Existing association methods consider only parts of these associations, or simply operate them in a sequential manner, which reduce the quality and efficiency of motion reconstruction. 
+ Robust Realtime Motion Capture System: propose a 4D limb bundle parsing solver based on heuristic searching to enable realtime reconstruction(30fps).
+ Multi-person 3D Skeleton Dataset: multi-view RGB video with aligned high-quality 3D skeletons from optitrack, mainly focusing on close interactions and challenging motion.

### Getting Started

Our project run on visual studio 2017(c++ 17), with requriements [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page), [jsoncpp](https://github.com/open-source-parsers/jsoncpp) and [opencv(4.2)](https://opencv.org/). 

You can download our pre-built bin file on

```
 https://pan.baidu.com/s/184RRDghnwiSCmEbxneyamQ  4enh
```

 Then Unzip the dependency file in `C:\cppmodule` or other place, and check the `.props` file to make sure compiler can find the correct include path and lib path. Moreover, you need to copy the dll file of dependency to exe directory or add them to system environment.  
 Finally, you will choose platform `release` and `x64` to compile and run our demo.

### Citation

```
Yuxiang Zhang, Liang An, Tao Yu, xiu Li, Kun Li, Yebin Liu. "4D Association Graph for Realtime Multi-person Motion Capture Using Multiple Video Cameras". arXiv 2020

@InProceedings{20204DAssociation,
  author = {Zhang, Yuxiang and An, Liang and Yu, Tao and Li, xiu and Li, Kun and Liu, Yebin},
  title = {4D Association Graph for Realtime Multi-person Motion Capture Using Multiple Video Cameras},
  booktitle = {IEEE International Conference on Computer Vision and Pattern Recognition, (CVPR)},
  year={2020},
}
```

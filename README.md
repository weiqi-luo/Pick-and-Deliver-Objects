# Pick-and-Deliver-Objects-to-the-Professor
## Project Introduction

TIAGo is a service robot designed to work in indoor environments.  

In this project, we work with TIAGo for a complete service task: when people is offering TIAGo a cup, he is able to react to it and take over the cup, then look around searching for Professor and deliver the cup.

<p align="center">
    <img src="pics/tiago.png", width="400">
    <br>
</p>



## Group Member and Task Split

| Group Member |                        Task                         |
| :----------: | :-------------------------------------------------: |
|   Ji Chen    | System Integration, Speech Recognition, Navigation  |
|  Weiqi Luo   | Gesture Detection, Face Detection, Object Detection |
|  Chao Dong   |                    Manipulation                     |

## An Overview of Perception Part

### 1. Gesture Detection based on OpenPose

[OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose) is the state-of-the-art pose estimation algorithm, which enables a real-time multi-person key-point detection for body, face, and hands estimation.

Gesture detection is aimed to detect the gesture of offering something, which triggers the start of the Pick-and-Deliver-Objects task.



<p align="center">
    <img src="pics/gesture.png", width="600">
    <br>
    <sup>An example of gesture detection.</sup>
</p>

### 2. Object Detection based on YOLO

[You only look once (YOLO)](https://pjreddie.com/darknet/yolo/)  is a state-of-the-art, real-time object detection system. On a Pascal Titan X it processes images at 30 FPS and has a mAP of 57.9% on COCO test-dev.

The bounding boxes obtained with YOLO helps to specify the position of the object, hence TIAGo is able to reach his hand to grasp the target object.

<p align="center">
    <img src="pics/object.png", width="350">
    <br>
    <sup>An example of face detection</sup>
</p>

### 3. Face Detection based on Python Library Face Recognition 

Face Detection is a Python Library which is built using [dlib](http://dlib.net/)'s state-of-the-art face recognition built with deep learning. The model has an accuracy of 99.38% on the [Labeled Faces in the Wild](http://vis-www.cs.umass.edu/lfw/) benchmark.

Face detection is used to distinguish professor from other unknown person so as to deliver the cup. 

<p align="center">
    <img src="pics/face.png", width="600">
    <br>
    <sup>An example of face detection</sup>
</p>


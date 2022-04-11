# Video Analysis for Obstacle Avoidance of Unmanned Surface Vehicles (Issa Mouawad)

## Object Detection:
The folder (darknet) contains the official code for YOLO written in C and CUDA and pulled from this repository (https://github.com/pjreddie/darknet). The code is modified to allow for batch processing (using a text file for the input images) and outputs the boat detections (to be saved in a file).
the suggested format of the input file can be of the form

grl1/00001.jpg

....

Additionally, the Make file is modified to allow for GPU inference.

The weights (of the trained network on MS COCO ) can be downloaded from here: 
wget https://pjreddie.com/media/files/yolov3.weights

The code is built using simply 

make

The following command can be used to run batch object detection

./darknet detect cfg/yolov3.cfg yolov3.weights file_list.txt -thresh .05 > output.txt

## Object Tracking
Object tracking is implemented in a Python notebook which uses several classes already defined in the (Video Tracking) folder. The code uses OpenCV mainly among other minor libraries.
The code uses as an input:
* The frames stored in a folder (the convention of naming is number of frame padded to 5 numbers)
* The detections file, which is the output of the object detection step.
The Tracking procedure then outputs 
* A video containing projecting the tracking results on the frames.
* A JSON file compatible with (http://cocodataset.org/#format-results).

## Additional Requirements
motmetrics, imutils

## 3D Localization
3D Localization using the 3D point cloud is implemented in Matlab. The procedure currently is not linked directly to the tracking, but rather uses hard-coded tracks as input.
The procedure (mainly accessible via function Get_Lidar_BB(BB_2D, InvM,pointCloud,order)) takes as inputs:
* A 4D vector representing the boudning box (MinX,MinY,MaxX,MaxY)
* The pseudo-inverse of the projection matrix
* The point cloud object.
* An optional paramter representing a prior on the depth of the obstacle (default is 1 for obstacles that are considered the closest to the sensor, this might can be passed differently if a prior exists, from the image-plane for example, that obstacles are occluding each others

The function returns two parameters:
* The 3D bounding box in the form [X1 Y1; X2 Y2; X3 Y3; X4 Y4]
* the centroid as a 3D point

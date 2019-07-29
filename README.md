# MSThesis_Code
Video Analysis for Obstacle Avoidance in Unmanned Surface Vehicles 
## Object Detection:
The folder (darknet) contains the official code for YOLO written in C and CUDA and pulled from this repository (https://github.com/pjreddie/darknet). The code is modified to allow for batch processing (using a text file for the input images) and outputs the boat detections (to be saved in a file).

Additionally, the Make file is modified to allow for GPU inference.

The weights (of the trained network on MS COCO ) can be downloaded from here
wget https://pjreddie.com/media/files/yolov3.weights

The code is built using simply 
make

The following command can be used to run batch object detection
./darknet detect cfg/yolov3.cfg yolov3.weights file_list.txt -thresh .05 > output.txt


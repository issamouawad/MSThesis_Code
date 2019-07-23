import numpy as np
import math
import scipy.interpolate as interp
from scipy.spatial import distance
import imutils
import time
from sklearn import preprocessing
import cv2 as cv
import time
from sklearn.utils.linear_assignment_ import linear_assignment
from utils import iou
from utils import get_distance
from utils import bounding_box_naive
from detection import Detection
from track import Track
from utils import get_points_in_bb

class Tracker(object):
    def __init__(self,method='keypoint_flow'):
        self.tracking_method = method
        self.tracks = []
        self.cur_id=1
        self.frameCount =1
        self.detect_interval=3
        self.track_len = 10
        self.feature_params=dict(maxCorners=200,qualityLevel=0.3,minDistance=7,blockSize=7)
        self.lk_params=dict(winSize=(15,15),maxLevel=2,criteria=(cv.TERM_CRITERIA_EPS|cv.TERM_CRITERIA_COUNT,10,0.03))
        self.flow_time=0
   
    def get_distance_matrix(self,dets,frame):
        dists = np.zeros((len(dets),len(self.tracks)),np.float32)
        for itrack in range(len(self.tracks)):
            for ipred in range(len(dets)):
                
                desc_dist = np.linalg.norm(dets[ipred].hog-self.tracks[itrack].hog,ord=1)
                
                iou_overlap = iou(dets[ipred].corners(),self.tracks[itrack].corners())
              
                uncertainety =np.maximum(1-dets[ipred].conf,0.5)
            
                dists[ipred,itrack] = uncertainety*((1-iou_overlap)+desc_dist)
                
                #dists[ipred,itrack] = ((1-iou_overlap)+desc_dist)
        return dists
    def track(self,dets,frame_gray,prev_frame_gray):
        
        dists = self.get_distance_matrix(dets,frame_gray)
        matched_indices = linear_assignment(dists)
        
        for m in matched_indices:
            
            
            iou_dist = iou(dets[m[0]].corners(),self.tracks[m[1]].corners())
            if((dists[m[0],m[1]]>0.7 and iou_dist<0.2 )or (iou_dist <=0)):#iou_threshold):
                m[0] = -1
                m[1]=-1
            
        for trk in self.tracks:
            trk.matched=False
        
        for d,det in enumerate(dets):
            if(d not in matched_indices[:,0] ):
                if(det.conf>=0.5):
                
                    self.tracks.append(Track(self.tracking_method,self.cur_id,det,frame_gray))
                    
                    self.cur_id+=1
                
                
            else:
                index = np.where(matched_indices[:,0]==d)
                index = matched_indices[index][0][1]
                self.tracks[index].update(det,frame_gray,prev_frame_gray)
        
        for t,trk in enumerate(self.tracks):
            if(t not in matched_indices[:,1] and trk.matched==False):
                
                trk.missed_count+=1

                
                trk.apply_prediction(frame_gray,prev_frame_gray)
       
        self.frameCount+=1
    def get_display_tracks(self):
        
        another = []
        
        
        self.tracks = [track for track in self.tracks if track.conf>=0]
        return [track for track in self.tracks if track.conf>0.05]
        
    
   
       
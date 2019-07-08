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
from utils import get_distance
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
    def calc_flow_fb(self,frame_gray,prev_frame_gray):
        frame_gray = cv.cvtColor(frame_gray,cv.COLOR_BGR2GRAY)
        prev_frame_gray = cv.cvtColor(prev_frame_gray,cv.COLOR_BGR2GRAY)
        flow  = cv.calcOpticalFlowFarneback(prev_frame_gray,frame_gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        for track in self.tracks:
            track.flow = flow;
    def calc_flow_kp(self,frame_gray,prev_frame_gray):
        frame_gray = cv.cvtColor(frame_gray,cv.COLOR_BGR2GRAY)
        prev_frame_gray = cv.cvtColor(prev_frame_gray,cv.COLOR_BGR2GRAY)
        mask = np.zeros_like(frame_gray)
        mask[:] = 0
        for track in self.tracks:
            cv.rectangle(mask,(int(track.xmin),int(track.ymin)),(int(track.xmax),int(track.ymax)),255,-1)
        p0 = cv.goodFeaturesToTrack(prev_frame_gray, mask = mask, **self.feature_params)
        if(not p0 is None ):
            p1, _st, _err = cv.calcOpticalFlowPyrLK(prev_frame_gray, frame_gray, p0, None, **self.lk_params)
            for track in self.tracks:
                track.new_points = get_points_in_bb(np.float32(p1).reshape(-1, 2),track.corners())
                track.prev_points = get_points_in_bb(np.float32(p0).reshape(-1, 2),track.corners())
    def calc_flow_backup(self,frame_gray,prev_frame_gray):
        start = time.time()
        frame_gray = cv.cvtColor(frame_gray,cv.COLOR_BGR2GRAY)
        prev_frame_gray = cv.cvtColor(prev_frame_gray,cv.COLOR_BGR2GRAY)
        all_points = []
        
        mask = np.zeros(frame_gray.shape, dtype = "uint8")
        
        
        for track in self.tracks:
            all_points.extend(track.new_points)
        #print(all_points)
        if(len(all_points)>0):
            p0 = np.float32([tr for tr in all_points]).reshape(-1,1, 2)
            #print('p0 is ',p0)
            p1, _st, _err = cv.calcOpticalFlowPyrLK(prev_frame_gray, frame_gray, p0, None, **self.lk_params)
            p0r, _st, _err = cv.calcOpticalFlowPyrLK(frame_gray, prev_frame_gray, p1, None, **self.lk_params)
            d = abs(p0-p0r).reshape(-1, 2).max(-1)
            good = d < 1
            new_tracks = []
            updated_old = []
            for tr, (x, y), good_flag in zip(all_points, p1.reshape(-1, 2), good):
                if not good_flag:
                    continue
                all_points.append(tr)
                updated_old.append(tr)
                if len(all_points) > self.track_len:
                    del all_points[0]
                new_tracks.append((x,y))
            print(self.frameCount,':','new points are', len(new_tracks))
            print(self.frameCount,':','old points are', len(updated_old))
            #print(new_tracks)
            if not self.frameCount % self.detect_interval == 0:
                for track in self.tracks:
                    track.new_points = get_points_in_bb(new_tracks,track.corners())
                    track.prev_points = get_points_in_bb(updated_old,track.corners())
            all_points = new_tracks
        
        if self.frameCount % self.detect_interval == 0:
            mask = np.zeros_like(frame_gray)
            mask[:] = 0
            for track in self.tracks:
                cv.rectangle(mask,(int(track.xmin),int(track.ymin)),(int(track.xmax),int(track.ymax)),255,-1)
            for x, y in all_points:
                cv.circle(mask, (x, y), 5, 0, -1)
            
            p = cv.goodFeaturesToTrack(frame_gray, mask = mask, **self.feature_params)
            if p is not None:
                for x, y in np.float32(p).reshape(-1, 2):
                    cv.circle(mask, (x, y), 5, 255, -1)
                    all_points.extend([(x, y)])
            cv.imwrite('debug_frames/ex%s.jpg'%self.frameCount,mask)
            print(self.frameCount,':',len(all_points),' points were found')
            for track in self.tracks:
                print(self.frameCount,':', len(get_points_in_bb(all_points,track.corners())),' points for track ',track.corners())
                track.new_points =  get_points_in_bb(all_points,track.corners())
        self.flow_time+=(time.time()-start)
    def get_distance_matrix(self,dets,frame):
        dists = np.zeros((len(dets),len(self.tracks)),np.float32)
        for itrack in range(len(self.tracks)):
            for ipred in range(len(dets)):
                #iou_dist = (1- iou(dets[ipred].corners(),self.tracks[itrack].corners()))
                
                desc_dist = np.linalg.norm(dets[ipred].hog-self.tracks[itrack].hog,ord=1)
                
                #desc_dist = get_distance(dets[ipred].descriptor , self.tracks[itrack].descriptor)
                
                iou_overlap = iou(dets[ipred].corners(),self.tracks[itrack].corners())
              
                uncertainety =np.maximum(1-dets[ipred].conf,0.5)
                #if(iou_dist==1):
                    #iou_dist=3
               # if(iou_dist<0.7):
                    #iou_dist = 0
                dists[ipred,itrack] = uncertainety*((1-iou_overlap)+desc_dist)
                
                #dists[ipred,itrack] = ((1-iou_overlap)+desc_dist)
        return dists
    def track(self,dets,frame_gray,prev_frame_gray):
#        if(self.frameCount>1 ):
#            if(self.tracking_method=='keypoint_flow'):
#                self.calc_flow(frame_gray,prev_frame_gray)
#            elif(self.tracking_method=='dense_flow'):
#                self.calc_flow_fb(frame_gray,prev_frame_gray)
        
        dists = self.get_distance_matrix(dets,frame_gray)
        matched_indices = linear_assignment(dists)
        
        for m in matched_indices:
            #descr_t = self.tracks[m[1]].descriptor
            #descr_p = dets[m[0]].descriptor

#            if(self.tracks[m[1]].missed_count<3):
#                iou_threshold=1.5
#            elif(self.tracks[m[1]].missed_count<8):
#                iou_threshold=1.8
#            else:
#                iou_threshold=1.9
                
            
            iou_dist = iou(dets[m[0]].corners(),self.tracks[m[1]].corners())
            if((dists[m[0],m[1]]>0.7 and iou_dist<0.2 )or (iou_dist <=0)):#iou_threshold):
            #if((dists[m[0],m[1]]>1.8 and iou_dist<0.2 )or (iou_dist <=0)):#iou_threshold):
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
                
#                if(trk.tracked_count<10):
#                    if(trk.missed_count>6):
#                        trk.conf = 0.2 #hide
#                    elif(trk.missed_count>4):
#                        trk.conf = 0.3 #remove
#                elif(trk.tracked_count<30):
#                    if(trk.missed_count>12):
#                        trk.conf=0.2 #hide
#                    elif(trk.missed_count>8):
#                        trk.conf = 0.3 #remove
#                else:
#                    if(trk.missed_count>25):
#                        trk.conf=0.2 #hide
#                    elif(trk.missed_count>20):
#                        trk.conf = 0.3 #remove
#                if(trk.missed_count>30):
#                    trk.conf=0 #remove
                
                trk.apply_prediction(frame_gray,prev_frame_gray)
       
        self.frameCount+=1
    def get_display_tracks(self):
        
        another = []
        
        
        self.tracks = [track for track in self.tracks if track.conf>=0]
        return [track for track in self.tracks if track.conf>0.05]
        
    def get_collision_points(self):
        cols = []
        objs = self.get_display_tracks()
        for obj in objs:
            if(self.point_in_collison(obj.botleft())):
                cols.append(obj.botleft())
            elif(self.point_in_collison(obj.botright())):
                cols.append(obj.botright())
            elif(self.point_in_box(self.A,obj)):
                cols.append(np.array([self.A[0],obj.ymax]))
           
            
        return cols
    def point_in_collison(self,p):
        
        if(p[1]<self.frame_height/2):
            return False
        
        q1 = (5*self.frame_height *p[0])/self.frame_width + 3*p[1] -4*self.frame_height
        q2 = (5*self.frame_height *p[0])/self.frame_width - 3*p[1] -self.frame_height
        if((q1>0 and q2>0) or (q1<0 and q2<0)):
            return False
        return True
    def point_in_box(self,p,box):
        return p[0]>box.xmin and p[0]<box.xmax and p[1]>box.ymin and p[1]<box.ymax
       
### Tensorflow 1.9
import sys
sys.path.append('../detection/')
import object_detection_lib
import time
import threading
import numpy
import traceback
import cv2
import platform
import sys
#TODO ALL: import ROS/RR libraries


class ObjectDetectionNode:
    def __init__(self):
        #detection initialization
        self.__odc = object_detection_lib.ObjectDetection(0.8,'../detection/inference_files')
        self.current_frame=cv2.imread('test.png')
        #TODO RR: initialize parameters in your service definition, connect to services here
        #TODO ROS: initialize publisher and subscriber here

    #TODO RR: include start_streaming/stop_streaming similar in webcam service, to make detect function running in background
    def detect(self):
        return self.__odc.scan_for_objects(self.current_frame)

        #TODO RR: modify this function to get image from webcam service, convert image to opencv object and process the image to get detection results, store that in your property

    #TODO ROS: include callback function for image subscriber, convert image to opencv object and process the image to get detection results, publish them

def main():
    #TODO ROS: initialize rosnode
    #TODO RR: register service type from file, register service 
    
    odc = ObjectDetectionNode()
    detected_list,centroids_list=odc.detect()       #return detected object names and centroid list (row, column)
    print(detected_list,centroids_list)
    #show processed image
    cv2.namedWindow("Image")
    cv2.imshow("Image",odc.current_frame )   
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    #TODO ALL: hold the script from exiting
if __name__ == '__main__':
    main()

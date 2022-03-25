#! /usr/bin/env python

## @package orient_correction
# The ROS node for prediction the orientation of the box
#
# Defines a ROS action server for predicting the orientation of the box from an image

import rospy
import actionlib
from bark_msgs.msg import OrientCorrectionAction, OrientCorrectionResult
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

## CorrectOrient class
#
# Defines a ROS action server for predictiong the orientation of the box
class CorrectOrient:
    '''
    Class for correcting orientation
    '''
    ## Constructor of LaserModul class
    #
    def __init__(self):
        ## @var bridge
        #  CvBridge() object for conversions between numpy arrays and ROS Image message types
        self.bridge = CvBridge()

        ## @var server
        #  The ROS action server for the OrientCorrectionAction action. The name of the action is "orient_correction"
        self.server = actionlib.SimpleActionServer('orient_correction', OrientCorrectionAction, self.execute, False)
        self.server.start()


    ## OrientCorrectionAction callback
    # This function gets called whenever the ROS action server receives a goal from a client
    # @param goal bark_msgs/OrientCorrectionGoal type action goal, it contains an image for predicting the orientation of the box (see action definition for further details)
    def execute(self, goal):
        '''
        OrientCorrectionAction callback

        goal: bark_msgs/OrientCorrectionGoal, action goal, it contains an image for predicting the orientation of the box (see action definition for further details)
        '''
        # INPUT: cv2 BGR image
        # OUTPUT: correctionangle in degres, clockwise
        image = self.bridge.imgmsg_to_cv2(goal.image, "bgr8")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 200, None, 3)

        cdst = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        linesP = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, None, 100, 10)
        Alpha = np.zeros(len(linesP),)  
        cdstP = np.copy(cdst)

        if linesP is not None:
            for i in range(0, len(linesP)):
                l = linesP[i][0]
                cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
                dx = l[0]-l[2]
                dy = l[1]-l[3]
                Delta = np.linalg.norm([dx, dy])
                Alpha[i] = np.degrees(np.arcsin(dy/Delta))
        bins = int(np.amax(Alpha)-np.amin(Alpha))
        Histogram = np.histogram(Alpha, bins)

        PeakIndex = np.argmax(Histogram[0])
        PeakAngle = Histogram[1][PeakIndex]
        result = OrientCorrectionResult()
        result.angle = PeakAngle

        self.server.set_succeeded(result)
    
    
if __name__ == '__main__':
    rospy.init_node('orient_correction_server')

    server = CorrectOrient()
    rospy.spin()
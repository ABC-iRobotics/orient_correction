import rospy
import actionlib
from bark_msgs.msg import OrientCorrectionAction, OrientCorrectionResult
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2


class CorrectOrient:

    def __init__(self):
        self.bridge = CvBridge()
        self.server = actionlib.SimpleActionServer('orient_correction', OrientCorrectionAction, self.execute, False)
        self.server.start()


    def execute(self, goal):
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
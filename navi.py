import rospy  
import actionlib  
import collections
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, PoseStamped  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal    
from math import pow, sqrt
import cv2.aruco as aruco
import rospy, cv2, cv_bridge, numpy, math
from smach import State,StateMachine
from sensor_msgs.msg import Image,CompressedImage
import time,smach_ros
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
import os


class RobotRun:
    def __init__(self):
        self.finished=0
        self.locations={1:((4.02028627077,4.18346248303,0.0),(0.0,0.0,0.998375561307,0.0569757719158)),
                        2:((-0.0817270829501-0.1,3.92411381007,0),(0,0,-0.7,0.71)),
                        4:((4.18356752335-0.1,0.100314113691+0.07,0.0),(0,0,0.9998,0.02)),
                        3:((0.084077691137,0.00733391767957-0.02,0.0),(0,0,0.15,0.99)),
                        5:((-2.75443580179,0.202781882274, 0.0),(0.0,0.0,-0.315998295757,0.948759757304)),
                        6:((3.45965783139,3.82832585195,0.0),(0.0,0.0,-0.99234481702,0.12349803291))
                        }       
        self.ckpt={1:((3.23534612934-0.2,3.32886478703-0.04,0.0),(0.0,0.0,-0.986964821947,0.160936137147)),
                   2:((2.06065698749+0.05,3.35087398272,0.0),(0.0,0.0,0.0301346368705,0.999545848704))
        }
        # pre set 
        rospy.init_node('movebase_client_py')
        self.move_base=actionlib.SimpleActionClient('move_base',MoveBaseAction)
        initial_pose = PoseWithCovarianceStamped()
        rospy.loginfo("Click on the map in RViz to set the intial pose...")  
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)  
        self.last_location = Pose()  
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
        keyinput = int(input("Input 0 to continue,or reset the initialpose!\n"))
        while keyinput != 0:
            rospy.loginfo("Click on the map in RViz to set the intial pose...")  
            rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)  
            rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
            keyinput = int(input("Input 0 to continue,or reget the initialpose!")) 
        while initial_pose.header.stamp == "":  
            time.sleep(1)
            rospy.sleep(1)
        rospy.loginfo("Starting running")
        rospy.loginfo("Updating current pose.")
        initial_pose.header.stamp = ""
        self.goal = MoveBaseGoal()  
 
    def go(self,x): 
        self.goal.target_pose.pose = Pose(Point(self.locations[x][0][0],self.locations[x][0][1],self.locations[x][0][2]),
        Quaternion(self.locations[x][1][0],self.locations[x][1][1],self.locations[x][1][2],self.locations[x][1][3])) # start-point
        self.goal.target_pose.header.frame_id = 'map'  
        self.goal.target_pose.header.stamp = rospy.Time.now()
        rospy.loginfo("Going to: " + str(x))
        self.move_base.send_goal(self.goal)
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(220))
        if not finished_within_time:  
                self.move_base.cancel_goal()  
                rospy.loginfo("Timed out achieving goal")  
        else:  
                state = self.move_base.get_state()  
                if state == GoalStatus.SUCCEEDED:  
                    rospy.loginfo("Ready to start!")
                else:  
                    rospy.loginfo("Goal failed with error code: " + str(self.goal_states[state]))

    def gotockpt(self,x):
        self.goal.target_pose.pose = Pose(Point(self.ckpt[x][0][0],self.ckpt[x][0][1],self.ckpt[x][0][2]),
        Quaternion(self.ckpt[x][1][0],self.ckpt[x][1][1],self.ckpt[x][1][2],self.ckpt[x][1][3])) # start-point
        self.goal.target_pose.header.frame_id = 'map'  
        self.goal.target_pose.header.stamp = rospy.Time.now()
        rospy.loginfo("Going to ckpt:" + str(x))
        self.move_base.send_goal(self.goal)
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(180))
        if not finished_within_time:  
                self.move_base.cancel_goal()  
                rospy.loginfo("Timed out achieving goal")  
        else:  
            pass

        
    def park(self,f1=0,f2=0):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # self.aruco_sub = rospy.Subscriber('/aruco_single/pose',PoseStamped,queue_size=10,callback=self.aruco_callback) 
        self.twist = Twist()
        while True:
            # rospy.init_node('aruco_single')
            res=rospy.wait_for_message('/aruco_single/pose', PoseStamped, timeout=None)
            xl=res.pose.position.x
            yl=res.pose.position.y
            zl=res.pose.position.z
            x=res.pose.orientation.x
            y=res.pose.orientation.y
            z=res.pose.orientation.z
            w=res.pose.orientation.w
            xx = x*x
            yy = y*y
            zz = z*z
            ret=numpy.zeros((4,4),dtype='float64')
            ret[0,0] = 1-2*(y*y+z*z)
            ret[0,1] = 2*(x*y-w*z)
            ret[0,2] = 2*(w*y+x*z)
            ret[1,0] = 2*(x*y+w*z)
            ret[1,1] = 1-2*(x*x+z*z)
            ret[1,2] = 2*(y*z-w*x)
            ret[2,0] = 2*(x*z-w*y)
            ret[2,1] = 2*(y*z+w*x)
            ret[2,2] = 1-2*(x*x+y*y)
            ret[3,3] = 1
            bef=numpy.array([xl,yl,zl,1])
            new=numpy.dot(ret,bef)
            newx=new[0]
            newy=new[1]
            newz=new[2]
            rospy.loginfo(''+str(new))
            xlmt=0.004
            
            if(newx<-0.036-xlmt) and not f1:
                if (z>-0.2):
                    self.direction(0)
                elif (z<-0.24):
                    self.direction(1)
                else:
                    self.direction(2)
            elif (newx>-0.036+xlmt) and not f1:
                if (z<0.2):
                    self.direction(1)
                elif z>0.24:
                    self.direction(0)
                else:
                    self.direction(2)
            else:
                f1=True
                if z>0.014:
                    self.direction(0)
                elif z<-0.014:
                    self.direction(1)
                elif zl>0.27:
                    self.direction(2)
                else:
                    self.direction(8)
                    break


    def direction(self,i):
        if i==0:
            self.twist.angular.z=0.1
            self.twist.linear.x=0
        elif i == 1:
            self.twist.angular.z=-0.1
            self.twist.linear.x=0
        elif i ==2 :
            self.twist.angular.z=0
            self.twist.linear.x=0.14
        elif i==8:
            self.twist.angular.z=0
            self.twist.linear.x=0
        self.cmd_vel_pub.publish(self.twist)
    
    

    def update_initial_pose(self, initial_pose):  
        self.initial_pose = initial_pose
    def shutdown(self):  
        rospy.loginfo("Stopping the robot...")  
        self.move_base.cancel_goal()  
        rospy.sleep(1)  
        self.cmd_vel_pub.publish(Twist())  
        rospy.sleep(1)

class ROOT(State):
    def __init__(self):
        State.__init__(self, outcomes=['ot2','ot3','ot4'])
    def execute(self, ud):
        global r,lap
        if lap==0:
            r.gotockpt(1)
            return 'ot2'
        else: # when the robot has run a lap
            r.go(6)
            while markID==0:
                rospy.wait_for_message('raspicam_node/image/compressed', CompressedImage)  
                a=aruco_detecter()
            r.gotockpt(1)
            if markID==2:
                return 'ot2'
            if markID==3:
                return 'ot3'
            if markID==4:
                return 'ot4'

            
            
class S2(State):
    def __init__(self):
        State.__init__(self, outcomes=['2ot3','2otf'])
    
    def execute(self, ud):
        global r
        r.go(2)
        if lap==0:
            return '2ot3'
        else:
            r.shutdown()
            return '2otf'

class S3(State):
    def __init__(self):
        State.__init__(self, outcomes=['3ot4','3otf'])
    def execute(self, ud):
        global r,lap
        r.go(3)
        if lap==0:
            return '3ot4'
        else:
            r.shutdown()
            return '3otf'
    
class S4(State):
    def __init__(self):
        State.__init__(self, outcomes=['4ot1','4otf'])
    
    def execute(self, ud):
        global lap,r
        r.go(4)
        if lap==0:
            lap=1
            r.gotockpt(2)
            r.go(1)
            return '4ot1'
        else:
            r.shutdown()
            return '4otf'

class aruco_detecter():
    def __init__(self):
        self.ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}
        self.soundhandle = SoundClient()
        self.bridge=cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('raspicam_node/image/compressed', CompressedImage, self.aruco_detect)


    def aruco_detect(self,msg):
        global markID,find
        image=self.bridge.compressed_imgmsg_to_cv2(msg,desired_encoding='bgr8')
        h, w, d = image.shape
        # bev = cv2.warpPerspective(image, self.homography, (w,h)) 
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)    
        flag, gray_thresh = cv2.threshold(gray,125,255,cv2.THRESH_BINARY)
        gray_mask = 255-gray_thresh
        gray_mask[gray<10] = 0
        this_aruco_dictionary = cv2.aruco.Dictionary_get(self.ARUCO_DICT["DICT_ARUCO_ORIGINAL"])
        this_aruco_parameters = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(image, this_aruco_dictionary, parameters=this_aruco_parameters)
        if len(corners) > 0 and not find:
	        for marker_id in ids:
                    rospy.loginfo("Find Aruco! ID: "+str(marker_id[0]))
                    # os.system("python sound.py")
                    # self.soundhandle.say(str(marker_id[0]), 'voice_kal_diphone', 1)
                    for i in range(marker_id[0]):
                        self.soundhandle.say(str(marker_id[0]), 'voice_kal_diphone', 1) 
                        rospy.sleep(1)
                    #     rospy.loginfo(i)
                    if marker_id[0]==4:
                        os.system("python sound.py")
                    else:
                        for i in range(marker_id[0]):
                            os.system("python sound_copy.py")
                    markID = marker_id
                    find=1
r=None
lap=0
markID=0
find=0

def main():

    # Create a SMACH state machine
    sm = StateMachine(outcomes=['finish'])
    global r
    r=RobotRun()
    # Open the container
    with sm:
        # Add states to the container
        StateMachine.add('ROOT', ROOT(), 
                               transitions={'ot2':'s2', 
                                            'ot3':'s3',
                                            'ot4':'s4'})
        StateMachine.add('s2', S2(), 
                               transitions={'2ot3':'s3',
                                            '2otf':'finish'})
        StateMachine.add('s3', S3(), 
                               transitions={'3ot4':'s4',
                                            '3otf':'finish'})
        StateMachine.add('s4', S4(), 
                               transitions={'4ot1':'ROOT',
                                            '4otf':'finish'})



    # Execute SMACH plan
    outcome = sm.execute()
# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    main()
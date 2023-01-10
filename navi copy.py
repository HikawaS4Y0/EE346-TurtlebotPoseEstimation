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


class RobotRun:
    def __init__(self):
        self.state=2
        self.finished=0
        self.locations={1:((4.02028627077,4.18346248303,0.0),(0.0,0.0,0.998375561307,0.0569757719158)),
                        2:((-0.0817270829501,3.92411381007,0),(0,0,-0.7,0.71)),
                        4:((4.18356752335,0.100314113691,0.0),(0,0,0.9998,0.02)),
                        3:((0.084077691137,0.00733391767957,0.0),(0,0,0.15,0.99)),
                        5:((-2.75443580179,0.202781882274, 0.0),(0.0,0.0,-0.315998295757,0.948759757304)),
                        6:((3.45965783139,3.82832585195,0.0),(0.0,0.0,-0.99234481702,0.12349803291))
}       
        self.ckpt={1:((3.23534612934,3.32886478703,0.0),(0.0,0.0,-0.986964821947,0.160936137147)),
                   2:((2.06065698749,3.35087398272,0.0),(0.0,0.0,0.0301346368705,0.999545848704))
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
            rospy.sleep(1)
        rospy.loginfo("Starting running")

        rospy.loginfo("Updating current pose.")
        initial_pose.header.stamp = ""
        self.goal = MoveBaseGoal() 
        self.go()

        
            
    
 
    def go(self): 
        if self.state==7:
            self.move_base.cancel_all_goals()
            self.park()
            return
        elif self.state==2:
            self.gotockpt(1)
        elif self.state==5:
            self.gotockpt(2)
        x=self.state
        if x==5:
            x=1 
        
        self.goal.target_pose.pose = Pose(Point(self.locations[x][0][0],self.locations[x][0][1],self.locations[x][0][2]),
        Quaternion(self.locations[x][1][0],self.locations[x][1][1],self.locations[x][1][2],self.locations[x][1][3])) # start-point
        self.goal.target_pose.header.frame_id = 'map'  
        self.goal.target_pose.header.stamp = rospy.Time.now()
        rospy.loginfo("Going to: " + str(x))
        self.move_base.send_goal(self.goal)

        finished_within_time = self.move_base.wait_for_result(rospy.Duration(180))
        if not finished_within_time:  
                self.move_base.cancel_goal()  
                rospy.loginfo("Timed out achieving goal")  
        else:  
                state = self.move_base.get_state()  
                if state == GoalStatus.SUCCEEDED:  
                    rospy.loginfo("Ready to start!")
                    self.state+=1
                    self.go()
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
        super().__init__(self, outcomes=['ot2','ot3','ot4'], input_keys=['aruco_num'])
    def execute(self, ud):
        

class S2(State)

    
# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        RobotRun()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from turtlebot3_msgs.msg import Sound
# # self.soundhandle.say(str(marker_id[0]), 'voice_kal_diphone', 1)

# soundhandle=SoundClient()
# soundhandle.say('2', 'voice_kal_diphone', 1)
# soundhandle.stopAll()
# # soundhandle.say("hello world", 'voice_kal_diphone', 1)
# soundhandle.sendMsg()
# rospy.loginfo("ok")
# print('okk')
def beep(i):
    pub = rospy.Publisher('sound', Sound, queue_size=10)
    rospy.init_node('s', anonymous=True)
    # rate = rospy.Rate(10)
    msg=Sound()
    msg.value=i
    pub.publish(msg)
    # for i in range(4):
    #     pub.publish(msg)
    rospy.loginfo("sound")

# beep(1)
pub = rospy.Publisher('sound', Sound, queue_size=10)
rospy.init_node('s', anonymous=True)
# rate = rospy.Rate(10)
msg=Sound()
msg.value=1
pub.publish(msg)
rospy.loginfo("sound 123")
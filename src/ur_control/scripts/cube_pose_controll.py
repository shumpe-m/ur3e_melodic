import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
import numpy as np
class cube_pose_controll():
    def __init__(self):
        pass

    def pose_controll(self):
        rand = np.random.rand(2)
        state_msg = ModelState()
        state_msg.model_name = 'cube3'
        state_msg.pose.position.x = rand[0] * 0.025 - 0.35
        state_msg.pose.position.y = rand[1] * 0.15
        state_msg.pose.position.z = 0.81
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 0

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )

        except (rospy.ServiceException, e):
            print("Service call failed: %s" % e)

if __name__ == '__main__':
    try:
        cube_pose_controll.pose_controll()
    except rospy.ROSInterruptException:
        pass
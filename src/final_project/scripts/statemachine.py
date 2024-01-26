#!/usr/bin/env python3

import threading
import rospy
import time
import actionlib
from smach import State, StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from std_msgs.msg import Empty

start_time=0
end_time=0

waypoints = []

wpp_list = {1 : (4.5, -0.5),
           2 : (5.0, -4.3),
           3 : (-6.25, -3.9),
           4 : (-12.5, -0.7),
           5 : (-8.5, 0.02)}

# AMCL NODE
def convert_PoseWithCovarianceStamped_to_PoseArray(warypoints):
    """Publishing waypoints as pose array
        You can see at rviz"""
    poses=PoseArray()
# YOU can change frame_id as "odom" or "map"...
    poses.header.frame_id="map"
    poses.poses=[pose.pose.pose for pose in waypoints]

    return poses

class GetPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'], output_keys=['waypoints'])

        #Create publisher to publish waypoints.
        self.pose_array_publisher=rospy.Publisher('waypoints', PoseArray, queue_size=1)
        # if they can't receive the info, initialize as ...
        self.custom_waypoint_topic=rospy.get_param('~custom_waypoints_topic', 'my_waypoint_list')

        # start thread to wait for reset messages to initialize waypoints list.
        def wait_for_path_reset():
            """thread worker function"""
            global waypoints
            while not rospy.is_shutdown():
                data=rospy.wait_for_message('path_reset', Empty)
                # rospy.loginfo("Received path RESET message")
                # function which initialize the queue for path
                self.initialize_path_queue()
                rospy.sleep(3) # wait 3(s) for message because 'rostopic echo' latches.. you can change this
                # Usage of Thread
        reset_thread=threading.Thread(target=wait_for_path_reset)
        reset_thread.start()
    def initialize_path_queue(self):
        global waypoints
        waypoints=[]

        # Publish empty waypoints queue.
        self.pose_array_publisher.publish(convert_PoseWithCovarianceStamped_to_PoseArray(waypoints))

    def execute(self, userdata):
        global waypoints
        self.initialize_path_queue()
        self.path_ready=False

        # Start thread to listen for when the path is ready
        def wait_for_path_ready(): # for thread
            """thread worker function"""
            data=rospy.wait_for_message('path_ready', Empty)
            # rospy.loginfo("Received path READY message")
            self.path_ready=True

        ready_thread=threading.Thread(target=wait_for_path_ready)
        ready_thread.start()

        waypoints_topic=self.custom_waypoint_topic
        # rospy.loginfo("Waiting to receive waypoints via Pose msg on topic %s" % waypoints_topic)
        # rospy.loginfo("To start following waypoints: 'rostopic pub /path_ready std_msgs/Empty -1")

        # Wait for published waypoints
        while not self.path_ready:
            try:
                # waiting for waypoints_topic type: Pose~
                pose=rospy.wait_for_message(waypoints_topic, PoseWithCovarianceStamped, timeout=1)
            except rospy.ROSException as e:
                if 'timeout exceeded' in str(e):
                    continue
                else:
                    raise e
            # rospy.loginfo("Received new point")
            waypoints.append(pose)

            self.pose_array_publisher.publish(convert_PoseWithCovarianceStamped_to_PoseArray(waypoints))

        return 'success'



#######################################################################################################################
class FollowPath(State):
    def __init__(self):
        # both state, result are string type
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'])
        #parameter defined in launch file
        self.frame_id=rospy.get_param('~goal_frame_id', 'map')

        # Get a move_base action client
        #define the clients
        self.client=actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # rospy.loginfo('Connecting to move_base')
        self.client.wait_for_server()
        # rospy.loginfo('connected to move_base.')

#the range where the states operate
    def execute(self, userdata):
        global waypoints
        global start_time

        start_time=time.time()
        rospy.loginfo("Start time is " + str(start_time))

        for waypoint in waypoints:
            if waypoints == []:
                # rospy.loginfo("They waypoint queue has been reset.")
                break

            # Otherwise, publish next waypoint as goal
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id=self.frame_id
            # You need to write waypoint or get from function...
            goal.target_pose.pose.position=waypoint.pose.pose.position
            goal.target_pose.pose.orientation=waypoint.pose.pose.orientation


            if(goal.target_pose.pose.position.x== wpp_list[2][0] and goal.target_pose.pose.position.y == wpp_list[2][1]):
                # After getting food, wait for 5(s)
                rospy.loginfo("Get the food")
                rospy.sleep(5)
            elif(goal.target_pose.pose.position.x==wpp_list[1][0] and goal.target_pose.pose.position.y == wpp_list[1][1]):
                pass
            else:
                for i in range(3,6,1):
                    if(goal.target_pose.pose.position.x==wpp_list[i][0] and goal.target_pose.pose.position.y==wpp_list[i][1]):
                        rospy.loginfo("Serving at table%s is complete"%i)

            # rospy.loginfo("Executing move_base goal to position (x,y): %s, %s"%(waypoint.pose.pose.position.x, waypoint.pose.pose.position.y))
            # rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
            self.client.send_goal(goal=goal)
            self.client.wait_for_result()
        return "success"


################################ Last states ###################################
class PathComplete(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        global end_time
        end_time = time.time()
        total_time = end_time - start_time
        rospy.loginfo("################################")
        rospy.loginfo("##### I bring you a plate. #####")
        rospy.loginfo("### Please return the dishes ###")
        rospy.loginfo("Total Execution Time: {:.2f} seconds".format(total_time))
        rospy.loginfo("################################")
        return "success"

################################################################################################
# followed 3states will be operated in state machine
def main():
    rospy.init_node("custom_follow_waypoints")
    sm=StateMachine(outcomes=["success"])

    with sm:
        StateMachine.add('GET_PATH', GetPath(), transitions={'success':'FOLLOW_PATH'}, remapping={'waypoints':'waypoints'})
        StateMachine.add('FOLLOW_PATH', FollowPath(), transitions={'success':'FOLLOW_COMPLETE'}, remapping={'waypoints':'waypoints'})
        StateMachine.add('FOLLOW_COMPLETE', PathComplete(), transitions={'success':'GET_PATH'})
    outcome=sm.execute()

if __name__ == '__main__' :
    main()


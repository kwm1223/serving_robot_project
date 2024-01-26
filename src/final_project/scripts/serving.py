#! /usr/bin/env python3

#if using noetic line1 -> #! /usr/bin/env python3


import rospy
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback

startPoint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

pickupPoint = [6.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

#goalList = serving point + endpoint
goalList = [[7.5, -4.0, 0.0, 0.0, 0.0, 1.0, 0.0],
             [-4.0, -4.0, 0.0, 0.0, 0.0, 1.0, 0.0],
             [-10.0, -0.5, 0.0, 0.0, 0.0, -1.0, 0.0],
             ]

endPoint = [
             #[2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
             [-6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
             ]

goalListLength = len(goalList)

#for Test
testList = [[-10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
             [-6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]]


def setLocation(location,n=0):
    #rospy.loginfo("going to x: %d y : %d"%(location[0],location[1]))
    goal_msg = MoveBaseGoal()
    goal_msg.target_pose.header.frame_id = 'map'
    goal_msg.target_pose.pose.position.x = location[0]
    goal_msg.target_pose.pose.position.y = location[1]
    goal_msg.target_pose.pose.position.z = location[2]
    goal_msg.target_pose.pose.orientation.x = location[3]
    goal_msg.target_pose.pose.orientation.y = location[4]
    goal_msg.target_pose.pose.orientation.z = location[5]
    goal_msg.target_pose.pose.orientation.w = location[6]
    
    # sends the goal to the action server, specifying which feedback function
    # to call when feedback received
    client.send_goal(goal=goal_msg, feedback_cb=feedback_callback)
    client.wait_for_result()

def feedback_callback(msg):
    """
    definition of the feedback callback. This will be called when feedback
    is received from the action server
    it just prints a message indicating a new message has been received
    """
    pass
    #rospy.loginfo("[Feedback] Going to Goal Pose...")

# initializes the action client node
rospy.init_node('move_base_action_client')

# create the connection to the action server
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

# waits until the action server is up and running
client.wait_for_server()

#set robot start point
setLocation(startPoint)

#Input Enter key for starting
while True:
    input_start = input("Press Enter to start")
    if input_start:
        break

#start Time
startTime = rospy.get_time()

rospy.loginfo("#"*30)
rospy.loginfo("Start Serving!!")
rospy.loginfo("#"*30)

#pick Up point
setLocation(pickupPoint)
rospy.loginfo("Picking up some Drinks...")
time.sleep(3)
rospy.loginfo("Completed Picking up Drinks!!")
rospy.loginfo("Complete Picking up Time : " + str(rospy.get_time()-startTime))
while not rospy.is_shutdown():
    while goalList:
        goal = goalList.pop(0)
        #rospy.loginfo(str(goal))
        rospy.loginfo("#"*30)
        rospy.loginfo("Serving to Table NO.%d..."%(goalListLength-len(goalList)))

        setLocation(goal)
        #rospy.loginfo("Time :" + str(rospy.get_time()))
        #if goalList :
        #after 
        #rospy.loginfo("#"*30)
        rospy.loginfo("Arrived Table NO.%d!"%(goalListLength-len(goalList)))
        rospy.loginfo("Arrive Time : " + str(rospy.get_time()-startTime))
        #else :
        #    break
        #after serving
        rospy.loginfo("Serving...")
        time.sleep(2)
        rospy.loginfo("Serving Completed!")
        rospy.loginfo("Completed Time : " + str(rospy.get_time()-startTime))
        rospy.loginfo("#"*30)

    rospy.loginfo("#"*30)
    rospy.loginfo("Return to Sink!")
    rospy.loginfo("#"*30)

    while endPoint :
        end = endPoint.pop(0)
        setLocation(end)

    endTime = rospy.get_time()
    rospy.loginfo("#"*30)
    rospy.loginfo("#"*5+"Completed Serving!!"+"#"*6)
    rospy.loginfo("#"*30)

    #rospy.loginfo("####End Time :" + str(endTime)+"#"*6)
    rospy.loginfo("####Serving Time : " + str(endTime-startTime)+ "####")

    rospy.loginfo("#"*30)

    break

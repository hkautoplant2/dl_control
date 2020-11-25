#! /usr/bin/env python



import rospy

import tf2_ros


from std_srvs.srv import EmptyResponse, EmptyRequest, Empty

import math

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from dl_control.srv import GoToTarget,GoToTargetRequest,GoToTargetResponse
from dl_control.srv import GetPos,GetPosRequest,GetPosResponse
UC_finished = False
stop_publishing_joints = False

x_curent = 0
y_curent = 0
z_curent = 0



recieved_angles = False
stop_publishing_joints = False
UC_finished = False
x_current = 0
y_current = 0
z_current = 0
omega = 0
alpha = 0
beta = 0
gamma = 0


# function used to set the stop pbulishing variable if publish_joint topic
def stop_pub_joints_cb(msg):
    global stop_publishing_joints
    if msg.data == False:
        #print("Keep publishing joints")
        stop_publishing_joints = False
    else:

        #print("STOP publishing joints")
        stop_publishing_joints = True


def finished_cb(msg):
    global UC_finished
    if msg.data == False:
        UC_finished = False
    else:
        UC_finished = True

def create_JointMsg(angles):


    """
    msg = JointMsg()

    msg.omega = angles[0]
    msg.alpha = angles[1]
    msg.beta = angles[2]
    msg.gamma = angles[3]
    """
    msg = Float32MultiArray()
    msg.data = angles


    return msg

# recursiv function, publiosh as long as the stop_publishing_joints is false
def publish_msg2(msg,pub):
    
    if stop_publishing_joints == True:
        #print("stop publishing joints")
        return
    else:
        #print("publishing to UC")
        rospy.sleep(0.4)
        pub.publish(msg)
        publish_msg2(msg,pub)
# recursiv function, reads angles from UC until finished is true
def read_sensor():

    if UC_finished == True:
        #print("UC done")
        return
    else:
        #print("UC finished topic=", UC_finished)
        rospy.sleep(0.1)
        #print("reading sensor")
        read_sensor()

def convert_gamma(gamma):
    gamma = gamma + math.pi/2
    l1 = 157
    l2 = 202
    l3 = 202
    l4 = 185
    l5 = math.sqrt(l1**2+l4**2-2*l1*l4*math.cos(gamma))
    
    A1_1 = (l1**2+l5**2-l4**2)/(2*l1*l5)
    B1_1 = (l2**2+l5**2-l3**2)/(2*l2*l5)

    A = math.acos(A1_1)
    B = math.acos(B1_1)

    gamma = A + B
    return gamma

# function takes a pos and calculated trough IK the required angles
# and publish it to the UC
def go_to_target(req):
    global omega
    global alpha
    global beta
    global gamma
    global x_current
    global y_current
    global z_current
    # tell UC angles are incoming

    #pub_bool = rospy.Publisher("UC_transmit_angles", Bool,queue_size = 10)
    #UC_transmit_angles = True
    #pub_bool.publish(UC_transmit_angles)

    x = req.x
    y = req.y
    z = req.z 

    # setting parameters
    A = 1590
    B = 1670
    C = 100
    L = 810
    #L = 500 # used as safety so it dosnt hit the floor

    #L = 1155

    print("goal position = ",x,y,z)



    beta_limit = 160
    alpha_limit =155 # degress


    omega = math.atan2(y,x)
    d = math.sqrt(x**2+y**2)
    dz = math.sqrt((z-L)**2+d**2)

    if d > A + B:
        print("ERROR: out of range")
        exit()
    beta = math.acos((A**2 + B**2 - dz**2)/(2*A*B))



    if beta < 0:
        beta = -beta


    alpha = math.acos((A**2+dz**2-B**2)/(2*A*dz))

    alpha = alpha + math.atan2((z-L),d)


    alpha = math.pi/2 + alpha
    beta = beta
    gamma = math.pi/2 - (math.pi - ((alpha-math.pi/2)+beta))
    gamma = convert_gamma(gamma);

    #print("goal angles=",[omega,alpha,beta,gamma])
    omega = round(omega*180/math.pi,2)
    alpha = round(alpha*180/math.pi,2)
    beta = round(beta*180/math.pi,2)
    gamma= round(gamma*180/math.pi,2)



    print("goal angles=",[omega,alpha,beta,gamma])
    if beta > beta_limit:
        print("required beta is out of limit")
        exit()

    if alpha > alpha_limit:
        print("required alpha is out of limit")
        exit()

    if beta < 0:
        beta = -beta

    if alpha < 0:
        alpha = -alpha
    # rewrite angles in degrees





    # set dimenstion of the Float32MultiArray

    # create the topic for the uc
    pub = rospy.Publisher("joint_IK", Float32MultiArray,queue_size = 1)
    target_angles = [omega,alpha,beta,gamma]
    joint_msg = create_JointMsg(target_angles)


    r = rospy.Rate(1)
    #print("angles=",joint_msg)

    stop_publishing_joints_sub = rospy.Subscriber("publish_joint",Bool,stop_pub_joints_cb)
    # publish joints untill UC has recieved the joints
    print("waiting for message from UC")
    uc_ready_sub = rospy.Subscriber("UC_ready_topic",Bool)
    #rospy.wait_for_message("UC_ready_topic",Bool)


    #print("start time is :",starttime)
    #takes 0.8 seconds
    print("publish the goal angle")
    publish_msg2(joint_msg,pub)
    #print("the time difference is:",timeit.default_timer() - starttime)





    print("goal angles=",[omega,alpha,beta,gamma])
    x_curent,y_curent,z_curent = FK()
    return x_current,y_curent,z_current

def FK_cb(msg):

    global x_current
    global y_current
    global z_current
    global recieved_angles
    recieved_angles = True



    omega = msg.data[0]
    alpha = msg.data[1]
    beta = msg.data[2]
    gamma = msg.data[3]

    omega = omega*math.pi/180
    alpha = alpha*math.pi/180
    beta = beta*math.pi/180
    gamma = gamma*math.pi/180
    print("current angles=",msg.data)

    A = 1590
    B = 1670
    C = 100
    L = 810

    dz = math.sqrt(A**2+B**2 - 2*A*B*math.cos(beta))
    alpha1 = math.acos((A**2 + dz**2 - B**2)/(2*A*dz))
    alpha2 = alpha - math.pi/2 - alpha1

    z = dz*math.sin(alpha2)+L 
    d = dz*math.cos(alpha2)
    x = math.sqrt(d**2/(math.tan(omega)**2+1))
    y = math.sqrt(d**2-x**2)

    x_current = x
    y_current = y
    z_current = z


    #print(z)
    #print(w)
    #w= float(w)
    """
    x_current = float(w_current/sec(omega))
    y_current = float(x_current*math.tan(omega))
    """

    # publish to Rviz
    """
    joint_states_msg = JointState()tlt-infer faster_rcnn -e /workspace/examples/faster_rcnn/specs/fastrcnn_retrain_pruned12.txt
    joint_states_msg.name = ['2_base_1_rot_base',"3_rot_base_arm1","4_arm1_arm2","5_arm2_kran",]
    joint_states_msg.position = [omega,alpha,beta,gamma]
    joint_states_msg.header.frame_id = "world"

    print("publishing",joint_states_msg)
    rviz_joint_pub = rospy.Publisher("joint_states",JointState,queue_size=10)
    rviz_joint_pub.publish(joint_states_msg)
    """
    #print("current pos=",x_current,y_current,z_current)


def FK():
    stop_publishing_joints_sub = rospy.Subscriber("publish_joint",Bool,stop_pub_joints_cb)


    joint_sensor_sub = rospy.Subscriber("Joint_State_uc",Float32MultiArray,FK_cb)
    finished_sub = rospy.Subscriber("finished",Bool,finished_cb)
    rospy.wait_for_message("Joint_State_uc",Float32MultiArray)
    print("read sensor data")
    read_sensor()



    return x_current,y_current,z_current


def reset_variables():
    global stop_publishing_joints
    global UC_finished


    stop_publishing_joints = False
    UC_finished = False

def get_pos_pub(msg,pub):
    rospy.sleep(0.1)
    #print("retrived angles=",recieved_angles)
    if recieved_angles == False:

        pub.publish(msg)

        get_pos_pub(msg,pub)
    else:
        return

def get_pos(msg):

    get_only_angles_pub = rospy.Publisher("get_angle_topic",Bool,queue_size = 10)
    print("Get the pos")
    get_pos_pub(True,get_only_angles_pub)


    print("current pos",x_current,y_current,z_current)

    return x_current,y_curent,z_current





def FK_IK_server():

    sub_stop = rospy.Subscriber("publish_joint_master",Bool,stop_pub_joints_cb)

    sub_joints = rospy.Subscriber("Joint_State_uc",Float32MultiArray,FK_cb)


    rospy.init_node("IK_FK_node")
    # get a target pos
    server_inv = rospy.Service("go_to_target",GoToTarget,go_to_target)
    server_fk = rospy.Service("get_pos",GetPos,get_pos)
    print("ready for input")
    rospy.spin()




def main():
    FK_IK_server()




if __name__ == '__main__':
	main()

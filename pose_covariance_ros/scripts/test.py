#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from math import sin,cos
import random 
from geometry_msgs.msg import PoseWithCovarianceStamped
np.set_printoptions(suppress=True)
import matplotlib.pyplot as plt

def random_float(low, high):
    return random.random()*(high-low) + low

thx = []

def AHfx( n,th,):
    T_a = np.eye(4, dtype=np.float64)
    T_a[0,3] = afx[n]
    T_d = np.eye(4, dtype=np.float64)
    T_d[2,3] = dfx[n]

    Rzt = np.array([[cos(th[n]), -sin(th[n]),     0 ,0],
                    [sin(th[n]),  cos(th[n]),     0, 0],
                    [0,               0,              1, 0],
                    [0,               0,              0, 1]],copy=False)
        

    Rxa = np.array([[1, 0,                  0,                     0],
                    [0, cos(alphfx[n]), -sin(alphfx[n]),   0],
                    [0, sin(alphfx[n]),  cos(alphfx[n]),   0],
                    [0, 0,                 0,                  1]],copy=False)

    A_i = T_d.dot(Rzt).dot(T_a).dot(Rxa)

    return A_i

def HTransfx(th):         
    T_06 = np.eye(4, dtype=np.float64)
    for i in range(0,len(th)):
        T_06 = np.dot(T_06,AHfx( i,th))

    return T_06


node     = rospy.init_node('test')
rospy.sleep(1)
DH_d     = np.asarray(rospy.get_param("~DH_d"))
DH_a     = np.asarray(rospy.get_param("~DH_a"))
DH_alpha = np.asarray(rospy.get_param("~DH_alpha"))
joint_pub_topic = rospy.get_param("~joint_pub_topic")
# DH_theta = np.asarray(rospy.get_param("~DH_theta"))

dfx    = DH_d.copy()
afx    = DH_a.copy()
alphfx = DH_alpha.copy()

jnt_msg = JointState()
jnt_msg.name = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint","wrist_3_joint"]


jnt_pub = rospy.Publisher(joint_pub_topic, JointState,queue_size=1)


x_tot   = np.asarray([])
y_tot   = np.asarray([])
z_tot   = np.asarray([])
x_c_vec = np.asarray([])
y_c_vec = np.asarray([])
z_c_vec = np.asarray([])
x_s_vec = np.asarray([])
y_s_vec = np.asarray([])
z_s_vec = np.asarray([])

# while not rospy.is_shutdown():
while len(x_tot) < 50:
    print(len(x_tot))
    thx = np.random.uniform(low=-2*np.pi, high=2*np.pi, size=(6,))
    
    jnt_msg.position = thx

    while not rospy.is_shutdown():
        jnt_msg.header.stamp = rospy.Time.now()
        jnt_pub.publish(jnt_msg)
        out_pose = None
        try:
            out_pose = rospy.wait_for_message("/pose_covariance_ros_node_test/pose_cov_wrist_3_joint",PoseWithCovarianceStamped,timeout=0.02)
        except:
            pass
        if out_pose is not None:
            break

    x_c = out_pose.pose.covariance[0]
    y_c = out_pose.pose.covariance[7]
    z_c = out_pose.pose.covariance[14]
    x_c_vec = np.append(x_c_vec,x_c)
    y_c_vec = np.append(y_c_vec,y_c)
    z_c_vec = np.append(z_c_vec,z_c)
    # x_c = out_pose.pose.pose.position.x
    # y_c = out_pose.pose.pose.position.y
    # z_c = out_pose.pose.pose.position.z

    x_s = np.asarray([])
    y_s = np.asarray([])
    z_s = np.asarray([])

    while len(x_s) < 1000:
        for ln in range(0,len(dfx)):
            if DH_d[ln] != 0:
                dfx[ln] = DH_d[ln] + np.random.normal(0, 0.005, 1)
            if DH_a[ln] != 0:
                afx[ln] = DH_a[ln] + np.random.normal(0, 0.005, 1)
            if DH_alpha[ln] != 0:
                alphfx[ln] = DH_alpha[ln] + np.random.normal(0, 0.005, 1)

        tf_o = HTransfx(thx)

        x_s = np.append(x_s,tf_o[0,3])
        y_s = np.append(y_s,tf_o[1,3])
        z_s = np.append(z_s,tf_o[2,3])

    

    # x_tot = np.append(x_tot,(np.std(x_s[-100:])-x_c))
    # y_tot = np.append(y_tot,(np.std(y_s[-100:])-y_c))
    # z_tot = np.append(z_tot,(np.std(z_s[-100:])-z_c))

    x_tot = np.append(x_tot,(np.std(x_s)-x_c))
    y_tot = np.append(y_tot,(np.std(y_s)-y_c))
    z_tot = np.append(z_tot,(np.std(z_s)-z_c))
    x_s_vec = np.append(x_s_vec,np.std(x_s))
    y_s_vec = np.append(y_s_vec,np.std(y_s))
    z_s_vec = np.append(z_s_vec,np.std(z_s))

    # HTransfx(thx)
    # rospy.sleep(0.1)

fig, axs = plt.subplots(3)

# axs[0].plot(range(0,len(x_tot)), x_tot,label="x")
# axs[1].plot(range(0,len(x_tot)), y_tot,label="y")
# axs[2].plot(range(0,len(x_tot)), z_tot,label="z")

axs[0].plot(range(0,len(x_s_vec)), x_s_vec,label="x_s_anal")
axs[1].plot(range(0,len(x_s_vec)), y_s_vec,label="y_s_anal")
axs[2].plot(range(0,len(x_s_vec)), z_s_vec,label="z_s_anal")
axs[0].plot(range(0,len(x_s_vec)), x_c_vec,label="x_c")
axs[1].plot(range(0,len(x_s_vec)), y_c_vec,label="y_c")
axs[2].plot(range(0,len(x_s_vec)), z_c_vec,label="z_c")
axs[0].legend()
axs[1].legend()
axs[2].legend()
# axs[3].plot(range(0,len(x_s_vec)), np.divide(x_s_vec,x_c_vec),label="x")
# axs[3].plot(range(0,len(x_s_vec)), np.divide(y_s_vec,y_c_vec),label="y")
# axs[3].plot(range(0,len(x_s_vec)), np.divide(z_s_vec,z_c_vec),label="z")
plt.show()



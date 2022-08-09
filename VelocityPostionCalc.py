#!/usr/bin/env python3
import time 
import rospy 
from tf.transformations import *
from sensor_msgs.msg import Imu
import math 
import rospy
from std_msgs.msg import Float32

# Decalre the pervious time 

tPervious = 0

# Declare the Accelration in X and Y direction 

accelX = 0 
accelY = 0 

# Declare the velocities in the x direction and y direction 

lastVelocity_x  = 0
lastlVelocity_y = 0 

# Declare the position in x and y directions 

lastPosX = 0
lastPosY = 0  

# Declare the magnitude velocity 
magVelocity = 0 
 


# get the data from the sensors 
def callback(data):
    # make the variables global 

    global accelX , accelY


    accelX = data.linear_acceleration.x
    accelY = data.linear_acceleration.y
    
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def main():
    global tPervious , magVelocity
    global lastVelocity_x , lastlVelocity_y
    global lastPosX, lastPosY

    rospy.init_node('Waypoints', anonymous=True)
    
    magnitudeV_pub = rospy.Publisher('magnitudeV', Float32, queue_size=10)

    posX_pub = rospy.Publisher('posX', Float32, queue_size=10)
    posY_pub = rospy.Publisher('posY', Float32, queue_size=10)

    rospy.Subscriber("wit/imu", Imu, callback)

    rate = rospy.Rate(10) # 10hz
    
    
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # Measaure the current time 
        
        tCurrent = time.process_time() 
        
        #  Optain the delta of time 
        delta_time = tCurrent - tPervious
        rospy.loginfo(str(delta_time))

        currentVelocityX = lastVelocity_x + accelX * delta_time 
        currentPosX = lastPosX + currentVelocityX *delta_time 

        currentVelocityY = lastlVelocity_y + accelY * delta_time 
        currentPosY = lastPosY + currentVelocityY *delta_time

        magVelocity = math.sqrt(((currentVelocityX)**2) + ((currentVelocityY)**2) )

        lastVelocity_x = currentVelocityX  
        lastPosX = currentPosX 

        lastlVelocity_y = currentVelocityY
        lastPosY = currentPosY 

        tPervious = tCurrent
        
        magnitudeV_pub.publish(magVelocity)

        posX_pub.publish(currentPosX)
        posY_pub.publish(currentPosY)

        rate.sleep()
        # rospy.spin()


# Read the accel in the x axis 
 

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
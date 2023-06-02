#==========================================
# Title:  UR5 robot arm force control in Z
# Author: H. Oktay Erkol
# Date:   31 Jul 2022
#==========================================

import rtde_control
import rtde_receive
import numpy
import time
import os

#==========================================
# applyForce function
#==========================================
def applyForce(rtde_c, rtde_r, force, translationZ):
    
    print ('taking its position, until contact in z')
    # move in z until contact
    speed = [0, 0, -0.04, 0, 0, 0]
    rtde_c.moveUntilContact(speed)
    print ('contacted a surface, force will be applied in 3 seconds!')
    time.sleep(3)
    
    # force control parameters
    selection_vector = [0, 0, 1, 0, 0, 0]   # axis-Z is selected
    wrench_down = [0, 0, -1*force, 0, 0, 0]     # downward force 
    limits = [2, 2, 1.5, 1, 1, 1]           # movement limits of the robot
    task_frame = [0, 0, 0, 0, 0, 0]
    force_type = 2
    
    # save home position
    homePos=rtde_r.getActualTCPPose()
    # calculate target pos adding z translation
    targetPos=numpy.array(homePos)
    targetPos[2]=targetPos[2]-translationZ

    currentTime=0;
    # apply force and check if reached to the target pos
    print ("force mode start position: ", homePos[2])
    while(1):
        currentPosition=rtde_r.getActualTCPPose()
        rtde_c.forceMode(task_frame, selection_vector, wrench_down, force_type, limits)
        if currentPosition[2]<=targetPos[2]:
            rtde_c.forceModeStop()
            print ("force mode stop position: ", currentPosition[2])
            break
        if currentTime>=processTime:
            rtde_c.forceModeStop()
            print ("force mode stop position:  ", currentPosition[2], 'time is over!')
            break
        else:
            time.sleep(0.01)
            currentTime+=0.01
            
            
#==========================================
# main program
#==========================================

fileName='ur5_data.csv' # file name for the recorded data
force=20                # applied force in Z direction
processTime=10          # force applying  period as second
translationZ=0.1        # in meters, max Z position

# initial parameters        
ip="172.22.22.2"    # robot IP address
frequency=500       # data acquisition frequency
vel = 0.1           # TCP velocity
accel = 0.2         # TCP acceleration

# create control and communication objects
rtde_c = rtde_control.RTDEControlInterface(ip)
rtde_r = rtde_receive.RTDEReceiveInterface(ip,frequency)
rtde_c.zeroFtSensor()

# create file path and name
outputFile=os.getcwd()+'/'+fileName


# take the current position
homePos=rtde_r.getActualTCPPose()


# start recording, apply force and stop recording
rtde_r.startFileRecording(outputFile)
applyForce(rtde_c, rtde_r, force, translationZ)
rtde_r.stopFileRecording()       

# wait n second
time.sleep(2)

# move the arm to the first position
print ("Robot is going to home position", homePos[2])
rtde_c.moveL(homePos, vel, accel)


# end of the program
rtde_c.stopScript()
print('finish')       

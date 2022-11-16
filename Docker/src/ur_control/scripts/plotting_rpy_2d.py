#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import LinkStates

import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# https://lilaboc.work/archives/23793422.html
class PlottingRPY:
    def __init__(self):
        ## subscriber
        self.sub_vector = rospy.Subscriber("/gazebo/link_states", LinkStates, self.callbackVector)
        ## msg
        self.rpy = LinkStates()
        ## list
        self.list_x = []
        self.list_y = []
        self.list_z = []
        ## line
        self.line = None
        ## time
        # self.start_time = time.time()
        ## parameter
        self.interval = 0.1
        self.shown_size = 100

        ## initialization
        self.initializePlot()
        ## loop
        self.mainLoop()

    def callbackVector(self, msg):
        self.rpy = msg

    def initializePlot(self):
        plt.figure(figsize = [12,4,4])
        plt.ion()	#interactive mode on
        ## empty list
        self.list_x = [0 for i in range(self.shown_size)]
        self.list_y = [0 for i in range(self.shown_size)]
        self.list_z = [0 for i in range(self.shown_size)]

        ## xy
        plt.subplot(3, 1, 1)
        plt.xlabel("x")
        plt.ylabel("y")
        plt.xlim(-1, 1)
        plt.ylim(-1, 1)
        self.line_xy, = plt.plot(self.list_x, self.list_y)
        ## xz
        plt.subplot(3, 1, 2)
        plt.xlabel("x")
        plt.ylabel("z")
        plt.xlim(-1, 1)
        plt.ylim(0, 2)
        self.line_xz, = plt.plot(self.list_x, self.list_z)
        ## yz
        plt.subplot(3, 1, 3)
        plt.xlabel("y")
        plt.ylabel("z")
        plt.xlim(-1, 1)
        plt.ylim(0, 2)
        self.line_yz, = plt.plot(self.list_y, self.list_z)


    def mainLoop(self):
        while not rospy.is_shutdown():
            self.updatePlot()
            self.drawPlot()

    def updatePlot(self):
        ## append
        if self.rpy.pose != []:
            #print(self.rpy)
            self.list_x.append(self.rpy.pose[7].position.x)
            self.list_y.append(self.rpy.pose[7].position.y)
            self.list_z.append(self.rpy.pose[7].position.z)
            ## pop
            self.list_x.pop(0)
            self.list_y.pop(0)
            self.list_z.pop(0)
            ## xy
            self.line_xy.set_xdata(self.list_x)
            self.line_xy.set_ydata(self.list_y)
            ## xz
            self.line_xz.set_xdata(self.list_x)
            self.line_xz.set_ydata(self.list_z)
            ## yz
            self.line_yz.set_xdata(self.list_y)
            self.line_yz.set_ydata(self.list_z)



    def drawPlot(self):
        ## draw
        plt.draw()
        plt.pause(self.interval)

def main():
    rospy.init_node('plotting_rpy', anonymous=True)
    plotting_rpy = PlottingRPY()
    rospy.spin()

if __name__ == '__main__':
    main()
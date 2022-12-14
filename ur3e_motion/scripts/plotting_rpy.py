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
        self.shown_size = 1000

        ## initialization
        self.initializePlot()
        ## loop
        self.mainLoop()

    def callbackVector(self, msg):
        self.rpy = msg

    def initializePlot(self):
        fig = plt.figure()
        self.ax = fig.add_subplot(111, projection='3d')
        plt.ion()	#interactive mode on
        ## empty list
        self.list_x = [0 for i in range(self.shown_size)]
        self.list_y = [0 for i in range(self.shown_size)]
        self.list_z = [0 for i in range(self.shown_size)]

        self.ax.set_title("End-effector position")
        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")
        self.ax.set_zlabel("z")

        self.ax.grid(True)
        self.line, = self.ax.plot(self.list_x, self.list_y, self.list_z)

    def mainLoop(self):
        while not rospy.is_shutdown():
            self.updatePlot()
            self.drawPlot()

    def updatePlot(self):
        ## append
        if self.rpy.pose != []:
            print(self.rpy)
            self.list_x.append(self.rpy.pose[7].position.x)
            self.list_y.append(self.rpy.pose[7].position.y)
            self.list_z.append(self.rpy.pose[7].position.z)
            ## pop
            self.list_x.pop(0)
            self.list_y.pop(0)
            self.list_z.pop(0)
            ## roll
            self.line.set_xdata(self.list_x)
            self.line.set_ydata(self.list_y)
            self.ax.set_xlim(-2, 2)
            self.ax.set_ylim(-2, 2)
            self.ax.set_zlim(-1, 3)

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
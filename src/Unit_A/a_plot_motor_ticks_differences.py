#!/usr/bin/env python

from pylab import *
from lego_robot import LegoLogfile

if __name__ =='__main__':

	logfile = LegoLogfile()
	logfile.read('robot4_motors.txt')


	for i in range(25):
		print logfile.motor_ticks[i]

	plot(logfile.motor_ticks)
	show()
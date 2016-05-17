#!/usr/bin/env python

# Multiply a distribution by another distribution.
# 06_c_multiply_distribution
# Claus Brenner, 26 NOV 2012
from pylab import plot, show, ylim
from distribution import *
import numpy as np 

def multiply(a, b):
    """Multiply two distributions and return the resulting distribution."""

    # --->>> Put your code here.
    a_values = a.values 
    b_values = b.values
    final_values = []
    final_list =[]

    print b.start() - a.start()
    print a.stop() -a.start()


    if a.start() >= b.start ():
    	for i in xrange(min(len(a_values),len(b_values))):
    		final_values.append(a_values[i]*b_values[(a.start()-b.start())+i])
    	constant = float(sum(final_values))

    	for i in range(len(final_values)):
    		final_list.append(float(final_values[i]/constant))

    if b.start() > a.start():
    	for i in xrange(min(len(a_values),len(b_values))):
    		if (b.start()-a.start())+i < a.stop():
    			final_values.append(b_values[i]*a_values[(b.start()-a.start())+i])
    		else:
    			pass
    	constant = float(sum(final_values))

    	for i in range(len(final_values)):
    		final_list.append(float(final_values[i]/constant))

    
    return Distribution(a.offset,final_list)  # Modify this to return your result.


if __name__ == '__main__':
    arena = (0,1000)

    # Here is our assumed position. Plotted in blue.
    position_value = 400
    position_error = 100
    position = Distribution.triangle(position_value, position_error)
    plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
         color='b', linestyle='steps')

    # Here is our measurement. Plotted in green.
    # That is what we read from the instrument.
    measured_value = 500
    measurement_error = 200
    measurement = Distribution.triangle(measured_value, measurement_error)
    plot(measurement.plotlists(*arena)[0], measurement.plotlists(*arena)[1],
         color='g', linestyle='steps')

    # Now, we integrate our sensor measurement. Result is plotted in red.
    position_after_measurement = multiply(position, measurement)
    plot(position_after_measurement.plotlists(*arena)[0],
         position_after_measurement.plotlists(*arena)[1],
         color='r', linestyle='steps')
    #ylim(0.0, 1.1)
    show()

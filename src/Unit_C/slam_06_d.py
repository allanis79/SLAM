#!/usr/bin/env python


# Histogram implementation of a bayes filter - combines
# convolution and multiplication of distributions, for the
# movement and measurement steps.
# 06_d_histogram_filter
# Claus Brenner, 28 NOV 2012
from pylab import plot, show, ylim
from distribution import *
import numpy as np 


def move(distribution, delta):
    """Returns a Distribution that has been moved (x-axis) by the amount of
       delta."""
    return Distribution(distribution.offset + delta, distribution.values)



# --->>> Copy your convolve(a, b) and multiply(a, b) functions here.

def convolve(a, b):
    """Convolve distribution a and b and return the resulting new distribution."""

    # --->>> Put your code here.

    val_a = a.values
    val_b = b.values
    final_list = np.convolve(val_a,val_b)
    
    return Distribution(a.offset + b.offset, final_list)

def multiply(a, b):
    """Multiply two distributions and return the resulting distribution."""

    # --->>> Put your code here.
    a_values = a.values 
    b_values = b.values
    final_values = []
    final_list=[]
    '''
    print 'len a',len(a_values)
    print 'len b',len(b_values)
    print 'a start',a.start()
    print 'b start', b.start()
    print 'a stop', a.stop()
    print 'b stop', b.stop()
    print '------------------------'
    '''
    
    if a.start() == b.start() and a.stop() == b.stop():
    	print 'sai'
    
    	final_values = np.multiply(a_values,b_values)
    	constant = float(sum(final_values))
    	final_list =[float(i/constant) for i in final_values]
    	
    	
    if a.start() < b.start() and a.stop() > b.stop():
    	print 'krishna'
    
    	temp = b.start() - a.start()
    	b_values = [0] * temp + b_values
    	for i in xrange(0,temp):
    		b_values.append(0)

    	
    	final_values = np.multiply(a_values,b_values)
    	constant = float(sum(final_values))
    	final_list =[float(i/constant) for i in final_values]
    

    if a.start() < b.start() and a.stop() == b.stop():
    	print
    
    	temp = b.start() - a.start()
    	b_values = [0] * temp + b_values
  
    	final_values = np.multiply(a_values,b_values)
    	constant = float(sum(final_values))
    	final_list =[float(i/constant) for i in final_values]
    	
    
    return Distribution(a.offset,final_list)  # Modify this to return your result.





if __name__ == '__main__':
    arena = (0,220)

    # Start position. Exactly known - a unit pulse.
    start_position = 10
    position = Distribution.unit_pulse(start_position)
    plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
         linestyle='steps')

    # Movement data.
    controls  =    [ 20 ] * 10

    # Measurement data. Assume (for now) that the measurement data
    # is correct. - This code just builds a cumulative list of the controls,
    # plus the start position.
    p = start_position
    measurements = []
    for c in controls:
        p += c
        measurements.append(p)

    # This is the filter loop.
    for i in xrange(len(controls)):
        # Move, by convolution. Also termed "prediction".
        control = Distribution.triangle(controls[i], 10)
        position = convolve(position, control)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             color='b', linestyle='steps')

        # Measure, by multiplication. Also termed "correction".
        measurement = Distribution.triangle(measurements[i], 10)
        position = multiply(position, measurement)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             color='r', linestyle='steps')

    show()

#!/usr/bin/env python

# This adds the derivative of g, this time with respect to the control
# (left and right motor movement).
#
# slam_07_c_control_derivative
# Claus Brenner, 11.12.2012
from lego_robot import *
from math import sin, cos, pi
from numpy import *

class ExtendedKalmanFilter:

    @staticmethod
    def g(state, control, w):
        x, y, theta = state
        l, r = control
        if r != l:
            alpha = (r - l) / w
            rad = l/alpha
            g1 = x + (rad + w/2.)*(sin(theta+alpha) - sin(theta))
            g2 = y + (rad + w/2.)*(-cos(theta+alpha) + cos(theta))
            g3 = (theta + alpha + pi) % (2*pi) - pi
        else:
            g1 = x + l * cos(theta)
            g2 = y + l * sin(theta)
            g3 = theta

        return array([g1, g2, g3])

    @staticmethod
    def dg_dcontrol(state, control, w):
        theta = state[2]
        l, r = tuple(control)
        if r != l:

            # --->>> Put your code here.
            # This is for the case l != r.
            # Note g has 3 components and control has 2, so the result
            # will be a 3x2 (rows x columns) matrix.
            alpha = float((r-l)/w)
            num_1 = float(w*r)
            den_1 = float((r-l)*(r-l))
            constant_1 = float(num_1/den_1)
            num_2 = float(r+l)
            den_2 = float(2*(r-l))
            constant_2 = float(num_2/den_2)

            g1_l = float(constant_1 *(sin(theta+alpha) - sin(theta)) - constant_2*(cos(theta+alpha)))
            g2_l = float(constant_1 *(cos(theta) - cos(theta+alpha)) - constant_2 *(sin(theta+alpha)))
            g3_l = -float(1/w)


            num_3 = float(w*l)
            den_3 = den_1
            constant_3 = float(num_3/den_3)
            constant_4 = constant_2 

            g1_r = float(constant_4 * cos(theta+alpha) - constant_3 * (sin(theta+alpha) - sin(theta)))
            g2_r = float(constant_4 * sin(theta+alpha) - constant_3 *(cos(theta) - cos(theta+alpha)))

            g3_r = float(1/w)

            m = array([[g1_l,g1_r],[g2_l,g2_r],[g3_l,g3_r]])

            
            
            
        else:

            # --->>> Put your code here.
            # This is for the special case l == r.
            

            constant = float(l/w)

            g1_l = float((cos(theta)+ (constant*sin(theta))))/2
            g2_l = float((sin(theta) - (constant * cos(theta))))/2
            g3_l = -float(1/w)


            g1_r = float(( cos(theta) - (constant * sin(theta))))/2
            g2_r = float((sin(theta) + (constant * cos(theta))))/2
            g3_r = float(1/w)

            m = array([[g1_l,g1_r],[g2_l,g2_r],[g3_l,g3_r]])

           
         
            
        return m


if __name__ == '__main__':
    # If the partial derivative with respect to l and r (the control)
    # are correct, then the numerical derivative and the analytical
    # derivative should be the same.

    # Set some variables. Try other variables as well.
    # In particular, you should check cases with l == r and l != r.
    x = 10.0
    y = 20.0
    theta = 35. / 180. * pi
    state = array([x, y, theta])
    l = 50.0
    r = 50.0
    control = array([l, r])
    w = 150.0

    # Compute derivative numerically.
    print "Numeric differentiation dl, dr"
    delta = 1e-7
    control_l = array([l + delta, r])
    control_r = array([l, r + delta])
    dg_dl = (ExtendedKalmanFilter.g(state, control_l, w) -\
             ExtendedKalmanFilter.g(state, control, w)) / delta
    dg_dr = (ExtendedKalmanFilter.g(state, control_r, w) -\
             ExtendedKalmanFilter.g(state, control, w)) / delta
    dg_dcontrol_numeric = column_stack([dg_dl, dg_dr])
    print dg_dcontrol_numeric

    # Use the above code to compute the derivative analytically.
    print "Analytic differentiation dl, dr:"
    dg_dcontrol_analytic = ExtendedKalmanFilter.dg_dcontrol(state, control, w)
    print dg_dcontrol_analytic

    # The difference should be close to zero (depending on the setting of
    # delta, above).
    print "Difference:"
    print dg_dcontrol_numeric - dg_dcontrol_analytic
    print "Seems correct:", allclose(dg_dcontrol_numeric, dg_dcontrol_analytic)

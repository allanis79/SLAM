#!/usr/bin/env python

# The full Kalman filter, consisting of prediction and correction step.
#
# slam_07_f_kalman_filter
# Claus Brenner, 12.12.2012
from lego_robot import *
from math import sin, cos, pi, atan2, sqrt
from numpy import *
from slam_d_library import get_observations, write_cylinders


class ExtendedKalmanFilter:
    def __init__(self, state, covariance,
                 robot_width, scanner_displacement,
                 control_motion_factor, control_turn_factor,
                 measurement_distance_stddev, measurement_angle_stddev):
        # The state. This is the core data of the Kalman filter.
        self.state = state
        self.covariance = covariance

        # Some constants.
        self.robot_width = robot_width
        self.scanner_displacement = scanner_displacement
        self.control_motion_factor = control_motion_factor
        self.control_turn_factor = control_turn_factor
        self.measurement_distance_stddev = measurement_distance_stddev
        self.measurement_angle_stddev = measurement_angle_stddev

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
    def dg_dstate(state, control, w):
    	theta = state[2]
        l, r = control
        if r != l:

            # --->>> Put your code here.
            # This is for the case r != l.
            # g has 3 components and the state has 3 components, so the
            # derivative of g with respect to all state variables is a
            # 3x3 matrix. To construct such a matrix in Python/Numpy,
            # use: m = array([[1, 2, 3], [4, 5, 6], [7, 8, 9]]),
            # where 1, 2, 3 are the values of the first row of the matrix.
            # Don't forget to return this matrix.
            alpha = float((r-l)/w)
            R = float(l/alpha)
            val_1 = float((R+(w/2))*(cos(theta + alpha) - cos(theta)))
            val_2 = float((R+(w/2))*(sin(theta + alpha) - sin(theta)))
            m = array([[1, 0, val_1], [0, 1, val_2], [0, 0, 1]])  # Replace this.

        else:

            # --->>> Put your code here.
            # This is for the special case r == l.
            val_1 = -float(l/sin(theta))
            val_2 = float(l/cos(theta))
            m = array([[1, 0, val_1], [0, 1, val_2], [0, 0, 1]])  # Replace this.

        return m


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

            g1_l = float(0.5*(cos(theta)+ (constant*sin(theta))))
            g2_l = float(0.5*(sin(theta) - (constant * cos(theta))))
            g3_l = -float(1/w)


            g1_r = float(0.5*( cos(theta) - (constant * sin(theta))))
            g2_r = float(0.5*(sin(theta) + (constant * cos(theta))))
            g3_r = float(1/w)

            m = array([[g1_l,g1_r],[g2_l,g2_r],[g3_l,g3_r]])

           
         
            
        return m

    @staticmethod
    def get_error_ellipse(covariance):
        """Return the position covariance (which is the upper 2x2 submatrix)
           as a triple: (main_axis_angle, stddev_1, stddev_2), where
           main_axis_angle is the angle (pointing direction) of the main axis,
           along which the standard deviation is stddev_1, and stddev_2 is the
           standard deviation along the other (orthogonal) axis."""
        eigenvals, eigenvects = linalg.eig(covariance[0:2,0:2])
        angle = atan2(eigenvects[1,0], eigenvects[0,0])
        return (angle, sqrt(eigenvals[0]), sqrt(eigenvals[1]))        

    def predict(self, control):
    	"""The prediction step of the Kalman filter."""
        # covariance' = G * covariance * GT + R
        # where R = V * (covariance in control space) * VT.
        # Covariance in control space depends on move distance.
        left, right = control

        # --->>> Put your code to compute the new self.covariance here.
        # First, construct the control_covariance, which is a diagonal matrix.
        # In Python/Numpy, you may use diag([a, b]) to get
        # [[ a, 0 ],
        #  [ 0, b ]].
        # Then, compute G using dg_dstate and V using dg_dcontrol.
        # Then, compute the new self.covariance.
        # Note that the transpose of a Numpy array G is expressed as G.T,
        # and the matrix product of A and B is written as dot(A, B).
        # Writing A*B instead will give you the element-wise product, which
        # is not intended here.

        # state' = g(state, control)

        # --->>> Put your code to compute the new self.state here.

        a_l = float((self.control_motion_factor * left) * (self.control_motion_factor * left) + (self.control_turn_factor*(left-right)) * (self.control_turn_factor*(left-right)))
        a_r = float((self.control_motion_factor * right) * (self.control_motion_factor * right) + (self.control_turn_factor*(left-right)) * (self.control_turn_factor*(left-right)))

        control_covariance = diag([a_l,a_r])

        G = self.dg_dstate(self.state,control,self.robot_width)
        V = self.dg_dcontrol(self.state,control,self.robot_width)

        R_1 = dot(V,control_covariance)
        R = dot(R_1,V.T)
        g_1 = dot(G,self.covariance)
        G_1 = dot(g_1,G.T)

        self.covariance = G_1 + R 

        self.state = self.g(self.state,control,self.robot_width)


    @staticmethod
    def h(state, landmark, scanner_displacement):
        """Takes a (x, y, theta) state and a (x, y) landmark, and returns the
           measurement (range, bearing)."""
        dx = landmark[0] - (state[0] + scanner_displacement * cos(state[2]))
        dy = landmark[1] - (state[1] + scanner_displacement * sin(state[2]))
        r = sqrt(dx * dx + dy * dy)
        alpha = (atan2(dy, dx) - state[2] + pi) % (2*pi) - pi

        return array([r, alpha])

    @staticmethod
    def dh_dstate(state, landmark, scanner_displacement):

    	   # --->>> Insert your code here.
        # Note that:
        # x y theta is state[0] state[1] state[2]
        # x_m y_m is landmark[0] landmark[1]
        # The Jacobian of h is a 2x3 matrix.

        theta = state[2]

        delta_x = landmark[0] - (state[0] + scanner_displacement * cos(state[2]))
        delta_y = landmark[1] - (state[1] + scanner_displacement * sin(state[2]))
        q = sqrt(delta_x* delta_x + delta_y * delta_y)

        r_x = -(delta_x/sqrt(q))
        r_y = -(delta_y/sqrt(q))

        constant = (scanner_displacement/sqrt(q))

        a = (delta_x*sin(theta))
        b = (delta_y*cos(theta))

        r_theta = (constant*(a-b))

        alpha_x = (delta_y/q)
        alpha_y = -(delta_x/q)
        constant1 = (scanner_displacement/q)
        c = delta_x * cos(theta)
        d = delta_y*sin(theta)
        E = c+d

        alpha_theta = - (constant1 * E) - 1
        #alpha_theta = alpha_theta - 1
        return array([[r_x,r_y,r_theta], [alpha_x,alpha_y,alpha_theta]]) # Replace this.

    def correct(self, measurement, landmark):
        """The correction step of the Kalman filter."""

        # --->>> Put your new code here.
        #
        # You will have to compute:
        # H, using dh_dstate(...).
        # Q, a diagonal matrix, from self.measurement_distance_stddev and
        #  self.measurement_angle_stddev (remember: Q contains variances).
        # K, from self.covariance, H, and Q.
        #  Use linalg.inv(...) to compute the inverse of a matrix.
        # The innovation: it is easy to make an error here, because the
        #  predicted measurement and the actual measurement of theta may have
        #  an offset of +/- 2 pi. So here is a suggestion:
        #   innovation = array(measurement) -\
        #                self.h(self.state, landmark, self.scanner_displacement)
        #   innovation[1] = (innovation[1] + pi) % (2*pi) - pi
        # Then, you'll have to compute the new self.state.
        # And finally, compute the new self.covariance. Use eye(3) to get a 3x3
        #  identity matrix.
        #
        # Hints:
        # dot(A, B) is the 'normal' matrix product (do not use: A*B).
        # A.T is the transposed of a matrix A (A itself is not modified).
        # linalg.inv(A) returns the inverse of A (A itself is not modified).
        # eye(3) returns a 3x3 identity matrix.

        H = self.dh_dstate(self.state, landmark, self.scanner_displacement)
        a = self.measurement_distance_stddev * self.measurement_distance_stddev
        b = self.measurement_angle_stddev * self.measurement_angle_stddev
        Q = diag([a,b])
        constant_5 =  dot(self.covariance,H.T)
        constant_6 = dot(H,constant_5)
        print shape(constant_6)
        constant_7 = linalg.inv(constant_6 + Q)

        K = dot(constant_5,constant_7)
        innovation = array(measurement) - self.h(self.state, landmark, self.scanner_displacement)
        innovation[1] = (innovation[1] + pi) % (2*pi) - pi

        self.state = self.state + dot(K,innovation)

    
        constant_8 = dot(K,H)

        self.covariance = dot((eye(3) - constant_8),self.covariance)




if __name__ == '__main__':
    # Robot constants.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 155.0

    # Cylinder extraction and matching constants.
    minimum_valid_distance = 20.0
    depth_jump = 100.0
    cylinder_offset = 90.0
    max_cylinder_distance = 300.0

    # Filter constants.
    control_motion_factor = 0.35  # Error in motor control.
    control_turn_factor = 0.6  # Additional error due to slip when turning.
    measurement_distance_stddev = 200.0  # Distance measurement error of cylinders.
    measurement_angle_stddev = 15.0 / 180.0 * pi  # Angle measurement error.

    # Measured start position.
    initial_state = array([1850.0, 1897.0, 213.0 / 180.0 * pi])
    # Covariance at start position.
    initial_covariance = diag([100.0**2, 100.0**2, (10.0 / 180.0 * pi) ** 2])
    # Setup filter.
    kf = ExtendedKalmanFilter(initial_state, initial_covariance,
                              robot_width, scanner_displacement,
                              control_motion_factor, control_turn_factor,
                              measurement_distance_stddev,
                              measurement_angle_stddev)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    logfile.read("robot4_scan.txt")
    logfile.read("robot_arena_landmarks.txt")
    reference_cylinders = [l[1:3] for l in logfile.landmarks]

    # Loop over all motor tick records and all measurements and generate
    # filtered positions and covariances.
    # This is the Kalman filter loop, with prediction and correction.
    states = []
    covariances = []
    matched_ref_cylinders = []
    for i in xrange(len(logfile.motor_ticks)):
        # Prediction.
        control = array(logfile.motor_ticks[i]) * ticks_to_mm
        kf.predict(control)

        # Correction.
        observations = get_observations(
            logfile.scan_data[i],
            depth_jump, minimum_valid_distance, cylinder_offset,
            kf.state, scanner_displacement,
            reference_cylinders, max_cylinder_distance)
        for j in xrange(len(observations)):
            kf.correct(*observations[j])

        # Log state, covariance, and matched cylinders for later output.
        states.append(kf.state)
        covariances.append(kf.covariance)
        matched_ref_cylinders.append([m[1] for m in observations])

    # Write all states, all state covariances, and matched cylinders to file.
    f = open("kalman_prediction_and_correction.txt", "w")
    for i in xrange(len(states)):
        # Output the center of the scanner, not the center of the robot.
        print >> f, "F %f %f %f" % \
            tuple(states[i] + [scanner_displacement * cos(states[i][2]),
                               scanner_displacement * sin(states[i][2]),
                               0.0])
        # Convert covariance matrix to angle stddev1 stddev2 stddev-heading form
        e = ExtendedKalmanFilter.get_error_ellipse(covariances[i])
        print >> f, "E %f %f %f %f" % (e + (sqrt(covariances[i][2,2]),))
        # Also, write matched cylinders.
        write_cylinders(f, "W C", matched_ref_cylinders[i])        

    f.close()

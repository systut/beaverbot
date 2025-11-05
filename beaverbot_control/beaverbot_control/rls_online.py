import numpy as np
import math
import rospy


#class rls online

class RLSOnline:
    def __init__(self, s0, P0, R):
        """
        Initialize the Recursive Least Squares (RLS) algorithm.
        
        Parameters:
        s0 : np.ndarray
            Initial state estimate.
        P0 : np.ndarray
            Initial error covariance matrix.
        R : np.ndarray
            Measurement noise covariance matrix.
        """
        self.s0 = s0
        self.P0 = P0
        self.R = R
        #the list where the estimates of the slip will be stored
        self.estimates = []
        self.estimates.append(s0)
        # the list where the estimation error covariance matrices will be stored Pk
        self.estimationErrorCovarianceMatrices = []
        self.estimationErrorCovarianceMatrices.append(P0)
        #the list where the Kalman gain matrices will be stored Kk
        self.gainMatrices = []
        # the list where the estimation errors will be stored ek
        self.errors = []

        self.yaw_diff = []
        self.angular_vel_z = []

        # the distance between the wheels
        self.L = 0.5

        # this variable is used to track the current time step k of the estimator 
        # after every time step arrives, this variables increases for one 
        # in this way, we can track the number of variblaes
        self.previousTimeStep=0
    
    #writing method to estimate slip from simulation data
    def predict_sim(self, yaw, yaw_previous, ground_angular_velocity_z, delta_t):
            """
            First calculating the theta difference and the angular velocity 
            """
            yaw_diff= np.array([yaw - yaw_previous])
            yaw_diff = (yaw_diff + math.pi) % (2 * math.pi) - math.pi  # unwrap to [-pi, pi]
            C = np.array([delta_t * ground_angular_velocity_z ])
            #Calculating L matrix and its inverse
            L_matrix = self.R + np.matmul(C, np.matmul(self.estimationErrorCovarianceMatrices[self.previousTimeStep], C.T))
            L_matrix_inverse = np.linalg.inv(L_matrix)            

            #Calculating the Kalman gain matrix
            gain_matrix = np.matmul(self.estimationErrorCovarianceMatrices[self.previousTimeStep], np.matmul(C.T, L_matrix_inverse))

            #Calculating the estimation error(correction term (yk -Cxk))
            error = (C-yaw_diff) - np.matmul(C, self.estimates[self.previousTimeStep])
            rospy.loginfo(f"Error in RLS: {error}")
            #Calculating the new estimate
            estimate = self.estimates[self.previousTimeStep] + np.matmul(gain_matrix, error)

            #Calculating the new estimation error covariance matrix
            ImKc = np.eye(np.size(self.s0), np.size(self.s0)) - np.matmul(gain_matrix, C)
            estimationErrorCovarianceMatrix = np.matmul(ImKc, self.estimationErrorCovarianceMatrices[self.previousTimeStep])

            #Storing the results
            self.estimates.append(estimate)
            self.estimationErrorCovarianceMatrices.append(estimationErrorCovarianceMatrix)
            self.gainMatrices.append(gain_matrix)
            self.errors.append(error)
            self.yaw_diff.append(yaw_diff)
            self.angular_vel_z.append(ground_angular_velocity_z)

            # increase the time step
            self.previousTimeStep = self.previousTimeStep + 1
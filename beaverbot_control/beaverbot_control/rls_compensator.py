import rospy

import numpy as np

from beaverbot_control.rls_online import RLSOnline

class RLSCompensator:
    def __init__(self, trajectory):
        """
        Initialize the Recursive Least Squares (RLS) algorithm.
        
        Parameters:
        trajectory : namedtuple: The reference trajectory
        """
        self.trajectory = trajectory
        s0 = np.array([[0.0]])       # initial slip estimate
        P0 = np.eye(1) * 100.0      # large initial covariance
        R = np.eye(1) * 0.01         # measurement noise
        self.rls = RLSOnline(s0, P0, R)
        self.yaw_previous = None
        self.first_step = True

    
    # writing method to implement online RLS and compensate the velocities from reference file
    def execute(self, state, input, index, delta_t):
        """! Execute the controller
        @param state<list>: The state of the vehicle
        @param input<list>: The input of the vehicle
        @param previous_index<int>: The previous index
        @param delta_t<float>: The time step
        @return<tuple>: The status and control
        """
        status = True
        if index >= len(self.trajectory.u[0, :]) - 1:
            return False, [0, 0]
        ground_angular_velocity_z = self.trajectory.u[1, index]# this is a angular vel from trajectory.u which is a 2d vector [v,w]
        unwrapped_yaw = np.unwrap([state[2]])[0]
        if not self.first_step:
            self.rls.predict_sim(
                yaw=unwrapped_yaw, 
                yaw_previous=self.yaw_previous, 
                ground_angular_velocity_z=ground_angular_velocity_z, 
                delta_t=delta_t)
        rospy.loginfo(f"Yaw: {unwrapped_yaw}, Yaw previous: {self.yaw_previous}, Ground angular velocity z: {ground_angular_velocity_z}")
        self.yaw_previous= unwrapped_yaw
        slip = self.rls.estimates[-1][0, 0]
        #slip cannot be negative or greater than 1
        slip = max(0.0, min(slip, 0.2))
        
        # Compensate the velocities  and angular velocities
        v = self.trajectory.u[0, index]/(1-slip)
        w = self.trajectory.u[1, index]/(1-slip)
        rospy.loginfo(f"Difference in velocities due to slip compensation: v: {v - self.trajectory.u[0, index]}, w: {w - self.trajectory.u[1, index]}, slip: {slip}")
        self.first_step = False
        return status, [v, w]
    #from reference trajectory getting the reference velocities
        
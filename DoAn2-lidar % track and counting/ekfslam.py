import numpy as np

class EKFSLAM():
    def __init__(self, max_landmark, x_error, y_error, theta_error, range_error, bearing_error, threshold):
        self.max_landmark = max_landmark
        
        # current known number of landmarks
        self.Nt = 0
        
        # state space initialization
        self.mean = np.zeros((2 * max_landmark + 3, 1))
        self.covariance = np.zeros((2 * max_landmark + 3, 2 * max_landmark + 3))
        for i in range(3, 2 * max_landmark + 3):
            self.covariance[i][i] = 1e7
        
        # noise covariance
        self.R = np.diag([x_error, y_error, theta_error])
        self.Q = np.diag([range_error, bearing_error])

        # threshold to create new landmark (pick from the Chi-square table)
        self.threshold = threshold

    def update_location(self, delta_x, delta_y, delta_theta, observation):
        #############
        # Prediction:

        # predict robot's pose mean
        self.mean[0][0] = self.mean[0][0] + delta_x
        self.mean[1][0] = self.mean[1][0] + delta_y
        self.mean[2][0] = self.mean[2][0] + delta_theta

        # Jacobian of the motion
        Gx = np.array(([[0, 0, -delta_y], [0, 0, delta_x], [0, 0, 0]]))

        # map Gx matrix (3x3) to (2N+3)x(2N+3) dimensional space
        Fx = np.eye(3, 2 * self.max_landmark + 3)
        G = np.eye(2 * self.max_landmark + 3, 2 * self.max_landmark + 3) + Fx.T @ Gx @ Fx

        # predict robot's pose covariance
        self.covariance = G @ self.covariance @ G.T + Fx.T @ self.R @ Fx

        #############
        # Correction:
        
        # dictionary contains the landmarks' range and bearing
        z = {}

        # dictionary contains the landmarks' Jacobian
        H = {}

        # dictionary contains the landmarks' innovation covariance
        psi = {}

        # for all known landmarks do
        for k in range(1, self.Nt + 1):
            landmark_delta_x = self.mean[2 * (k + 1) - 1][0] - self.mean[0][0]
            landmark_delta_y = self.mean[2 * (k + 1)][0] - self.mean[1][0]
            landmark_delta = np.array(([[landmark_delta_x], [landmark_delta_y]]))
            q = (landmark_delta.T @ landmark_delta)[0][0]

            # predict the landmark's range and bearing
            z[k] = np.array(([[np.sqrt(q)], [np.arctan2(landmark_delta_y, landmark_delta_x) - self.mean[2][0]]]))
            
            # Jacobian of the observation
            Hx = 1/q * np.array([[-np.sqrt(q) * landmark_delta_x, -np.sqrt(q) * landmark_delta_y, 0, np.sqrt(q) * landmark_delta_x, np.sqrt(q) * landmark_delta_y],
                                 [landmark_delta_y, -landmark_delta_x, -q, -landmark_delta_y, landmark_delta_x]])
            
            # map Hx matrix (2x5) to a higher dimensional space
            Fx = np.row_stack((np.eye(3, 2 * self.max_landmark + 3), np.column_stack((np.zeros((2, 2 * k + 1)), np.eye(2, 2 * self.max_landmark - 2 * k + 2)))))
            H[k] = Hx @ Fx
            
            # innovation covariance
            psi[k] = H[k] @ self.covariance @ H[k].T + self.Q

        # for all observed landmarks do
        for i in range(1, observation.shape[1] + 1):
            # add new observation as known landmark
            self.mean[2 * (self.Nt + 1 + 1) - 1][0] = observation[0][i - 1]
            self.mean[2 * (self.Nt + 1 + 1)][0] = observation[1][i - 1]

            landmark_delta_x = self.mean[2 * (self.Nt + 1 + 1) - 1][0] - self.mean[0][0]
            landmark_delta_y = self.mean[2 * (self.Nt + 1 + 1)][0] - self.mean[1][0]
            landmark_delta = np.array(([[landmark_delta_x], [landmark_delta_y]]))
            q = (landmark_delta.T @ landmark_delta)[0][0]

            # predict the observation's range and bearing
            zt = np.array(([[np.sqrt(q)], [np.arctan2(landmark_delta_y, landmark_delta_x) - self.mean[2][0]]]))
            z[self.Nt + 1] = zt

            # Jacobian of the observation
            Hx = 1/q * np.array([[-np.sqrt(q) * landmark_delta_x, -np.sqrt(q) * landmark_delta_y, 0, np.sqrt(q) * landmark_delta_x, np.sqrt(q) * landmark_delta_y],
                                 [landmark_delta_y, -landmark_delta_x, -q, -landmark_delta_y, landmark_delta_x]])

            # map Hx matrix (2x5) to a higher dimensional space
            Fx = np.row_stack((np.eye(3, 2 * self.max_landmark + 3), np.column_stack((np.zeros((2, 2 * (self.Nt + 1) + 1)), np.eye(2, 2 * self.max_landmark - 2 * (self.Nt + 1) + 2)))))
            H[self.Nt + 1] = Hx @ Fx

            # innovation covariance
            psi[self.Nt + 1] = H[self.Nt + 1] @ self.covariance @ H[self.Nt + 1].T + self.Q

            # dictionary contains the observation's Mahalanobis distance
            pi = {}

            # for all known landmarks except the newest do
            for k in range(1, self.Nt + 1):
                pi[k] = (zt - z[k]).T @ np.linalg.inv(psi[k]) @ (zt - z[k])

            # set a threshold to create new landmark
            pi[self.Nt + 1] = self.threshold

            # find the landmark closest to the observation
            j = min(pi, key=pi.get)

            # check if new landmark has to be created
            self.Nt = max(self.Nt, j)

            # compute the Kalman gain
            K = self.covariance @ H[j].T @ np.linalg.inv(psi[j])

            # update the system state
            self.mean = self.mean + K @ (zt - z[j])
            self.covariance = (np.eye(2 * self.max_landmark + 3, 2 * self.max_landmark + 3) - K @ H[j]) @ self.covariance

# robot = EKFSLAM(max_landmark=3, x_error=10, y_error=10, theta_error=np.deg2rad(1), range_error=0.01, bearing_error=np.deg2rad(1), threshold=5.991)
# delta_x = 10
# delta_y = 10
# delta_theta = 5
# observation = np.array(([[1000],[1000]]))
# robot.update_location(delta_x, delta_y, delta_theta, observation)
# print(robot.mean)
# print(robot.covariance)
#!/usr/bin/python3
import rospy
import math
import time
from lmpcc_msgs.msg import obstacle_array
from frenet_optimal_planner.srv import ObservedRisk
import numpy as np
from scipy.stats import multivariate_normal
from nav_msgs.msg import Path
import matplotlib.pyplot as plt
import time
from scipy.stats import mvn
from std_msgs.msg import Float64


class risk_assessment:

    def __init__(self):
        self.mean = []
        self.covariance = []
        self.covariance_matrix = []
        self.mean_matrix = []
        self.num_of_obstacles = 0
        rospy.init_node('risk_computation', anonymous=True)

    def risk_calculator(self, req):
        disc_radius = 0.625 / math.sqrt(2)
        risk = 0.0
        risk_per_stage = []
        self.ObstaclePredictionsCallback(req.obstacles)
        self.mean_matrix = np.array(self.mean_matrix)
        self.covariance_matrix = np.array(self.covariance_matrix)
        start = time.time()
        if self.num_of_obstacles > 0:
            # the first dimension indicates number of obstacles, second dimension indicates number of modes, and third
            # dimension indicates horizon length
            for j in range(self.mean_matrix.shape[0]):
                for mode in range(self.mean_matrix.shape[1]):
                    # since the planned trajectory length could be less than the predicted obstacles trajectories, it is
                    # better to iterate over the length of the planned trajectory.
                    for i in range(len(req.pred_traj)):
                        low = [req.pred_traj[i].pose.position.x - disc_radius, req.pred_traj[i].pose.position.y -
                               disc_radius]
                        upp = [req.pred_traj[i].pose.position.x + disc_radius, req.pred_traj[i].pose.position.y +
                               disc_radius]
                        integral_value, _ = mvn.mvnun(low, upp, self.mean_matrix[j][mode][i],
                                                      self.covariance_matrix[j][mode][i])

                        if integral_value > risk:
                            risk = integral_value
        end = time.time()
        #print("the risk calculation function takes: ", (end - start) * 1e03, "ms")
        return risk

    def quaternionToAngle(self, pose):
        ysqr = pose.orientation.y * pose.orientation.y
        t3 = 2 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y)
        t4 = 1 - 2 * (ysqr + pose.orientation.z * pose.orientation.z)
        angle = math.atan2(t3, t4)
        return angle

    def ObstaclePredictionsCallback(self, data):
        binomial = 0
        self.mean = []
        self.covariance = []
        self.covariance_matrix = []
        self.mean_matrix = []
        self.num_of_obstacles = 0
        mean_for_all_obstacles = []
        covariance_for_all_obstacles = []
        for obstacle_prediction in data:

            self.num_of_obstacles += 1
            #print("the number of modes is", len(obstacle_prediction.gaussians))
            # this one should have the same length as no. modes
            for obstacle_prediction_gaussian in obstacle_prediction.gaussians:
                x_poses = []
                y_poses = []
                psi = []
                covariance = []
                # should have the same length as the prediction horizon
                for obstacle_prediction_gaussian_mean_per_stage in obstacle_prediction_gaussian.mean.poses:
                    x_poses.append(obstacle_prediction_gaussian_mean_per_stage.pose.position.x)
                    y_poses.append(obstacle_prediction_gaussian_mean_per_stage.pose.position.y)
                    angle = self.quaternionToAngle(obstacle_prediction_gaussian_mean_per_stage.pose)

                    psi.append(angle)
                # it includes the mean at each stage for this obstacle's mode
                self.mean = np.vstack((x_poses, y_poses)).T
                major_semiaxis = obstacle_prediction_gaussian.major_semiaxis
                minor_semiaxis = obstacle_prediction_gaussian.minor_semiaxis
                major = 0
                minor = 0
                dt = 0.2
                for i in range(len(major_semiaxis)):    # this has the same length as the prediction horizon
                    R = np.array([[math.cos(-psi[i]), math.sin(-psi[i])], [-math.sin(-psi[i]), math.cos(-psi[i])]])
                    major += math.pow(major_semiaxis[i] * dt, 2)
                    minor += math.pow(minor_semiaxis[i] * dt, 2)
                    SVD = np.array([[major, 0], [0, minor]])
                    Sigma = R * SVD * np.transpose(R)
                    cov = np.array([[[Sigma[0, 0], Sigma[0, 1]], [Sigma[1, 0], Sigma[1, 1]]]])
                    covariance.append(cov)
                self.covariance_matrix.append(covariance)
                self.mean_matrix.append(self.mean)

            # we are done with all the modes of the corresponding obstacle
            # add another dimension for the number of obstacles
            if binomial == 0:
                mean_for_all_obstacles.append(self.mean_matrix)
                covariance_for_all_obstacles.append(self.covariance_matrix)
            elif binomial == 1 and len(self.mean_matrix) == 21:
                mean_for_all_obstacles.append(self.mean_matrix)
                covariance_for_all_obstacles.append(self.covariance_matrix)
            else:
                # This is a workaround to avoid obstacles with single mode
                while len(self.mean_matrix) < 21:
                    self.mean_matrix.append(self.mean)
                    self.covariance_matrix.append(covariance)
                mean_for_all_obstacles.append(self.mean_matrix)
                covariance_for_all_obstacles.append(self.covariance_matrix)

            self.mean_matrix = []
            self.covariance_matrix = []

        self.mean_matrix = np.array(mean_for_all_obstacles)
        self.covariance_matrix = np.array(covariance_for_all_obstacles)

    def risk_server(self):
        s = rospy.Service("planned_traj_risk", ObservedRisk, self.risk_calculator)
        rospy.spin()


if __name__ == '__main__':
    obj = risk_assessment()
    try:
        obj.risk_server()
    except rospy.ROSInterruptException:
        pass


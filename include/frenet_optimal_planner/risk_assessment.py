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
        #rospy.Subscriber("/pedestrian_simulator/trajectory_predictions", obstacle_array, self.ObstaclePredictionsCallback)

    def risk_calculator(self, req):
        disc_radius = 0.625
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
                    #for i in range(self.mean_matrix.shape[2]):
                    for i in range(len(req.pred_traj)):
                        #print("the loop index is: ", i)
                        #print("trajectory length is: ", len(req.pred_traj))
                        #print("object prediction length: ", self.mean_matrix.shape[2])
                        low = [req.pred_traj[i].pose.position.x - disc_radius, req.pred_traj[i].pose.position.y - disc_radius]
                        upp = [req.pred_traj[i].pose.position.x + disc_radius, req.pred_traj[i].pose.position.y + disc_radius]
                        # use a built-in function to evaluate the integration of the bivariate gaussian distribution
                        #print(self.num_of_obstacles)
                        #try:
                        #    mean = self.mean_matrix[i][mode][j]
                        #except:
                        #    self.mean_matrix = np.array(self.mean_matrix)
                        #    print("Oops, size of mean_matrix is", self.mean_matrix.shape)
                        #    print(self.mean_matrix.shape[1])
                        #    print(len(self.mean_matrix))
                        #covariance = self.covariance_matrix[i][j]
                        integral_value, _ = mvn.mvnun(low, upp, self.mean_matrix[j][mode][i], self.covariance_matrix[j][mode][i])
                        # This part is a trial to compute the risk in a more accurate way by sampling from the gaussian
                        # distribution and check for the samples inside the robot's circle (these are the samples in
                        # collision). The risk can then be evaluated by dividing these samples over the total drawn samples
                        #distr = multivariate_normal(cov=self.covariance_matrix[i][j], mean=self.mean_matrix[i][j],
                        #                            seed=1000)
                        #data = distr.rvs(size=5000)
                        #no_sample_in_collision = 0

                        #vehicle_pose = [req.pred_traj[i].pose.position.x, req.pred_traj[i].pose.position.y]
                        #diff = data - vehicle_pose
                        #distance_squared = diff[:, 0] ** 2 + diff[:, 1] ** 2
                        #distance = distance_squared ** 0.5

                        #for element in distance:
                        #    if element < 0.325:
                        #        no_sample_in_collision += 1
                        #for k in range(len(data)):
                        # compute the distance between each sampled data and vehicle's position
                        #    diff_x = req.pred_traj[i].pose.position.x - data[k][0]
                        #    diff_y = req.pred_traj[i].pose.position.y - data[k][1]
                        #    distance = math.sqrt(pow(diff_x, 2) + pow(diff_y, 2))
                        # this means this sample is in collision
                        #    if distance < 0.325:
                        #        no_sample_in_collision += 1

                        #integral_value = no_sample_in_collision / len(data)



                        #if integral_value > 0:
                        #    risk_per_stage.append(integral_value)
                        if integral_value > risk:
                            risk = integral_value
        #if (len(risk_per_stage) > 0.0001):
        #    risk = sum(risk_per_stage) / len(risk_per_stage)
        #risk = risk * 0.85138
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
            for obstacle_prediction_gaussian in obstacle_prediction.gaussians:  # this one should have the same length as no. modes
                x_poses = []
                y_poses = []
                psi = []
                covariance = []
                for obstacle_prediction_gaussian_mean_per_stage in obstacle_prediction_gaussian.mean.poses:  # should have the same length as the prediction horizon
                    x_poses.append(obstacle_prediction_gaussian_mean_per_stage.pose.position.x)
                    y_poses.append(obstacle_prediction_gaussian_mean_per_stage.pose.position.y)
                    angle = self.quaternionToAngle(obstacle_prediction_gaussian_mean_per_stage.pose)

                    psi.append(angle)
                self.mean = np.vstack((x_poses, y_poses)).T  # it includes the mean at each stage for this obstacle's mode
                #print("stacked mean", self.mean.shape)
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
                    #cov = np.array([[[major_semiaxis[i], 0], [0, minor_semiaxis[i]]]])
                    cov = np.array([[[Sigma[0, 0], Sigma[0, 1]], [Sigma[1, 0], Sigma[1, 1]]]])
                    covariance.append(cov)
                #self.covariance = np.concatenate(
                #    covariance)  # it includes the covariance matrix at each stage for this obstacle
                self.covariance_matrix.append(covariance)
                self.mean_matrix.append(self.mean)
                #print("stacked mean after each mode", test.shape)
                #print(len(self.mean_matrix))

            # we are done with all the modes of the corresponding obstacle
            # add another dimension for the number of obstacles
            if (binomial == 0):
                mean_for_all_obstacles.append(self.mean_matrix)
                covariance_for_all_obstacles.append(self.covariance_matrix)
            elif (binomial == 1 and len(self.mean_matrix) == 21):
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
            test = np.array(mean_for_all_obstacles)
            #print("shape of mean matrix:", test.shape)

                #test = np.array(self.covariance_matrix)
                #print(test.shape)
                #test = np.array(self.mean_matrix)
                #print(test.shape)

        self.mean_matrix = np.array(mean_for_all_obstacles)
        #print("shape of mean matrix:", self.mean_matrix.shape)
        #self.covariance_matrix = np.hstack(self.covariance_matrix)
        self.covariance_matrix = np.array(covariance_for_all_obstacles)
        #print("size of covaiance matix is:", self.covariance_matrix.shape)
        #self.mean_matrix = np.hstack(self.mean_matrix)
        #self.mean_matrix = np.reshape(self.mean_matrix, (20, len(obstacle_prediction.gaussians), self.num_of_obstacles, 2))
    def risk_server(self):
        s = rospy.Service("planned_traj_risk", ObservedRisk, self.risk_calculator)
        rospy.spin()


if __name__ == '__main__':
    obj = risk_assessment()
    try:
        obj.risk_server()
    except rospy.ROSInterruptException:
        pass


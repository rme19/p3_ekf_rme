
import numpy as np

import time

class ExtendedKalmanFilter:

	def __init__(self, initial_state, initial_covariance, motion_model, observation_model, **kwargs):
		# Process arguments
		proc_noise_std = kwargs.get('proc_noise_std', [0.02, 0.02, 0.01])
		obs_noise_std = kwargs.get('obs_noise_std', [0.02, 0.02, 0.01])

		self.mu = initial_state # Initial state estimate 
		self.Sigma = initial_covariance # Initial uncertainty

		self.g, self.G, self.V = motion_model() # The action model to use.
		
		# Standard deviations of the process or action model noise
		self.proc_noise_std = np.array(proc_noise_std)
		# Process noise covariance (R)
		self.R = np.diag(self.proc_noise_std ** 2)

		self.h, self.H = observation_model() # The observation model to use

		# Standard deviations for the observation or sensor model noise
		self.obs_noise_std = np.array(obs_noise_std)
		# Observation noise covariance (Q)
		self.Q = np.diag(self.obs_noise_std ** 2)

		self.exec_times_pred = []
		self.exec_times_upd = []

		
	def predict(self, u, dt):
		start_time = time.time()
        # === TODO: Implement the EKF prediction step ===
        # 1. Predict the new mean using motion model g
		print("self.mu type:", type(self.mu), "dtype:", self.mu.dtype)
		self.mu = self.g(self.mu, u, dt)
		print("self.mu type:", type(self.mu), "dtype:", self.mu.dtype)
        # 2. Compute the Jacobian of the motion model G_t
		G_t = self.G(self.mu, u, dt)
        # 3. Update the covariance
		self.Sigma = G_t @ self.Sigma @ G_t.T + self.R

		print("G_t type:", type(G_t), "dtype:", G_t.dtype)
		print("Sigma type:", type(self.Sigma), "dtype:", self.Sigma.dtype)
        # ===============================================

		end_time = time.time()
		execution_time = end_time - start_time
		self.exec_times_pred.append(execution_time)
		print(f"Execution time prediction: {execution_time} seconds")

		print("Average exec time pred: ", sum(self.exec_times_pred) / len(self.exec_times_pred))

		return self.mu, self.Sigma

	def update(self, z, dt):
		start_time = time.time()

        # === TODO: Implement the EKF correction step ===)
        # 1. Compute the Jacobian of the observation model H
		H_t = self.H(self.mu)
        # 2. Innovation covariance
		S_t = H_t @ self.Sigma @ H_t.T + self.Q

		print("H_t type:", type(H_t), "dtype:", H_t.dtype)
		print("Sigma type:", type(self.Sigma), "dtype:", self.Sigma.dtype)
		print("Q type:", type(self.Q), "dtype:", self.Q.dtype)

        # 3. Kalman gain
		K_t = self.Sigma @ H_t.T @ np.linalg.inv(S_t)
		# K_t = np.zeros((self.Sigma.shape[0], H_t.shape[0]))
        # 4. Innovation (difference between actual and expected measurement)
		y = (z - self.h(self.mu)).flatten()
        # 5. Update the state estimate
		self.mu = self.mu + K_t @ y
        # 6. Update the covariance
		I = np.eye(len(self.mu))
		self.Sigma = (I - K_t @ H_t) @ self.Sigma

        # ================================================

		end_time = time.time()
		self.exec_times_upd.append(end_time - start_time)
		return self.mu, self.Sigma

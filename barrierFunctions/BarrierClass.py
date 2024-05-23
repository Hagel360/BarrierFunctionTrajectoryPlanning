import numpy as np
from scipy import integrate

class Barrier:
    def __init__(self, d_hat=0.5, eps=1.0, n=2):
        self.d_hat = d_hat
        self.eps = eps
        self.n = n

## Kernel function
    def kernel(self, ds):
        if ds < (self.d_hat):
            return ((2/3)-ds**2) + ((1/2)*(np.abs(ds)**3))
        if self.d_hat <= ds < self.d_hat*2:
            return ((1/6)*(2-np.abs(ds))**3)
        else:
            return 0

    def h(self, ds):
        return (3/2)*self.kernel(((2*ds)/self.eps))

    def barrier(self, ds):
        return (self.h(ds)/(ds**(self.n-1)))

## Derivative Kernel function
    def derivate_kernel(self, ds):
        if ds < (self.d_hat/2):
            return -ds**2 + (3/2)*(np.abs(ds)**2) + np.sign(ds)
        if (self.d_hat/2) <= ds < self.d_hat:
            return (1/2)*(np.abs(ds))**2 * np.sign(ds)
        else:
            return 0
        
    def derivative_h(self, ds):
        return (3/2) * self.derivate_kernel((2*ds)/self.eps)
    
    def derivative_barrier(self, ds):
        return ((ds**self.n-1) * self.derivative_h(ds) - self.h(ds) * ((self.n-1) * ds**(self.n-2))) / (ds**self.n)**2
## Helpers
    def L_i(self, cur_point, next_point):
        result = np.hypot(cur_point[0] - next_point[0], cur_point[1] - next_point[1])
        return np.where(result == 0, 0.0001, result)
    
    def integrand(self, cur_point, next_point):
        return integrate.quad(lambda x: self.barrier(self.L_i(cur_point, next_point)), 0, 1)[0]
    
## Potentials
    def potential_value_p(self, point, K_list):
        sum = 0
        for traj in K_list:
            sum += self.L_i(point, traj) * self.integrand(point, traj)
        return sum
    
    def potential_value_sum(self, K_list):
        P = np.zeros((len(K_list), len(K_list[0])))
        for i, traj in enumerate(K_list):
            K_temp = np.delete(K_list, i, 0).reshape(-1, 2)
            for j, point in enumerate(traj):
                P[i][j] = self.potential_value_p(point, K_temp)
        return P
        
## Update trajectory
    def update_trajectory(self, T, K_list, lr=0.00001):
        new_vertices = np.zeros_like(T)
        for i, vertex in enumerate(T):
            gradient_sum = 0
            for K in K_list:
                # Calculate gradient for the current vertex
                for point in K:
                    distance = np.linalg.norm(vertex - point)
                    gradient_sum += self.derivative_barrier(distance) * (vertex - point) / distance
            # Update the current vertex
            new_vertices[i] = vertex - lr * gradient_sum

        return new_vertices
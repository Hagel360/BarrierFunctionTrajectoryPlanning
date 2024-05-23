import numpy as np

class Barrier:
    def __init__(self, d_hat=1.0, eps=1.0, n=2):
        self.d_hat = d_hat
        self.eps = eps
        self.n = n

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

    def update_trajectory(self, T, K_list, lr=0.000001):
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
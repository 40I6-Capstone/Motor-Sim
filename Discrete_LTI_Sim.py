import numpy as np


class DiscLTI:
    def __init__(self, A, B, C, D, K, x_0):
        self.A = A
        self.B = B
        self.C = C
        self.D = D
        self.K = K
        self.x = x_0
        self.y = 0
        self.initialized = True

    # Update system with simulation
    def update(self, u):
        self.x = np.matmul(self.A, self.x) + np.matmul(self.B, u) + self.K
        self.y = np.matmul(self.C, self.x)


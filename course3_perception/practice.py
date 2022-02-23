# import numpy as np

# R = np.array([
#     [1,0,0],
#     [0,-1,0],
#     [0,0,-1]
# ])

# k = np.array([[1,2,10]]).T

# global_homogeneous = np.array([[-0.5,1.5,9,1]]).T

# origin = np.concatenate((R,k), axis = 1) @ global_homogeneous

# print(origin)

t = 8.33
d = 150*10**3/3600*t -5/2*t**2

print(d)
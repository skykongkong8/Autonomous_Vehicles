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

des_list = [[j for i in range(10)] for j in range(5)]

pair1 = [(a,b) for a, b in zip(des_list[:-1], des_list[1:])]

pair2 = []
for i in range(len(des_list)-1):
    p1 = des_list[i]
    p2 = des_list[i+1]
    pair2.append((p1, p2))    

print(pair1 == pair2)
print(f"pair1 : {pair1} \n pair2 : {pair2}")
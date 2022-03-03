import numpy as np
import pickle

arr1 = np.array([[i*j for i in range(10)]for j in range(10)])
print(arr1[1] == arr1[1, :])

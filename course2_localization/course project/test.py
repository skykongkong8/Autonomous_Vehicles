import numpy as np
import pickle


with open('data/pt1_data.pkl', 'rb') as file:
    data = pickle.load(file)


imu_w = data['imu_w'].data

for w in imu_w:
    print(w)
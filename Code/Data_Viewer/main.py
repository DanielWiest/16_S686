import csv
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
#from tkinter.filedialog import askopenfilename


filename = "/Users/randallwiest/Desktop/NEET/NEET CL Control Project/16_S686/Multimedia/TestData/DATA_LOG_1.csv"

df = pd.read_csv(filename)
timeData = df["Time"]

#Euler_x,Euler_y,Euler_z,Grav_x,Grav_y,Grav_z,LinAcc_x ,LinAcc_y ,LinAcc_z

Euler_x = df["Euler_x"]
Euler_y = df["Euler_y"]
Euler_z = df["Euler_z"]
Grav_x = df["Grav_x"]
Grav_y = df["Grav_y"]
Grav_z = df["Grav_z"]
LinAcc_x = df["LinAcc_x"]
LinAcc_y = df["LinAcc_y"]
LinAcc_z = df["LinAcc_z"]
Control_Setpoint = df["Control_Setpoint"]

sampAvgTime = np.sum(np.array(timeData[1:])-np.array(timeData[:-1]))/(len(timeData)-1)
print("Data recorded at a average sample rate of",round(1000/sampAvgTime,2),"Hz")


plt.figure(1)
plt.subplot(4, 1, 1)
plt.title("Euler Angles")

plt.plot(timeData/1000, Euler_x)
plt.plot(timeData/1000, Euler_y)
plt.plot(timeData/1000, Euler_z)
plt.xlim([timeData[0]/1000,timeData.iloc[-1]/1000])
plt.legend(["X","Y","Z"])

plt.subplot(4, 1, 2)
plt.title("Gravity")
plt.plot(timeData/1000, Grav_x)
plt.plot(timeData/1000, Grav_y)
plt.plot(timeData/1000, Grav_z)
plt.xlim([timeData[0]/1000,timeData.iloc[-1]/1000])
plt.legend(["X","Y","Z"])

plt.subplot(4, 1, 3)
plt.title("Linear Acceleration")
plt.plot(timeData/1000, LinAcc_x)
plt.plot(timeData/1000, LinAcc_y)
plt.plot(timeData/1000, LinAcc_z)
plt.xlim([timeData[0]/1000,timeData.iloc[-1]/1000])
plt.legend(["X","Y","Z"])

plt.subplot(4, 1, 4)
plt.plot(timeData/1000, Control_Setpoint)
plt.xlim([timeData[0]/1000,timeData.iloc[-1]/1000])
plt.title("Control Setpoint")

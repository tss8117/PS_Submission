import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

df = pd.read_csv('cones.csv')
x = df['x'].copy()
y = df['y'].copy()

avg_x = np.mean(x)
avg_y = np.mean(y)
track_center = [avg_x,avg_y]

angle = np.arctan2((y-avg_y),(x-avg_x))

df2 = pd.concat([df,angle],axis=1)
df2.columns = ['x', 'y', 'color','angle']
#print(df2.to_string())
df_sorted = df2.sort_values(by='angle')
#print(df_sorted.to_string())

df_blue = df_sorted[df_sorted["color"] == "blue"]
df_yellow = df_sorted[df_sorted["color"] == "yellow"]
df_blue = df_blue.reset_index(drop=True)
df_yellow = df_yellow.reset_index(drop=True)
#print(df_blue.to_string())
#print()
#print(df_yellow.to_string())



blue_x = np.append(df_blue["x"].values, df_blue["x"].values[0])
blue_y = np.append(df_blue["y"].values, df_blue["y"].values[0])
yellow_x = np.append(df_yellow["x"].values, df_yellow["x"].values[0])
yellow_y = np.append(df_yellow["y"].values, df_yellow["y"].values[0])

centerline_x = (blue_x + yellow_x) / 2
centerline_y = (blue_y + yellow_y) / 2


#plt.plot(blue_x, blue_y, 'o-b')
#plt.plot(yellow_x, yellow_y, 'o-y')

#plt.plot(centerline_x, centerline_y, '--r')
fig,axis = plt.subplots()

axis.set_xlim(min(centerline_x)-1, max(centerline_x)+1)
axis.set_ylim(min(centerline_y)-1, max(centerline_y)+1)

animated_line, = axis.plot([], [], 'o--r',ms=8)
axis.plot(blue_x, blue_y, '^-b', label='Blue Cones')
axis.plot(yellow_x, yellow_y, '^-y', label='Yellow Cones')
axis.plot(centerline_x, centerline_y, '--r', label='Centerline')

def update(frame):
    animated_line.set_data(centerline_x[frame:frame+1], centerline_y[frame:frame+1])
    return animated_line,


animation = FuncAnimation(fig=fig, func=update, frames=len(centerline_x), interval=50)

plt.axis('equal')

animation.save("animation.mp4", writer="ffmpeg", fps=30)

plt.show()






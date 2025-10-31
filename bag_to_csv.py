import rosbag
import pandas as pd
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import os

bag_file = 'odom.bag'  # Replace with your actual .bag file
cmd_vel_data = []
odom_data = []

# Read bag file and extract data
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/beaverbot/cmd_vel', '/beaverbot/odom']):
        time = t.to_sec()
        if topic == '/beaverbot/cmd_vel':
            cmd_vel_data.append({
                'time': time,
                'linear_x': msg.linear.x,
                'angular_z': msg.angular.z
            })
        elif topic == '/beaverbot/odom':
            odom_data.append({
                'time': time,
                'pose_x': msg.pose.pose.position.x,
                'pose_y': msg.pose.pose.position.y,
                'orientation_z': msg.pose.pose.orientation.z
            })



# Convert to DataFrame
df_cmd = pd.DataFrame(cmd_vel_data).set_index('time')
df_odom = pd.DataFrame(odom_data).set_index('time')

# Create DataFrames with unique timestamps
df_cmd = pd.DataFrame(cmd_vel_data).drop_duplicates(subset='time').set_index('time')
f_odom = pd.DataFrame(odom_data).drop_duplicates(subset='time').set_index('time')

# Set sync time range based on overlap of cmd_vel and odom
start_time = df_cmd.index.min()
end_time   = df_cmd.index.max()

# Create uniform time index with 0.05s steps in the overlapping region
uniform_index = np.arange(start_time, end_time, 0.05)

# Interpolate to match timestamps
df_cmd_interp = df_cmd.reindex(uniform_index).interpolate(method='linear')
df_odom_interp = df_odom.reindex(uniform_index).interpolate(method='linear')
print(df_cmd_interp)
print(df_odom_interp)
# Combine
#df_sync = pd.concat([df_cmd_interp, df_odom_interp], axis=1).dropna()

# Save
#df_sync.to_csv("synced_output.csv")
#print(" Saved: synced_output.csv")


"""
# Create a uniform time index with 0.05s intervals
start_time = max(df_cmd.index.min(), df_odom.index.min())
end_time = min(df_cmd.index.max(), df_odom.index.max())
uniform_time = np.arange(start_time, end_time, 0.05)
uniform_index = pd.Index(uniform_time, name='time')

df_cmd = pd.DataFrame(cmd_vel_data).drop_duplicates(subset='time').set_index('time')
df_odom = pd.DataFrame(odom_data).drop_duplicates(subset='time').set_index('time')

# Resample with interpolation to match uniform 0.05s grid
df_cmd_interp = df_cmd.reindex(uniform_index).interpolate(method='linear')
df_odom_interp = df_odom.reindex(uniform_index).interpolate(method='linear')

# Combine both into one DataFrame
df_synced = pd.concat([df_cmd_interp, df_odom_interp], axis=1).dropna()
print(df_synced)
# Save to CSV
output_csv = 'synced_cmd_vel_odom_0.05s.csv'
df_synced.to_csv(output_csv)
print(f"CSV saved to: {os.path.abspath(output_csv)}")
"""
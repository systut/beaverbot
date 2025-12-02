
import pandas as pd
import matplotlib.pyplot as plt

# paths to  CSV file names ===
file_no_comp = 'trajectory_no_slip.csv'
file_comp = 'trajectory_with_slip.csv'

# the CSV data 
traj_no_comp = pd.read_csv(file_no_comp)
traj_comp = pd.read_csv(file_comp)

#Plot 
#plt.figure(figsize=(8, 6))
#plt.plot(traj_no_comp['x'].values, traj_no_comp['y'].values, label='Without Slip Compensation')
plt.plot(traj_comp['x'].values, traj_comp['y'].values, label='With slip Compensation')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Trajectory without slip')
plt.legend()
plt.grid() 
plt.show()
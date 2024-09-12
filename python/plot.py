import matplotlib.pyplot as plt
import pandas as pd

# Read the data from CSV file
data = pd.read_csv('../out/simulation_results.csv')

# Extract columns
time = data['Time']
x_position = data['X_Position']
y_position = data['Y_Position']

# Plot the data
plt.figure(figsize=(10, 6))
plt.plot(x_position, y_position, marker='o', linestyle='-', color='b')
plt.xlabel('Time (s)')
plt.ylabel('Position')
plt.title('Trajectory of the Point')
plt.grid(True)
plt.show()

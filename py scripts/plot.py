import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read the CSV file
df = pd.read_csv('/home/ali.bdeir/Desktop/half_sphere_dimensions/csv files/distances.csv')

# Calculate the RMS of the distances
distances = df['Distance']
rms = np.sqrt(np.mean(distances**2))

# Print the RMS value
print(f'Root Mean Square (RMS) of distances: {rms}')

# Plot the data
plt.figure(figsize=(10, 6))
plt.plot(df['PointIndex'], df['Distance'], marker='o', linestyle='-', color='r')
plt.xlabel('Point Index')
plt.ylabel('Distance')
plt.title('Distance of Points from Center')
plt.grid(True)
plt.show()


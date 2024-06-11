import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


noise = []
radius = []
center_x = []
center_y = []
center_z = []


with open('/home/ali.bdeir/Desktop/half_sphere_dimensions/build/sphere_parameters.txt', 'r') as file:
    next(file)
    for line in file:
        data = line.strip().split(',')
        noise.append(float(data[0]))
        radius.append(float(data[1]))
        center_x.append(float(data[2]))
        center_y.append(float(data[3]))
        center_z.append(float(data[4]))


plt.figure(figsize=(10, 5))
plt.plot(noise, radius, 'o')
plt.xlabel('Noise Amplitude')
plt.ylabel('Radius')
plt.title('Radius vs Noise Amplitude')
plt.grid(True)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
noise_amplitude = [abs(n) for n in noise]
sc = ax.scatter(center_x, center_y, center_z, c=noise_amplitude, cmap='viridis', label='Sphere Coordinates')
ax.set_xlabel('X_coordinates')
ax.set_ylabel('Y_coordinates')
ax.set_zlabel('Z_coordinates')
cbar = fig.colorbar(sc, pad=0.2)
cbar.set_label('Noise Amplitude')
cbar.outline.set_edgecolor('red')
cbar.ax.tick_params(color='red')
plt.show()


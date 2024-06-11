import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def generate_half_sphere(radius, resolution):
    points = []
    for phi in np.linspace(0, np.pi/2, resolution):  # Adjusted range for half sphere
        for theta in np.linspace(0, 2*np.pi, 2*resolution):
            x = radius * np.sin(phi) * np.cos(theta)
            y = radius * np.sin(phi) * np.sin(theta)
            z = radius * np.cos(phi)
            points.append([x, y, z])
    return np.array(points)


def generate_plane(length, width, resolution, sphere_radius):
    # Generate a grid of points in the x-y plane
    x = np.linspace(-length/2, length/2, resolution)
    y = np.linspace(-width/2, width/2, resolution)
    X, Y = np.meshgrid(x, y)
    # The plane equation is z = 0, only keep points outside the sphere
    Z = np.zeros_like(X)
    mask = X**2 + Y**2 > sphere_radius**2
    X = X[mask]
    Y = Y[mask]
    Z = Z[mask]
    # Combine the points into a single array
    points = np.column_stack([X.ravel(), Y.ravel(), Z.ravel()])
    return points


def save_pcd(filename, point_cloud):
    with open(filename, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write("WIDTH {}\n".format(point_cloud.shape[0]))
        f.write("HEIGHT 1\n")
        f.write("POINTS {}\n".format(point_cloud.shape[0]))
        f.write("DATA ascii\n")
        for point in point_cloud:
            f.write("{} {} {}\n".format(point[0], point[1], point[2]))


radius = 4.56
resolution = 50
sphere_radius = radius
plane_length = 12
plane_width = 12


half_sphere = generate_half_sphere(radius, resolution)
plane = generate_plane(plane_length, plane_width, resolution, sphere_radius)
point_cloud = np.vstack([half_sphere, plane])
save_pcd(f"/home/ali.bdeir/Desktop/half_sphere_dimensions/pcd files/half_sphere_with_plane_res({resolution}).pcd", point_cloud)
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(point_cloud[:, 0], point_cloud[:, 1], point_cloud[:, 2], s=1)
ax.set_xlabel('X', fontsize=14)
ax.set_ylabel('Y', fontsize=14)
ax.set_zlabel('Z', fontsize=14)
ax.set_title('Half Sphere with Plane Point Cloud', fontsize=16)

plt.show()

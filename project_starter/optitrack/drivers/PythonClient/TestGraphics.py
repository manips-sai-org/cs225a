import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation as R
import time
import redis, ast 

redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)

def create_box_vertices():
    """ Create vertices for a unit cube centered at the origin """
    vertices = np.array([
        [-0.5, -0.5, -0.5],
        [0.5, -0.5, -0.5],
        [0.5, 0.5, -0.5],
        [-0.5, 0.5, -0.5],
        [-0.5, -0.5, 0.5],
        [0.5, -0.5, 0.5],
        [0.5, 0.5, 0.5],
        [-0.5, 0.5, 0.5]
    ])
    return vertices

def apply_transformation(vertices, position, quaternion):
    """ Apply rotation and translation to the vertices """
    r = R.from_quat(quaternion)
    rotated_vertices = r.apply(vertices)
    transformed_vertices = rotated_vertices + position
    return transformed_vertices

def plot_box(ax, vertices):
    """ Plot a box given its vertices """
    # List of sides' polygons
    sides = [
        [vertices[j] for j in [0, 1, 2, 3]],
        [vertices[j] for j in [4, 5, 6, 7]], 
        [vertices[j] for j in [0, 1, 5, 4]], 
        [vertices[j] for j in [2, 3, 7, 6]], 
        [vertices[j] for j in [1, 2, 6, 5]], 
        [vertices[j] for j in [4, 7, 3, 0]]
    ]
    
    # Plot sides
    ax.add_collection3d(Poly3DCollection(sides, facecolors='cyan', linewidths=1, edgecolors='r', alpha=.25))

def update_plot(ax, vertices1, vertices2, frame):
    """ Update the plot for the live update """
    ax.clear()
    
    # Example: update position and orientation over time
    # position1 = np.array([np.sin(frame / 10.0), np.cos(frame / 10.0), np.sin(frame / 20.0)])
    # angle1 = frame / 20.0
    # quaternion1 = np.array([0, 0, np.sin(angle1 / 2), np.cos(angle1 / 2)])  # Rotation around z-axis
    
    # position2 = np.array([np.cos(frame / 15.0), np.sin(frame / 15.0), np.cos(frame / 25.0)])
    # angle2 = frame / 25.0
    # quaternion2 = np.array([np.sin(angle2 / 2), 0, 0, np.cos(angle2 / 2)])  # Rotation around x-axis

    position1 = np.array(ast.literal_eval(redis_client.get('sai2::optitrack::rigid_body_pos::1')))
    quaternion1 = np.array(ast.literal_eval(redis_client.get('sai2::optitrack::rigid_body_ori::1')))
        
    position2 = np.array(ast.literal_eval(redis_client.get('sai2::optitrack::rigid_body_pos::2')))
    quaternion2 = np.array(ast.literal_eval(redis_client.get('sai2::optitrack::rigid_body_ori::2')))
    
    transformed_vertices1 = apply_transformation(vertices1, position1, quaternion1)
    transformed_vertices2 = apply_transformation(vertices2, position2, quaternion2)
    
    plot_box(ax, transformed_vertices1)
    plot_box(ax, transformed_vertices2)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    # Set the limits of the plot
    max_extent = 3  # Adjust based on expected range of positions
    ax.set_xlim([-max_extent, max_extent])
    ax.set_ylim([-max_extent, max_extent])
    ax.set_zlim([-max_extent, max_extent])

def main():
    vertices = create_box_vertices()
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    frame = 0
    while True:
        update_plot(ax, vertices, vertices, frame)
        
        plt.draw()
        plt.pause(0.1)  # Pause to allow the plot to update
        frame += 1

if __name__ == "__main__":
    main()


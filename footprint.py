import numpy as np
from camera_calculator import CameraCalculator
import argparse
from mpl_toolkits import mplot3d # Needed but not called
import matplotlib.pyplot as plt


# Parse the input arguments
parser = argparse.ArgumentParser(description='Compute a ground footprint of a camera or other sensor')
parser.add_argument('-a', '--air_gap', type=float,
                    help='Distance from camera to plane of footprint (e.g., level reading '
                         'without offset)')
parser.add_argument('-c', '--camera_pos', type=float, nargs='+', help='XYZ position as tuple...for now')
parser.add_argument('-H', '--FOVh', type=float, help='Horizontal field of view of the camera in degrees')
parser.add_argument('-v', '--FOVv', type=float, help='Vertical field of view of the camera in degrees')
parser.add_argument('-r', '--roll', type=float, default=0, help='Roll of the camera (rotation y-axis) in degrees')
parser.add_argument('-p', '--pitch', type=float, default=0, help='Pitch of the camera (rotation x-axis) in degrees')
parser.add_argument('-y', '--yaw', type=float, default=90, help='Yaw of the camera (rotation z-axis) in degrees')
parser.add_argument('-d', '--datum', type=float, default=0, help='Datum elevation used for camera position')
parser.add_argument('-o', '--offset', type=float, default=None, help='Datum elevation used for camera position')
parser.add_argument('-u', '--units', type=str, default="feet", help='Unit of length measure, for labels')
args = parser.parse_args()


def centroid(arr):
    """Compute the centroid coordinate of an numpy array of XYZ points"""
    length = arr.shape[0]
    sum_x = np.sum(arr[:, 0])
    sum_y = np.sum(arr[:, 1])
    sum_z = np.sum(arr[:, 2])
    return sum_x / length, sum_y / length, sum_z / length


def getFootprint(camera_pos, FOVh, FOVv, roll=0, pitch=0, yaw=90):
    """Convenience function to drive inputs for CameraCalculator class"""
    c = CameraCalculator()
    bbox = c.getBoundingPolygon(FOVh, FOVv, roll, pitch, yaw, camera_pos)
    Xp = np.array([bbox[0].x, bbox[1].x, bbox[2].x, bbox[3].x, bbox[0].x])
    Yp = np.array([bbox[0].y, bbox[1].y, bbox[2].y, bbox[3].y, bbox[0].y])
    Zp = np.array([bbox[0].z, bbox[1].z, bbox[2].z, bbox[3].z, bbox[0].z])
    footprint = np.vstack((Xp, Yp, Zp)).T
    principle_point = np.array(centroid(footprint[0:4, :]))  # Don't average the extra point
    return footprint, principle_point


def main():
    """
        Main function call
        :return:
    """
    if args.offset is None:
        args.offset = -args.air_gap

    footprint, principle_point = getFootprint(args.camera_pos, args.FOVh, args.FOVv, args.roll, args.pitch, args.yaw)
    footprint[:, 2] = footprint[:, 2] + args.offset
    principle_point[2] = principle_point[2] + args.offset
    print(footprint)
    print(principle_point)

    ax = plt.axes(projection='3d')
    ax.scatter(args.camera_pos[0], args.camera_pos[1], args.camera_pos[2], marker="v", s=100)
    ax.plot(footprint[:, 0], footprint[:, 1], footprint[:, 2], linewidth=1.5, linestyle=":", color='black')
    ax.scatter(principle_point[0], principle_point[1], principle_point[2])
    ax.plot3D((principle_point[0], args.camera_pos[0]), (principle_point[1], args.camera_pos[1]), (principle_point[2],
                                                                                                   args.camera_pos[2]),
              'black', linestyle=":")

    ax.set_xlabel('X ' + args.units)
    ax.set_ylabel('Y ' + args.units)
    ax.set_zlabel('Z ' + args.units)
    ax.set_aspect('equal', 'datalim')
    plt.show()
    return footprint, principle_point


if __name__ == "__main__":
    try:
        main()

    except KeyboardInterrupt:
        event.set()

import time

import numpy as np
import rclpy
from matplotlib import pyplot as plt

from a_star import search
from mapUtilities import mapManipulator

POINT_PLANNER = 0
ASTAR_PLANNER = 1

PARABOLA = 0
SIGMOID = 1


class Planner:
    def __init__(self, type_, mapName="your_map/room"):
        self.type = type_
        self.mapName = mapName
        ## TODO: Adjust the laser_sig value which decides the safety distance to obstacles
        self.m_utils = mapManipulator(filename_=self.mapName, laser_sig=0.5)
        self.costMap = self.m_utils.make_likelihood_field()

    def plan(self, startPose, endPose):
        if self.type == POINT_PLANNER:
            return self.point_planner(endPose)

        elif self.type == ASTAR_PLANNER:
            return self.trajectory_planner(startPose, endPose)

    def point_planner(self, endPose):
        return (endPose[0], endPose[1])

    def trajectory_planner(self, startPoseCart, endPoseCart):
        start_time = time.time()
        startPoseCart = np.array(startPoseCart)[:2]
        endPoseCart = np.array(endPoseCart)[:2]

        # TODO: Convert to pixel coordinates using the m_utilites
        startPose = self.m_utils.position_2_cell(startPoseCart)
        endPose = self.m_utils.position_2_cell(endPoseCart)

        # convert to tuple
        startPose = (startPose[0], startPose[1])
        endPose = (endPose[0], endPose[1])
        # TODO: Call the A* search algorithm
        path = search(self.costMap, startPose, endPose)
        if path is None:
            return None

        pathCart = self.m_utils.cell_2_position(path)
        pathCart_list = pathCart.tolist()
        print("Time taken for A* is ", time.time() - start_time)

        # Visualization
        allObstacles = np.array(self.m_utils.getAllObstacles())
        plt.plot(allObstacles[:, 0], allObstacles[:, 1], "ko", markersize=4)
        plt.axis("equal")
        plt.title("A* path planning")
        plt.xlabel("x")
        plt.ylabel("y")
        # plot the path
        plt.plot(pathCart[:, 0], pathCart[:, 1], "b-", linewidth=3)
        # plot the start and end points
        plt.plot(startPoseCart[0], startPoseCart[1], "g*", markersize=10)
        plt.plot(endPoseCart[0], endPoseCart[1], "r*", markersize=10)
        plt.text(startPoseCart[0], startPoseCart[1], "start", fontsize=16)
        plt.text(endPoseCart[0], endPoseCart[1], "goal", fontsize=16)
        plt.show()

        return pathCart_list


if __name__ == "__main__":
    rclpy.init()

    # m_utilites=mapManipulator("./test_map/map.yaml", 0.1)
    #
    # map_likelihood=m_utilites.make_likelihood_field()

    # Testing the planner
    planner = Planner(ASTAR_PLANNER)
    print(planner.costMap.shape)
    # path=planner.trajectory_planner((0,0), planner.m_utilites.cell_2_position((214,53)))
    path = planner.trajectory_planner(
        planner.m_utils.cell_2_position(np.array([80, 60])),
        planner.m_utils.cell_2_position(np.array([100, 100])),
    )
    path_cells = [planner.m_utils.position_2_cell(point) for point in path]
    x_coords, y_coords = zip(*path_cells)
    plt.imshow(planner.costMap, cmap="gray")
    plt.scatter(x_coords, y_coords, color="red", s=50, marker=".", label="Points")
    plt.axis("off")
    plt.title("PGM Image")
    plt.legend()
    plt.show()

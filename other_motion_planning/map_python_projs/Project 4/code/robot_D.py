import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import matplotlib.animation as manimation

from utils import endpoints_to_edges, angle_diff, interpolate_angle
from utils import is_in_polygon, is_intersecting, line_intersection


class Robot:
    """A parent class for all robots"""

    def __init__(self, limits):
        """Initialize by providing limits of each degree of freedom"""
        # Limits in each dof, each limit is defined as (lower, upper, name)
        self.limits = limits
        self.dof = len(limits)

    def forward_kinematics(self, config):
        """Compute the endpoints of the robot given a configuration
        The last endpoint would be used for visualization of the sampling
        """
        raise NotImplementedError

    def get_edges(self):
        """Return the edges of the robot for collision checking"""
        raise NotImplementedError

    def distance(self, config1, config2):
        """Compute the distance between two configurations"""
        raise NotImplementedError

    def interpolate(self, config1, config2, num):
        """Interpolate between two configurations"""
        raise NotImplementedError

    def check_collision(
        self, config1, config2, map_corners, obstacles, obstacle_edges
    ):
        """Check colliding with obstacles between two configurations
        First perform an interpolation between the two configurations,
        then check if any of the interpolated configurations hit obstacles.
       
        arguments:
            config1 - configuration 1
            config2 - configuration 2
            map_corners - corners of the map
            obstacles - list of obstacles
            obstacle_edges - list of edges of obstacles, including map edges
        
        return:
            True if colliding with obstacles between the two configurations
        """
        # Get intepolated configurations in between config1 and config2
        configs_between = self.interpolate(config1, config2)

        # check if any of these configurations hit obstacles
        for config in configs_between:
            if self.check_collision_config(
                config, map_corners, obstacles, obstacle_edges
            ):
                return True
        return False

##    def check_collision_config(self, config, map_corners, obstacles, obstacle_edges):
##        """Check if a configuration is colliding with obstacles. Ensure that all  
##        cases are checked. Even ones that might not be present in the given map. 
##        arguments:
##            config - configuration of the robot
##            map_corners - corners of the map
##            obstacles - list of obstacles
##            obstacle_edges - list of edges of obstacles, including map edges
##        
##        return:
##            True if colliding with obstacles
##        """
##        # Get the edges of the robot for collision checking
##        robot_endpoint = self.forward_kinematics(config)[-1]
##        robot_edges = self.get_edges(config)
##
##        # Check if the robot endpoint is outside the map
##        if not is_in_polygon(robot_endpoint, map_corners):
##            return True
##
##        # Check if the robot endpoint is inside any obstacle
##        for obstacle in obstacles:
##            if is_in_polygon(robot_endpoint, obstacle):
##                return True
##
##        ### YOUR CODE HERE ###
##
##        return False



    def check_collision_config(self, config, map_corners, obstacles, obstacle_edges):
        # Get the endpoint of the robot for collision checking
        robot_endpoint = self.forward_kinematics(config)[-1]

        # Check if the robot endpoint is outside the map
        if not is_in_polygon(robot_endpoint, map_corners):
            return True

        # Check if the robot endpoint is inside any obstacle
        for obstacle in obstacles:
            if is_in_polygon(robot_endpoint, obstacle):
                return True

        # Check for collisions between robot and obstacle edges
        robot_edges = self.get_edges(config)
        
        
        for robot_edge in robot_edges:
            robot_edge = [robot_edge]
            for obstacle_edge in obstacle_edges:
                obstacle_edge = [obstacle_edge]
                if is_intersecting(robot_edge, obstacle_edge):
                    return True

        # If none of the collision checks passed, return False
        return False


    def draw_robot(self, ax, config, edgecolor="b", facecolor="g"):
        """Draw the robot given a configuration on a matplotlib axis.
        This is for visualization purpose only.
        """
        raise NotImplementedError


class PointRobot(Robot):
    """2D Point robot class"""

    def __init__(self):
        """Initialize the robot with no limits in x, y (0, 1000))"""
        super().__init__(limits=[
            (0, 1000, "x"),
            (0, 1000, "y")
        ])

    def forward_kinematics(self, config):
        """Simply return the configuration as the endpoint"""
        return [config]

    def get_edges(self, config):
        """Simply return an empty list"""
        return []

    def distance(self, config1, config2):
        """Euclidean distance"""
        x_diff = config1[0] - config2[0]
        y_diff = config1[1] - config2[1]
        return np.sqrt(x_diff**2 + y_diff**2)

    def interpolate(self, config1, config2, num=50):
        """Interpolate between two configurations"""
        configs_between = zip(
            np.linspace(config1[0], config2[0], num),
            np.linspace(config1[1], config2[1], num)
        )
        return configs_between

    def draw_robot(self, ax, config, edgecolor="b", facecolor="g"):
        ax.scatter(config[0], config[1], s=20, c=edgecolor)


class OmnidirectionalRobot(Robot):
    """Omnidirectional navigation robot class
    Its shape is defined as a rectangle with a width and a height.
    The robot could move in any direction with any angle in a 2D plane.
    """

    def __init__(self, width, height):
        """Initialize the robot with a width and height."""
        self.width = width
        self.height = height
        # Limits in each dof: (x, y, theta)
        # x, y have no limits unless bounded by the map (1000 as default)
        # theta range is (-pi, pi)
        super().__init__(limits=[
            (0, 1000, "x"),
            (0, 1000, "y"),
            (-np.pi, np.pi, "r")
        ])

    def forward_kinematics(self, config):
        """Compute the 4 corner coordinates of the robot given a configuration
        Also attach the center of the robot as the last endpoint.
        The last endpoint would be used for visualization of the sampling.
        arguments:
            config: [x, y, theta] of the rectangle

        return:
            endpoints: 4 corner coordinates of the rectangle and its center
                       [corner1, corner2, corner3, corner4, center]
        """
        # Check configuration shape
        assert len(config) == 3, "Configuration should be (x, y, theta)"

        x, y, theta = config
        endpoints = np.zeros((5, 2))

        ### YOUR CODE HERE ###

        # Define the four corners of the rectangle
        half_width = self.width / 2
        half_height = self.height / 2

        # Calculate the corners
        endpoints[0] = [x + half_height * np.cos(theta) + half_width * np.cos(theta + np.pi / 2), 
                        y + half_height * np.sin(theta) + half_width * np.sin(theta + np.pi / 2)]    #Top left of robot
        endpoints[1] = [x + half_height * np.cos(theta) - half_width * np.cos(theta + np.pi / 2),
                        y + half_height * np.sin(theta) - half_width * np.sin(theta + np.pi / 2)]    #Top Right of robot
        endpoints[2] = [x - half_height * np.cos(theta) - half_width * np.cos(theta + np.pi / 2),
                        y - half_height * np.sin(theta) - half_width * np.sin(theta + np.pi / 2)]    #Bottom Right of robot
        endpoints[3] = [x - half_height * np.cos(theta) + half_width * np.cos(theta + np.pi / 2),
                        y - half_height * np.sin(theta) + half_width * np.sin(theta + np.pi / 2)]    #Bottom Left of robot
        endpoints[4] = [x, y]                                                                                        #Center of the robot

        return endpoints

    def get_edges(self, config):
        """Compute the edges of the robot given a configuration"""
        # Get the 4 corner coordinates

        ### YOUR CODE HERE ###
        corners = self.forward_kinematics(config)
        edges_list = [(corners[i], corners[(i + 1)]) for i in range(4)]
        edges = [(tuple(point1), tuple(point2)) for point1, point2 in edges_list]
        #print(edges)
        return edges

    def distance(self, config1, config2):
        """Calculate the euclidean distance between two configurations
        arguments:
            p1 - config1, [x, y, theta]
            p2 - config2, [x, y, theta]

        return:
            distance in R^2 x S^1 space
        """

        x1, y1, theta1 = config1
        x2, y2, theta2 = config2
        euclidean_dist = np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

        # Angular distance in S^1 (circle)
        angular_dist = np.abs(theta1 - theta2)
        angular_dist = min(angular_dist, 2 * np.pi - angular_dist)

        return euclidean_dist + angular_dist
    
    def interpolate(self, config1, config2, num=50):
        """Interpolate between two configurations
        arguments:
            p1 - config1, [x, y, theta]
            p2 - config2, [x, y, theta]
        return:
            list with num number of configs from linear interploation in R^2 x S^1 space
        """

        ### YOUR CODE HERE ###
        x1, y1, theta1 = config1
        x2, y2, theta2 = config2

        # Interpolate in R^2 (x, y)
        x_interp = np.linspace(x1, x2, num)
        y_interp = np.linspace(y1, y2, num)

        # Interpolate in S^1 (theta) with angular distance consideration
        theta_interp = np.linspace(theta1, theta2, num)
        
        interpolated_configs = []
        for i in range(num):
            interpolated_configs.append([x_interp[i], y_interp[i], theta_interp[i]])

        return interpolated_configs 

    def draw_robot(self, ax, config, edgecolor="b", facecolor="pink"):
        # compute corners and draw rectangle
        corners = self.forward_kinematics(config)[:4]
        polygon = Polygon(
            corners, closed=True, edgecolor=edgecolor, facecolor=facecolor
        )
        ax.add_patch(polygon)


class KinematicChain(Robot):
    """Kinematic chain robot class
    A planar robot with a fixed base and pure revolute joints.
    Each link is a line segment.
    """

    def __init__(self, link_lengths, base=[0.1, 0.1]):
        """Initialize with a list of link lengths, and a fixed base."""
        self.base = base
        self.link_lengths = link_lengths
        self.num_joints = len(link_lengths)
        # Limits in each dof
        # assume all to be (-pi, pi)
        super().__init__(limits=[
            (-np.pi, np.pi, "r") for _ in range(self.num_joints)
        ])

    def forward_kinematics(self, config):
        """Compute the joint coordinates given a configuration of joint angles.
        The last endpoint would be used for visualization of the sampling
        arguments:
            config: A list of joint angles in radians.

        return:
            edges: A list of joint coordinates.
        """
        # Initialize the starting point as the fixed base
        joint_positions = [self.base]
        start_point = np.array(self.base)
        angle = 0

        # Compute the end points of each joint based on the configuration
        ### YOUR CODE HERE ###
        for i in range(self.num_joints):
            angle += config[i]
            end_point = start_point + np.array([self.link_lengths[i] * np.cos(angle), self.link_lengths[i] * np.sin(angle)])
            joint_positions.append(end_point)
            start_point = end_point
        return joint_positions

    def get_edges(self, config):
        """Compute the link line segments of the robot given a configuration.
        arguments:
            config: A list of joint angles in radians.

        return:
            edges: A list of line segments representing the link line segments.
        """
        # Check configuration length
        assert (
            len(config) == self.num_joints
        ), "Configuration should match the number of joints"

        ### YOUR CODE HERE ###
        edges_list = []

        joint_positions = self.forward_kinematics(config)

        for i in range(len(joint_positions) - 1):
            edges_list.append([joint_positions[i], joint_positions[i + 1]])
        edges = [(tuple(point1), tuple(point2)) for point1, point2 in edges_list]
        return edges

    def distance(self, config1, config2):
        """Calculate the euclidean distance between two configurations
        arguments:
            p1 - config1, [joint1, joint2, joint3, ..., jointn]
            p2 - config2, [joint1, joint2, joint3, ..., jointn]

        return:
            A Euclidean distance in S^1 x S^1 x ... x S^1 space
        """
        ### YOUR CODE HERE ###
        
        return np.linalg.norm(np.array(config1) - np.array(config2)) 

    def interpolate(self, config1, config2, num=50):
        """Interpolate between two configurations
        arguments:
            p1 - config1, [joint1, joint2, joint3, ..., jointn]
            p2 - config2, [joint1, joint2, joint3, ..., jointn]

        return:
            A Euclidean distance in 
            list with num number of configs from linear interploation in S^1 x S^1 x ... x S^1 space.
        """

        ### YOUR CODE HERE ###
        interpolated_configs = []
        for i in range(num):
            alpha = i / (num - 1)
            interpolated_config = [(1 - alpha) * c1 + alpha * c2 for c1, c2 in zip(config1, config2)]
            interpolated_configs.append(interpolated_config)
        return interpolated_configs
    
    def is_link_intersecting(self, E1, E2, config):
        joints = self.forward_kinematics(config)
        # print('Joints: ',joints)
        for i in range(len(joints)):
            if i > 0:
                joints[i] = joints[i].tolist()
        # print('Joints: ',joints)
        # print('E1,E2:',E1,E2)
        Line1 = list(E1[0])
        A,B = Line1
        A1, B1 = A
        A2, B2 = B
        Line2 = list(E2[0])
        A,B = Line2
        A3, B3 = A
        A4, B4 = B
        Line1 = [[A1,B1],[A2,B2]]
        Line2 = [[A3,B3],[A4,B4]]
        # print('Line1,Line2:',Line1,Line2)
        if A1 == A2:
            A1 = A1-0.0001
        if A3 == A4:
            A3 = A3-0.0001
            
        M1 = (B2-B1)/(A2-A1)
        M2 = (B4-B3)/(A4-A3)
        
        if (M1 == M2):
            return True
        
        Point = line_intersection(Line1, Line2)
        if Point:
            # print('point:',list(Point))
            if list(Point) not in joints:
                # print('Collision')
                return True
        return False
    

    def check_collision_config(self, config, map_corners, obstacles, obstacle_edges):
        # Get the endpoint of the robot for collision checking
        robot_endpoint = self.forward_kinematics(config)[-1]

        # Check if the robot endpoint is outside the map
        if not is_in_polygon(robot_endpoint, map_corners):
            return True

        # Check if the robot endpoint is inside any obstacle
        for obstacle in obstacles:
            if is_in_polygon(robot_endpoint, obstacle):
                return True

        # Check for collisions between robot and obstacle edges
        robot_edges = self.get_edges(config)
        #print("robot_edge",robot_edges)
        
        # print('Robot edges:',robot_edges)
        # print('Obstacle edges:',obstacle_edges)
        for robot_edge in robot_edges:
            robot_edge = [robot_edge]
            for obstacle_edge in obstacle_edges:
                obstacle_edge = [obstacle_edge]
                if is_intersecting(robot_edge, obstacle_edge):
                    return True
                
            for temp in robot_edges:
                temp = [temp]
                # print('R:',robot_edge)
                # print('T:',temp)
                if temp != robot_edge:
                    if self.is_link_intersecting(temp, robot_edge, config):
                        return True
        # If none of the collision checks passed, return False
        return False
    
    def draw_robot(self, ax, config, edgecolor="b", facecolor="black"):
        # compute joint positions and draw lines
        positions = self.forward_kinematics(config)
        # Draw lines between each joint
        for i in range(len(positions) - 1):
            line = np.array([positions[i], positions[i + 1]])
            ax.plot(line[:, 0], line[:, 1], color=edgecolor)
        # Draw joint
        for i in range(len(positions)):
            ax.scatter(positions[i][0], positions[i][1], s=5, c=facecolor)

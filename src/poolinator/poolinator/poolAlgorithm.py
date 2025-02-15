"""
Class used for the pool algorithm.

The basic algorithm is structured as follows:
    1. Calculate a direct shot from the nearest pocketable ball.
    2. If no direct shot, shoot directly at a ball to get new positions.
"""

import math
import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from poolinator.bridger import quaternion_from_euler


def calc_ang(ball_1, ball_2):
    """
    Calculate the angle from ball_1 to ball_2.

    Args:
    ----
    ball_1 (Vector3):
        (x, y, z) location of the center of first ball
    ball_2 (Vector3):
        (x, y, z) location of the center of second ball

    Returns
    -------
    angle (float):
        Angle between both balls

    """
    return math.atan2(ball_2.y - ball_1.y, ball_2.x - ball_1.x)


class PoolAlgorithm:
    """A class to plan shots in pool."""

    def __init__(self, balls, pockets, logger=None):
        """
        Initialize the state of the pool balls on the table.

        Args:
        ----
        balls (dict):
            A dictionary of every pool ball on the table containing
            the name of the ball and its position (Vector3)
            {"Name" : (x, y, z)}
        pockets:
            A list of every pocket on the table containing its x, y, z
            value [(Vector3)]
        logger:
            A node logger used for debugging (otherwise, None).

        """
        self.logger = logger
        self.balls = balls
        self.pockets = pockets
        self.D_ball = 0.0258  # diameter of pool balls (meters)
        self.D_stick = 0.037  # diameter of pool stick (meters)
        self.d = (
            0.12  # distance from target shot and end-effector frame (meters)
        )

        self.cue_ball_pos = None
        for key, value in balls.items():
            if key == 'blue_ball':
                self.cue_ball_pos = value  # allow easy access to cue ball pos
        self.break_radius = 0.05  # cluster radius to calculate where to break

    def check_obstacle(self, ball_1, ball_2, dist, ball_ignore=None):
        """
        Check if a ball will interfere with the impact between ball 1 and ball 2.

        Args:
        ----
        ball_1 (Vector3):
            (x, y, z) position of ball 1 (i.e. cue ball)
        ball_2 (Vector3):
            (x, y, z) position of ball 2 (i.e. target_ball)
        dist (float):
            distance from trajectory line to center of obstacle
        ball_ignore (Vector3):
            (x, y, z) position of ball to ignore, default None

        Returns
        -------
        obstruction(Bool):
            Boolean for whether a ball is in the trajectory path (if true, a
            different target ball or imact must be used)

        """
        obstruction = True

        # Make sure there is a potential obstacle
        if len(self.balls) < 3:
            return False

        # Check for obstruction
        slope_a = ball_1.y - ball_2.y
        slope_b = ball_2.x - ball_1.x
        slope_c = ball_1.x * ball_2.y - ball_2.x * ball_1.y
        for _, value in self.balls.items():
            # ball_ignore used when calculating final position of cue_ball
            if ball_1 == value or ball_2 == value or ball_ignore == value:
                continue

            # Check if potential obstruction is within the appropriate range
            if not (
                min(ball_1.x, ball_2.x) <= value.x <= max(ball_1.x, ball_2.x)
            ):
                continue

            # Calculate perpendicular distance from the line
            d_obs_num = abs(slope_a * value.x + slope_b * value.y + slope_c)
            d_obs_denom = math.sqrt(slope_a**2 + slope_b**2)
            d_obs = d_obs_num / d_obs_denom
            if d_obs < dist:
                return obstruction
        return False

    def calc_cue_pos(self, cue_ball, target_ball, impact):
        """
        Calculate the position of the cue.
        
        Given the position of the target ball, and desired impact (either a pocket 
        or wall position), calculate the end-effector position required to make
        the shot.

        Args:
        ----
        cue_ball (Vector3):
        target_ball (Vector3):
            (x, y, z) position of the target ball.
        impact (Vector3):
            (x, y, z) position of the final impact (either a wall
            position or the coordinates of the target pocket).

        Returns
        -------
        possible (Bool):
            Boolean for whether a ball is in the trajectory path.
            If false, a different target ball or imact must be used.
        ee (Point):
            (x, y, z) position of the cue stick.
        strike_ang (float):
            Rotation about the z axis at that point

        """
        strike_ang = None
        possible = False
        neutral_cue = Vector3()
        neutral_cue.x = 0.0
        neutral_cue.y = 0.0
        neutral_cue.z = 0.0

        # Check if target ball can go to impact point
        if self.check_obstacle(target_ball, impact, self.D_ball):
            # If true, there is an obstacle, so return false and
            # next two values don't matter
            return possible, neutral_cue, strike_ang

        # Calculate final position of cue ball at impact
        pocket_ang = calc_ang(target_ball, impact)
        cf = Vector3()
        cf.x = target_ball.x - self.D_ball * math.cos(pocket_ang)
        cf.y = target_ball.y - self.D_ball * math.sin(pocket_ang)
        cf.z = cue_ball.z

        # Check if cue ball can go to final position without obstruction
        if self.check_obstacle(
            cue_ball, cf, self.D_ball, target_ball
        ):
            return possible, neutral_cue, strike_ang

        # Calculate striking angle
        strike_ang = calc_ang(cue_ball, cf)

        sharpness = abs(pocket_ang - strike_ang)
        # check that angle of shot is reasonable
        if sharpness >= math.radians(45.0):
            return possible, neutral_cue, strike_ang

        ee = Point()
        ee.x = cue_ball.x - self.d * math.cos(strike_ang)
        ee.y = cue_ball.y - self.d * math.sin(strike_ang)
        ee.z = cue_ball.z

        if self.check_obstacle(ee, cue_ball, self.D_stick):
            return possible, neutral_cue, strike_ang

        possible = True
        # Return appropriate values

        return possible, ee, strike_ang

    def sink_cue_ball(self, ball, impact):
        """
        Shoot a ball directly at the point of impact.

        Does not account for obstacles or the angle of the balls after impact. 
        Used when there is no possible shot to a pocket (when `ball` is the cue ball).

        Args:
        ----
        ball (vector3):
            Position of the ball the arm is aiming for. 
        impact (vector3):
            The x, y, z vector3 point for the pocket or point on the wall.
        
        Returns
        -------
        ee(Point):
            Desired position of the end-effector frame.
        strike_ang(float):
            Rotation about the z axis at that point 

        """
        # Calculate striking angle
        strike_ang = calc_ang(ball, impact)
        ee = Point()
        ee.x = ball.x - self.d * math.cos(strike_ang)
        ee.y = ball.y - self.d * math.sin(strike_ang)
        ee.z = ball.z

        return ee, strike_ang

    def calc_cue_targ(self, cue_ball, pockets):
        """
        Calculate the final target_ball and desired cue pose for the shot.

        Args:
        ----
        cue_ball (Vector3):
            Position of the cue_ball.

        [pocket]:
            A list of every pocket on the table containing its x, y, z
            value (Vector3).

        Returns
        -------
        ee(Point):
            Desired position of the end-effector frame.
        strike_ang(float):
            Rotation about the z axis at that point.
        target_name(string):
            Name of the target ball.

        """
        N = len(pockets)

        # Check direct shot
        for key1, value1 in self.balls.items():
            if key1 == "red_ball":
                # Check if it is the last shot
                if len(self.balls) == 1:
                    # If only cue ball left, send to pocket 1
                    ee, strike_ang = self.sink_cue_ball(cue_ball, pockets[2])
                    return ee, strike_ang, key1
                continue

            for i in range(N):
                possible, ee, strike_ang = self.calc_cue_pos(
                    cue_ball, value1, pockets[i]
                )
                if possible:
                    return ee, strike_ang, key1
        
        # No possible shot, so just aim for first ball in list to move balls
        for key2, value2 in self.balls.items():
            if key2 == "red_ball":
                continue
            else:
                ee, strike_ang = self.sink_cue_ball(cue_ball, value2)
                return ee, strike_ang, key2

    def calc_strike_pose(self, cue_ball, balls, pockets, ee_list=None):
        """
        Calculate the end-effector pose in the base frame for the desired shot.

        Args:
        ----
        cue_ball (Vector3):
            Position of the cue_ball
        balls (dict):
            A dictionary of every pool ball on the table containing
            the name of the ball and its position (Vector3).
            {"Name" : (x, y, z)}
        [pocket]:
            A list of every pocket on the table containing its x, y, z
            value (Vector3).
        [ee_list]:
            A list of previously tried end-effector positions that resulted
            in a singularity or joint limit.

        Returns
        -------
        eePose (Pose):
            The end-effector pose that the franka should move to (in the base
            frame).

        """
        self.balls = balls
        ee, strike_ang, _ = self.calc_cue_targ(cue_ball, pockets)
        eePose = Pose()
        eePose.position = ee
        q = quaternion_from_euler(np.pi, 0.0, float(strike_ang) - np.pi / 4)
        eePose.orientation = q
        return eePose

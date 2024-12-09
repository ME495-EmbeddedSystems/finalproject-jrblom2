"""
Class used for the pool algorithm.

The basic algorithm is structured as follows:
    1. Calculate a direct shot from the nearest pocketable ball
    2. If no direct shot, calculate a wall shot with, at most, one impact from the 
       nearest pocketable ball
    3. If that's not possible, break up any clusters

Working functions (I think):
    - calc_dist
    - 

"""

import math
import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from poolinator.bridger import quaternion_from_euler


def calc_ang(ball_1, ball_2):
    """
    Calculates angle from ball_1 to ball_2
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


def calc_dist(pos_1, pos_2):
    """
    Calculates Euclidean distance between two points in 3D.

    Args:
    ----
    pos_1 (Vector3):
        (x, y, z) location of the first ball or object
    pos_2 (Vector3):
        (x, y, z) location of the second ball or object

    Returns
    -------
    dist (float):
        Distance between the two points
    """
    x_comp = pow((pos_2.x - pos_1.x), 2)
    y_comp = pow((pos_2.y - pos_1.y), 2)
    z_comp = pow((pos_2.z - pos_1.z), 2)
    return math.sqrt(x_comp + y_comp + z_comp)


class PoolAlgorithm:
    """A class to plan shots in pool."""

    def __init__(self, balls, pockets):
        """
        Initialize the state of the pool balls on the table.

        Args:
        ----
        balls (dict):
            A dictionary of every pool ball on the table containing
            the name of the ball and its position (Vector3).
            {"Name" : (x, y, z)}
        [pocket]:
            A list of every pocket on the table containing its x, y, z
            value (Vector3).

        Returns
        -------
        target (dict):
            Single entry of a ball that is being targeted
        contact_point (Pose):
            The final pose of the cue to hit the right
            point on the cue ball in the base frame.

        """
        self.balls = balls
        self.pockets = pockets
        self.D_ball = 0.00258  # diameter of pool balls (meters)
        self.D_stick = 0.0037  # diameter of pool stick (meters)
        self.d = (
            0.175  # distance from target shot and end-effector frame (meters)
        )

        self.cue_ball_pos = None
        for key, value in balls.items():
            if key == 'red_ball':
                self.cue_ball_pos = value  # allow easy access to cue ball pos
        self.break_radius = 0.05  # cluster radius to calculate where to break

    def check_obstacle(self, ball_1, ball_2, dist, ball_ignore=None):
        """
        Checks if a ball on the table will interfere with the
        impact between ball 1 and ball 2.

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
            Boolean for whether a ball is in the trajectory path.
            If true, a different target ball or imact must be used.
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

            # Check if potential obstruction is actually within the appropriate range
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

    def calc_cue_pos(self, target_ball, impact):
        """
        Calculates the position of the cue given the position of the
        target ball, and desired impact (either a pocket or wall position).

        Args:
        ----
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
        cf.x = target_ball.x + self.D_ball * math.cos(pocket_ang)
        cf.y = target_ball.y + self.D_ball * math.sin(pocket_ang)
        cf.z = self.cue_ball_pos.z

        # Check if cue ball can go to final position without obstruction
        if self.check_obstacle(
            self.cue_ball_pos, cf, self.D_ball, target_ball
        ):
            return possible, neutral_cue, strike_ang

        # Calculate striking angle
        strike_ang = calc_ang(self.cue_ball_pos, cf)
        sharpness = abs(
            pocket_ang - strike_ang
        )  # check that angle of shot is reasonable
        if sharpness >= math.radians(45.0):
            return possible, neutral_cue, strike_ang

        ee = Point()
        ee.x = self.cue_ball_pos.x - self.d * math.cos(strike_ang)
        ee.y = self.cue_ball_pos.y - self.d * math.sin(strike_ang)
        ee.z = self.cue_ball_pos.z

        if self.check_obstacle(ee, self.cue_ball_pos, self.D_stick):
            return possible, neutral_cue, strike_ang

        possible = True
        # Return appropriate values
        return possible, ee, strike_ang

    def break_balls(self):
        """
        If no possible direct shot, break cluster of balls.

        """
        pass

    def calc_cue_targ(self):
        """
        Calculates final target_ball and desired cue pose for the shot.

        Args:
        ----

        Returns
        -------
        cue(Point):
            Position of the cue stick.
        strike_ang(float):
            Rotation about the z axis at that point
        target_name(string):
            Name of the target ball
        """
        N = len(self.pockets)
        # Check direct shot
        for key1, value1 in self.balls.items():
            if key1 == "cue_ball":
                continue
            for i in range(N):
                possible, cue, strike_ang = self.calc_cue_pos(
                    value1, self.pockets[i]
                )
                if possible:
                    return cue, strike_ang, key1

    def test_strike_pose(self, ball, impact):
        """
        Test function for a single ball.
        Impact is the x, y, z vector3 point for the pocket or point on the wall.
        """

        # Calculate striking angle
        strike_ang = calc_ang(ball, impact)
        ee = Point()
        ee.x = ball.x
        ee.y = ball.y
        ee.z = ball.z

        q = quaternion_from_euler(np.pi, 0.0, float(strike_ang) - np.pi / 4)
        eePose = Pose()
        eePose.position = ee
        eeOrientation = q
        eePose.orientation = eeOrientation
        return eePose

    def calc_strike_pose(self):
        """
        Calculates end-effector pose in the base frame for the desired
        shot.

        Returns
        -------
        eePose (Pose)
        """

        ee, strike_ang, _ = self.calc_cue_targ()
        eePose = Pose()
        eePose.position = ee
        eeOrientation = Quaternion()
        eeOrientation.x = 0
        eeOrientation.y = 0
        eeOrientation.z = math.sin(strike_ang / 2)
        eeOrientation.w = math.cos(
            strike_ang / 2
        )  # Pure rotation about the z axis
        eePose.orientation = eeOrientation
        return eePose

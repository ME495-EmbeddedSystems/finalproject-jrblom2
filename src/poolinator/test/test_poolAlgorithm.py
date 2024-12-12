import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from poolinator.poolAlgorithm import PoolAlgorithm
import pytest
import random
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3

""" Simple test cases for poolAlgorithm.py."""

def calculate_pocket_positions():
    """
    Get the [x, y] of each pocket.

    Returns
    -------
    [pockets]: 
        List of tuples [x, y] for each pocket.

    """
    # Top left corner
    p1 = Vector3()
    p1.x = -pool_length/2
    p1.y = pool_width/2
    p1.z = 0.2
    # Bottom left corner
    p2 = Vector3()
    p2.x = -pool_length/2
    p2.y = -pool_width/2
    p2.z = 0.2
    # Top middle
    p3 = Vector3()
    p3.x = 0.0
    p3.y = pool_width/2
    p3.z = 0.2
    # Bottom middle
    p4 = Vector3()
    p4.x = 0.0
    p4.y = -pool_width/2
    p4.z = 0.2
    # Top right corner
    p5 = Vector3()
    p5.x = pool_length/2
    p5.y = pool_width/2
    p5.z = 0.2
    # Bottom right corner
    p6 = Vector3()
    p6.x = pool_length/2
    p6.y = -pool_width/2
    return ([p1, p2, p3, p4, p5, p6])

def generate_ball_positions():
    """
    Generate pool balls from the center of the pool table (0, 0).

    Returns
    -------
    balls (dict):
        a dictionary of the name of the ball and its position.

    """
    balls = []
    cue_ball = Vector3()
    cue_ball.x = 0.0
    cue_ball.y = 0.0
    cue_ball.z = 0.2
    ball_1 = Vector3()
    ball_2 = Vector3()
    ball_3 = Vector3()
    ball_1.x = 0.2
    ball_1.y = 0.07
    ball_1.z = 0.2 
    ball_2.x = 0.195
    ball_2.y = 0.065
    ball_2.z = 0.2
    ball_3.x = -0.2
    ball_3.y = -0.07
    ball_3.z = 0.2
    balls = {"red_ball": cue_ball, "ball_1": ball_1, "ball_2": ball_2, "ball_3": ball_3}
    return balls
    
D_ball = 0.0258  # diameter of pool balls (meters)
D_stick = 0.037
pool_length = 0.5
pool_width = 0.2
balls = generate_ball_positions()
pockets = calculate_pocket_positions()
PA = PoolAlgorithm(balls, pockets, None)  # start algorithm with logger as None

def test_obstruction():
    assert PA.check_obstacle(balls["red_ball"], balls["ball_1"], D_ball) == True
    assert PA.check_obstacle(balls["ball_1"], pockets[4], D_ball) == False

def test_calc_cue_pos1():
    possible, cue, strike_ang = PA.calc_cue_pos(balls["red_ball"], balls["ball_3"], pockets[0])
    assert not possible

def test_calc_cue_pos2():
    possible, cue, strike_ang = PA.calc_cue_pos(balls["red_ball"], balls["ball_3"], pockets[1])
    assert possible
    assert round(strike_ang) == -3

def test_calc_cue_pos3():
    possible, cue, strike_ang = PA.calc_cue_pos(balls["red_ball"], balls["ball_3"], pockets[2])
    assert not possible

def test_calc_cue_pos4():
    possible, cue, strike_ang = PA.calc_cue_pos(balls["red_ball"], balls["ball_3"], pockets[3])
    assert not possible

def test_calc_cue_pos5():
    possible, cue, strike_ang = PA.calc_cue_pos(balls["red_ball"], balls["ball_3"], pockets[4])
    assert not possible

def test_calc_cue_pos6():
    possible, cue, strike_ang = PA.calc_cue_pos(balls["red_ball"], balls["ball_3"], pockets[5])
    assert not possible

def test_balls():
    cue, strike_ang, key1 = PA.calc_cue_targ(balls["red_ball"], pockets)
    assert key1 == "ball_1"
    assert strike_ang == 0.3087105973592545
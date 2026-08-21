import numpy as np


def initialize_set(
    NUM,
    INI_X,
    INI_V,
    TARGET,
    R_MIN,
    EPSILON,
    H,
    KK,
    EPISODES,
    WALL_COLLISION_MULTIPLIER=2.0,
    ENV_TYPE=None,
    NUM_MOVING_DRONES=None,
):
    """Initialize the shared simulation settings used by the MPC modules."""
    global Num
    global num_moving_drones
    global K
    global h
    global core_num
    global episodes
    global r_min
    global epsilon
    global wall_collision_multiplier
    global env_type
    global ini_x
    global ini_v
    global target
    global terminal_index_list
    global position_list
    global pos_list

    Num = NUM
    num_moving_drones = NUM_MOVING_DRONES if NUM_MOVING_DRONES is not None else NUM
    K = KK
    h = H
    core_num = 1
    episodes = 150 if EPISODES is None else EPISODES
    r_min = R_MIN
    epsilon = EPSILON
    wall_collision_multiplier = WALL_COLLISION_MULTIPLIER
    env_type = ENV_TYPE

    ini_x = [np.array(x, dtype=np.float64) for x in INI_X]
    ini_v = [np.array(v, dtype=np.float64) for v in INI_V]
    target = [np.array(t, dtype=np.float64) for t in TARGET]

    # Remaining MPC horizon for each agent.
    terminal_index_list = [K for _ in range(Num)]
    # Position history for each agent.
    position_list = [[] for _ in range(Num)]
    # Nominal paths used by MPC collision avoidance.
    pos_list = None

import      SET
import      datetime
from        run          import *
from        others   import *
import      numpy        as np
import pickle
import copy
import os
import csv
from trajectory_planner import build_trajectory_controller
from yield_control import build_policy_yield_controller

def data_capture(a, b, c):
    data = {
        'pos_list': copy.copy(a),
        'position_list': copy.copy(b),
        'terminal_index_list': copy.copy(c)
    }
    return data

def initialize(cargo_configs=None):
    agent_list=[]
    for i in range(SET.Num):
        kwargs = {}
        if cargo_configs is not None and i < len(cargo_configs):
            cfg = cargo_configs[i]
            kwargs['cargo_type'] = cfg['cargo_type']
            kwargs['time_to_expiry'] = cfg['time_to_expiry']
            kwargs['patient_acuity'] = cfg['patient_acuity']
        agent_list+=[ uav(i,SET.ini_x[i],SET.ini_v[i],SET.target[i],SET.K, **kwargs) ]

    return agent_list

def PLAN( Num, ini_x, ini_v,target,r_min,epsilon,h,K,episodes, num_moving_drones=None, wall_collision_multiplier=2.0, verbose=True, env_type=None, cargo_configs=None, policy_recipe=None, round_trip_params=None, trajectory_mode="baseline"):

    # os.sched_setaffinity(0,[0,1,2,3,4,5,6,7])
    
    SET.initialize_set(Num, ini_x, ini_v, target,r_min,epsilon,h,K,episodes, wall_collision_multiplier, env_type, NUM_MOVING_DRONES=num_moving_drones)

    obj = {}

    ReachGoal=False

    episodes=SET.episodes
    
    agent_list=initialize(cargo_configs=cargo_configs)

    collect_data(agent_list)

    obj[0] = data_capture(SET.pos_list, SET.position_list, SET.terminal_index_list)

    velocity_data = {i: [] for i in range(Num)}
    path_data = {i: [] for i in range(Num)}

    if num_moving_drones is None:
        num_moving_drones = Num


    # Calculate nominal trajectories for each robot
    nominal_trajectories = {}
    for robot_id in range(num_moving_drones):
        initial_x, initial_y = ini_x[robot_id]  # Initial position
        final_x, final_y = target[robot_id]     # Final position
        nominal_trajectories[robot_id] = {
            'initial': (initial_x, initial_y),
            'final': (final_x, final_y),
            'step_size_x': (final_x - initial_x) / episodes,  # Step size in x direction
            'step_size_y': (final_y - initial_y) / episodes,  # Step size in y direction
        }

    # Track whether each robot has reached its target
    target_reached = [False] * num_moving_drones  # Initialize to False for all robots
    
    # Track individual completion times for each robot
    individual_completion_times = [episodes] * num_moving_drones  # Default to full episodes
    
    # Track when all robots reach their goals for make-span calculation
    all_goals_reached = False
    completion_step = episodes  # Default to full episodes if not all reach goals

    # Track which drones were yielding in the previous step (for MPC reset on release)
    pad_previously_yielding = set()

    # Final submission branch: the CLI selects either yield_control or a
    # trajectory_planner mode. Internally both run on the landing-pad env.
    if env_type == 'landing_pad':
        if round_trip_params and round_trip_params.get('use_trajectory_planner', False):
            controller = build_trajectory_controller(
                trajectory_mode,
                cargo_configs,
                round_trip_params,
                target,
                ini_x,
                num_moving_drones,
            )
        elif policy_recipe is not None:
            controller = build_policy_yield_controller(
                target=target,
                policy_recipe=policy_recipe,
            )
        else:
            raise ValueError(
                "Final submission requires either the `yield_control` track "
                "or a `trajectory_planner` mode."
            )
    else:
        controller = None

    # Per-frame log: captures controller decision each step for animation labels
    frame_log = [{'allowed': None, 'yielding': set(), 'method': None, 'scores': {}}]  # frame 0 = initial

    # the main loop
    start =datetime.datetime.now()
    end = start

    for i in range(1,episodes+1):
        end_last=end

        # Landing-pad lifecycle management (via controller)
        if controller is not None:
            controller.cleanup_landed(agent_list, target_reached,
                                      num_moving_drones, SET.K)

        obstacle_list=get_obstacle_list(agent_list,SET.Num)

        # Determine which drones to run MPC on this step
        yielding_drones = set()
        step_decision = {'allowed': None, 'yielding': set(), 'method': None, 'scores': {}}
        if controller is not None:
            active_drones = [j for j in range(num_moving_drones)
                            if not target_reached[j]]

            if len(active_drones) >= 1:
                result = controller.select_active_drone(
                    agent_list, active_drones, i, verbose
                )
                yielding_drones = result["yielding"]
                controller.freeze_yielding(agent_list, yielding_drones)
                step_decision = {
                    'allowed': result.get('allowed'),
                    'yielding': result.get('yielding', set()),
                    'method': result.get('method'),
                    'scores': result.get('scores', {}),
                }

        # Build list of drones to process (exclude landed and yielding)
        process_indices = [j for j in range(num_moving_drones)
                          if not target_reached[j] and j not in yielding_drones]
        
        # Reset MPC warm-start for drones just released from yielding
        if controller is not None:
            released = controller.get_released_drones(
                process_indices, pad_previously_yielding
            )
            controller.reset_mpc(agent_list, released, verbose)
        pad_previously_yielding = yielding_drones.copy()

        # Run MPC only for active, non-yielding drones
        if process_indices:
            process_agents = [agent_list[j] for j in process_indices]
            process_agents = run_one_step(process_agents, obstacle_list, verbose)
            for idx, j in enumerate(process_indices):
                agent_list[j] = process_agents[idx]

        # Update position history for drones that skipped MPC
        if controller is not None:
            controller.update_idle_positions(
                agent_list, process_indices, target_reached,
                target, num_moving_drones
            )

        # print
        end = datetime.datetime.now()
        if verbose:
            print("Step %s have finished, running time is %s"%(i,end-end_last))

        # Per-step hook: TTE countdown + round-trip FSM transitions.
        if controller is not None:
            controller.step_update(agent_list, target_reached, num_moving_drones)
    

        # Store velocity data
        for j, agent in enumerate(agent_list):
            if j < num_moving_drones:
                # Check if the robot has reached its target
                if not target_reached[j]:
                    distance_to_target = np.linalg.norm(agent.p - target[j])
                    # Use appropriate threshold based on environment
                    goal_threshold = 0.05 if env_type == 'landing_pad' else 0.02
                    if distance_to_target < goal_threshold:
                        target_reached[j] = True
                        individual_completion_times[j] = i  # Record the exact step when goal was reached
                        if verbose:
                            print(f"Robot {j} reached goal at step {i}, distance: {distance_to_target:.4f}, position: {agent.p}, target: {target[j]}")
                    
                    # Debug: Show progress for robots that are getting closer
                    if i % 20 == 0 and verbose:  # Every 20 steps
                        print(f"Robot {j} progress at step {i}: distance to goal = {distance_to_target:.4f}")

                vx, vy = agent.v  # Extract vx and vy from the agent's velocity
                velocity_data[j].append([vx, vy])

                # Actual position
                if target_reached[j]:
                    # Freeze actual position at the target
                    px, py = target[j]
                else:
                    # Use the current position
                    px, py = agent.p[0], agent.p[1]

                # Nominal position (always calculated for all steps)
                nominal_x = nominal_trajectories[j]['initial'][0] + i * nominal_trajectories[j]['step_size_x']
                nominal_y = nominal_trajectories[j]['initial'][1] + i * nominal_trajectories[j]['step_size_y']
                # Append actual and nominal positions to path_data
                path_data[j].append([px, py, nominal_x, nominal_y])
            else: # For stationary agents
                vx, vy = agent.v
                velocity_data[j].append([vx, vy])
                px, py = agent.p[0], agent.p[1]
                path_data[j].append([px, py, px, py]) # nominal is same as actual

        # Check if all moving robots have reached their goals (for make-span calculation)
        if controller is not None:
            done = controller.all_finished(target_reached, num_moving_drones)
        else:
            done = all(target_reached[:num_moving_drones])
        if not all_goals_reached and done:
            all_goals_reached = True
            completion_step = i
            if verbose:
                print(f"All moving robots reached their goals at step {i}")

        collect_data(agent_list)

        obj[i] = data_capture(SET.pos_list, SET.position_list, SET.terminal_index_list)
        frame_log.append(step_decision)

        if all_goals_reached:
            break
            
        #ReachGoal=check_reach_target(agent_list)

    obj['goal'] = SET.target

    # Save velocity data to CSV files
    for robot_id, velocities in velocity_data.items():
        if num_moving_drones is not None and robot_id >= num_moving_drones:
            continue
        filename = f"avg_delta_velocity_robot_{robot_id}.csv"
        with open(filename, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["vx", "vy"])  # Write header
            writer.writerows(velocities)  # Write velocity data

    print("Velocity CSV files saved.")

    # Save path deviation data to CSV files
    # Each file contains the actual and nominal positions for each robot
    for robot_id, positions in path_data.items():
        if num_moving_drones is not None and robot_id >= num_moving_drones:
            continue
        filename = f"path_deviation_robot_{robot_id}.csv"
        with open(filename, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["px", "py", "nominal_x", "nominal_y"])  # Write header
            writer.writerows(positions)  # Write position data

    print("Path deviation CSV files saved.")
    
    # Final check: if completion_step is still episodes, it means no robots reached goals
    if completion_step == episodes:
        if verbose:
            print(f"No robots reached their goals within the simulation time ({episodes} steps)")
    else:
        if verbose:
            print(f"All robots reached goals at step {completion_step} out of {episodes} total steps")
    
    # Report completion statistics
    successful_robots = sum(target_reached[:num_moving_drones])
    if verbose:
        print(f"\nCompletion Statistics:")
        print(f"  - Total moving robots: {num_moving_drones}")
        print(f"  - Robots that reached goals: {successful_robots}")
        print(f"  - Success rate: {(successful_robots/num_moving_drones)*100:.1f}%")
    
    if successful_robots > 0 and verbose:
        successful_times = [individual_completion_times[i] for i in range(num_moving_drones) if target_reached[i]]
        print(f"  - Fastest completion time: {min(successful_times)} steps")
        print(f"  - Slowest completion time: {max(successful_times)} steps")
        print(f"  - Average completion time: {sum(successful_times)/len(successful_times):.1f} steps")
    
    # Save TTG (time-to-goal) for each moving agent
    ttg_list = []
    for robot_id in range(num_moving_drones):
        # Use the individual completion time for each robot
        ttg = individual_completion_times[robot_id]
        reached_goal = target_reached[robot_id]
        ttg_list.append([robot_id, ttg, reached_goal])
    with open("ttg_impc_dr.csv", mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["robot_id", "ttg", "reached_goal"])  # Added reached_goal column
        writer.writerows(ttg_list)
    print("TTG CSV file saved.")

    if controller is not None and getattr(controller, "_llm_advisor", None) is not None:
        controller._llm_advisor.print_summary()
    if controller is not None and hasattr(controller, "print_negotiator_summaries"):
        controller.print_negotiator_summaries()
    
    return obj, agent_list, completion_step, frame_log
    

import numpy as np
import sys
import os
import json
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.animation import FuncAnimation
import matplotlib.lines as mlines
from test import PLAN
from plot import plot_trajectory
from pathlib import Path

# Import standardized environment configuration
sys.path.append(str(Path(__file__).resolve().parents[3] / 'src'))
from utils import StandardizedEnvironment

TRACK_ALIASES = {
    "yield_control": "yield_control",
    "policy_yield": "yield_control",
    "trajectory_planner": "trajectory_planner",
}
TRAJECTORY_MODES = {"baseline", "llm", "lookahead", "compare_all"}


def _load_config(config_arg):
    cfg_path = Path(config_arg)
    if not cfg_path.is_absolute():
        cfg_path = Path(__file__).resolve().parent / config_arg
    with open(cfg_path, 'r') as f:
        return json.load(f), cfg_path


def parse_command(argv):
    """Parse final-submission and legacy command styles.

    New style:
        app2_standardized.py yield_control configs/track_policy_*.json
        app2_standardized.py trajectory_planner baseline configs/track_trajectory_*.json
        app2_standardized.py trajectory_planner llm configs/track_trajectory_*.json
        app2_standardized.py trajectory_planner lookahead configs/track_trajectory_*.json
        app2_standardized.py trajectory_planner compare_all configs/track_trajectory_*.json

    Legacy style is still accepted:
        app2_standardized.py landing_pad configs/*.json
    """
    track_name = None
    trajectory_mode = "baseline"
    env_type = None
    scenario_config = None
    config_path = None
    verbose_mode = True

    args = list(argv[1:])
    if not args:
        return track_name, trajectory_mode, env_type, scenario_config, config_path, verbose_mode

    first = args[0]
    if first in TRACK_ALIASES:
        track_name = TRACK_ALIASES[first]
        env_type = "landing_pad"
        rest = args[1:]
        if track_name == "trajectory_planner":
            if rest and rest[0] in TRAJECTORY_MODES:
                trajectory_mode = rest[0]
                rest = rest[1:]
            elif rest and rest[0].endswith(".json"):
                trajectory_mode = "baseline"
            else:
                raise ValueError(
                    "trajectory_planner requires a mode "
                    "(baseline, llm, lookahead, compare_all) and a config path."
                )
        if rest and rest[0].endswith(".json"):
            scenario_config, config_path = _load_config(rest[0])
            verbose_mode = scenario_config.get("verbose", True)
        if rest and "--verbose" in rest:
            verbose_mode = True
        if rest and "--quiet" in rest:
            verbose_mode = False
        return track_name, trajectory_mode, env_type, scenario_config, config_path, verbose_mode

    # Legacy environment-first style.
    env_type = first
    if len(args) > 1:
        arg2 = args[1]
        if arg2.endswith(".json"):
            scenario_config, config_path = _load_config(arg2)
            env_type = scenario_config.get("env_type", env_type)
            verbose_mode = scenario_config.get("verbose", True)
        else:
            verbose_mode = (arg2 == "--verbose")
    if len(args) > 2:
        verbose_mode = (args[2] == "--verbose")
    return track_name, trajectory_mode, env_type, scenario_config, config_path, verbose_mode


def get_input(prompt, default, type_cast=str):
    while True:
        user_input = input(f"{prompt} (default: {default}): ")
        if not user_input:
            return default
        try:
            return type_cast(user_input)
        except ValueError:
            print(f"Invalid input! Please enter a valid {type_cast.__name__}.")

def _impc_logs_dirs():
    root_dir = Path(__file__).resolve().parents[3]
    anim_dir = root_dir / 'logs' / 'Social-IMPC-DR' / 'animations'
    traj_dir = root_dir / 'logs' / 'Social-IMPC-DR' / 'trajectories'
    anim_dir.mkdir(parents=True, exist_ok=True)
    traj_dir.mkdir(parents=True, exist_ok=True)
    return anim_dir, traj_dir

def save_gif_standardized(agent_list, r_min, filename=None, fps=5, num_moving_agents=None, scenario_type='impc', agent_summary='default', frame_log=None):
    """Save animation as GIF file using standardized environment configuration."""
    anim_dir, _ = _impc_logs_dirs()
    if filename is None:
        filename = anim_dir / f"{scenario_type}_{agent_summary}agents.gif"
    else:
        filename = anim_dir / filename

    # Use standardized plot creation
    fig, ax = StandardizedEnvironment.create_standard_plot(scenario_type, show_obstacles=True)
    
    # Build positions per frame from agent_list
    max_frames = max(len(a.position) for a in agent_list) if agent_list else 0

    # Use num_moving_agents to separate dynamic vs obstacle agents
    n_moving = num_moving_agents if num_moving_agents is not None else len(agent_list)
    dynamic_indices = list(range(n_moving))
    obstacle_indices = list(range(n_moving, len(agent_list)))

    # Use standardized colors
    colors = StandardizedEnvironment.AGENT_COLORS

    # Phase 6: per-drone home pads (drawn only when uav.home_pad differs
    # from the main pad, i.e. round-trip scenarios with explicit return
    # points). Static patches added once before animation starts.
    if scenario_type == 'landing_pad':
        _main_pad = StandardizedEnvironment.LANDING_PAD_CENTER
        _home_radius = StandardizedEnvironment.LANDING_PAD_RADIUS * 0.6
        for i in range(n_moving):
            home = getattr(agent_list[i], 'home_pad', None)
            if home is None:
                continue
            if np.linalg.norm(np.asarray(home) - np.asarray(_main_pad)) < 1e-3:
                continue
            c = colors[i % len(colors)]
            ax.add_patch(plt.Circle(home, radius=_home_radius,
                                    facecolor=c, edgecolor='black',
                                    linewidth=1, alpha=0.25, zorder=1))
            ax.add_patch(plt.Circle(home, radius=_home_radius,
                                    facecolor='none', edgecolor=c,
                                    linewidth=2, linestyle='--', zorder=2))
            ax.text(home[0], home[1] - _home_radius - 0.15,
                    f'H{i}', fontsize=8, ha='center', va='top',
                    color=c, fontweight='bold', zorder=2)

    dyn_scatter = ax.scatter([], [], c=[], s=200, edgecolors='black', linewidths=1, label='Agent')
    obs_scatter = ax.scatter([], [], c='gray', s=200, edgecolors='black', linewidths=1, label='Obstacle')

    # Goals as green stars — only for moving agents
    for i in range(n_moving):
        a = agent_list[i]
        gp = getattr(a, 'goal', None)
        if gp is not None and len(gp) == 2:
            ax.plot(gp[0], gp[1], '*', color='green', markersize=12)

    # Create standardized legend — only moving agents
    legend_handles, legend_labels = StandardizedEnvironment.create_standard_legend(n_moving)
    ax.legend(legend_handles, legend_labels,
              loc='center left', bbox_to_anchor=(1.01, 0.5), fontsize=12, borderaxespad=0., markerscale=1.2)
    plt.tight_layout()
    plt.subplots_adjust(right=0.8)

    # Abbreviated label maps for compact display
    _CARGO  = {'organ': 'org', 'blood_product': 'bld', 'medication': 'med', 'equipment': 'eqp'}
    _ACUITY = {'critical': 'crit', 'urgent': 'urg', 'routine': 'rtn'}

    # Per-drone text labels (one per moving drone)
    label_texts = []
    for k, i in enumerate(dynamic_indices):
        c = colors[i % len(colors)]
        txt = ax.text(0, 0, '', fontsize=6.5, ha='center', va='bottom',
                      color='black',
                      bbox=dict(boxstyle='round,pad=0.2', facecolor='white',
                                alpha=0.80, edgecolor=c, linewidth=0.8),
                      zorder=10, clip_on=False)
        label_texts.append(txt)

    def frame_positions(frame):
        pos = []
        for a in agent_list:
            if frame < len(a.position):
                pos.append(a.position[frame])
            else:
                pos.append(a.position[-1])
        return pos

    def animate(frame):
        pos = frame_positions(frame)
        # Obstacles
        obs_pos = [pos[i] for i in obstacle_indices] if obstacle_indices else []
        if obs_pos:
            obs_scatter.set_offsets(np.array(obs_pos).reshape(-1, 2))
        else:
            obs_scatter.set_offsets(np.empty((0, 2)))
        # Dynamic agents
        dyn_pos = [pos[i] for i in dynamic_indices] if dynamic_indices else []
        if dyn_pos:
            dyn_colors = [colors[i % len(colors)] for i in range(len(dyn_pos))]
            dyn_scatter.set_offsets(np.array(dyn_pos).reshape(-1, 2))
            dyn_scatter.set_color(dyn_colors)
        else:
            dyn_scatter.set_offsets(np.empty((0, 2)))
            dyn_scatter.set_color([])

        # Per-drone info labels
        if frame_log is not None:
            fi = min(frame, len(frame_log) - 1)
            fl = frame_log[fi]
            allowed     = fl.get('allowed')
            yielding_set = fl.get('yielding', set())
            scores      = fl.get('scores', {})
        else:
            allowed, yielding_set, scores = None, set(), {}

        for k, i in enumerate(dynamic_indices):
            a = agent_list[i]
            p = pos[i]
            tte = (a.tte_history[min(frame, len(a.tte_history) - 1)]
                   if hasattr(a, 'tte_history') and a.tte_history else a.time_to_expiry)
            cargo  = _CARGO.get(getattr(a, 'cargo_type', ''), getattr(a, 'cargo_type', '')[:3])
            acuity = _ACUITY.get(getattr(a, 'patient_acuity', ''), getattr(a, 'patient_acuity', '')[:3])
            score  = scores.get(i)
            if allowed == i:
                status = ' \u25b6'   # filled triangle = landing
            elif i in yielding_set:
                status = ' \u23f8'   # pause = orbiting/waiting
            else:
                status = ''
            line1 = f'D{i}: {cargo}|{acuity}'
            line2 = f'tte:{tte:.0f}'
            if score is not None:
                line2 += f' s:{score:.2f}'
            line2 += status
            label_texts[k].set_position((p[0], p[1] + 0.28))
            label_texts[k].set_text(f'{line1}\n{line2}')

        return [dyn_scatter, obs_scatter] + label_texts

    anim = FuncAnimation(fig, animate, frames=max_frames, 
                        interval=StandardizedEnvironment.ANIMATION_INTERVAL, blit=True)
    anim.save(str(filename), writer='pillow', fps=fps)
    print(f"GIF animation saved as {filename}")
    plt.close(fig)

def generate_animation_standardized(agent_list, r_min, filename=None, num_moving_agents=None, scenario_type='impc', agent_summary=None, frame_log=None):
    """Generate animation using standardized environment configuration."""
    if agent_summary is None:
        agent_summary = f"{len(agent_list)}"
    save_gif_standardized(agent_list, r_min, filename=filename, fps=StandardizedEnvironment.ANIMATION_FPS,
                         num_moving_agents=num_moving_agents,
                         scenario_type=scenario_type, agent_summary=agent_summary, frame_log=frame_log)

def _generate_animation_standardized_unused(agent_list, r_min, filename=None, num_moving_agents=None, scenario_type='impc', agent_summary=None):
    """(Unused) Old frame-by-frame capture path kept for reference."""
    frames = []
    
    # Use standardized plot creation
    fig, ax = StandardizedEnvironment.create_standard_plot(scenario_type, show_obstacles=True)
    
    # Use standardized colors
    colors = StandardizedEnvironment.AGENT_COLORS
    
    for step in range(len(agent_list[0].position)):
        ax.clear()
        
        # Reset plot with standardized settings
        ax.set_xlim(StandardizedEnvironment.GRID_X_MIN, StandardizedEnvironment.GRID_X_MAX)
        ax.set_ylim(StandardizedEnvironment.GRID_Y_MIN, StandardizedEnvironment.GRID_Y_MAX)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        
        # Add obstacles
        if scenario_type == 'doorway':
            obstacles = StandardizedEnvironment.get_doorway_obstacles()
        elif scenario_type == 'hallway':
            obstacles = StandardizedEnvironment.get_hallway_obstacles()
        elif scenario_type == 'intersection':
            obstacles = StandardizedEnvironment.get_intersection_obstacles()
        elif scenario_type == 'landing_pad':
            obstacles = StandardizedEnvironment.get_landing_pad_obstacles()
        else:
            obstacles = []
        
        for obs in obstacles:
            circle = Circle(obs, radius=StandardizedEnvironment.DEFAULT_AGENT_RADIUS, 
                          facecolor='gray', edgecolor='black', alpha=0.7)
            ax.add_patch(circle)
        
        # Draw landing pad marker
        if scenario_type == 'landing_pad':
            pad = StandardizedEnvironment.LANDING_PAD_CENTER
            pad_circle = Circle(pad, radius=StandardizedEnvironment.LANDING_PAD_RADIUS,
                              facecolor='yellow', edgecolor='red', linewidth=2, alpha=0.4, zorder=1)
            ax.add_patch(pad_circle)
            ax.plot(pad[0], pad[1], 'P', color='red', markersize=15, zorder=2)

            # Phase 6: per-drone home pads (drawn only when uav.home_pad
            # differs from the main pad, i.e. round-trip scenarios).
            home_pad_radius = StandardizedEnvironment.LANDING_PAD_RADIUS * 0.6
            for i, ag in enumerate(agent_list[:num_moving_agents or len(agent_list)]):
                home = getattr(ag, 'home_pad', None)
                if home is None:
                    continue
                if np.linalg.norm(np.asarray(home) - np.asarray(pad)) < 1e-3:
                    continue  # same as main pad -- skip
                color = colors[i % len(colors)]
                ax.add_patch(Circle(home, radius=home_pad_radius,
                                    facecolor=color, edgecolor='black',
                                    linewidth=1, alpha=0.25, zorder=1))
                ax.add_patch(Circle(home, radius=home_pad_radius,
                                    facecolor='none', edgecolor=color,
                                    linewidth=2, linestyle='--', zorder=2))
        
        # Only draw moving agents (skip stationary wall agents)
        n_moving = num_moving_agents if num_moving_agents is not None else len(agent_list)
        for i in range(n_moving):
            agent = agent_list[i]
            color = colors[i % len(colors)]
            
            # Use last known position if step is out of bounds
            pos_index = min(step, len(agent.position) - 1)
            pos = agent.position[pos_index]
            
            # Draw trajectory as a dotted line
            if pos_index > 0:
                past_positions = np.array(agent.position[:pos_index+1])
                ax.plot(past_positions[:, 0], past_positions[:, 1], linestyle="dotted", color=color, linewidth=2)
            
            # Draw solid line for completed part of the trajectory
            if pos_index > 5:
                completed_positions = np.array(agent.position[max(0, pos_index-5):pos_index+1])
                ax.plot(completed_positions[:, 0], completed_positions[:, 1], linestyle="solid", color=color, linewidth=2)

            # Draw drone as a circle using standardized radius
            circle = Circle(pos, radius=StandardizedEnvironment.DEFAULT_AGENT_RADIUS, 
                          edgecolor='black', facecolor=color, zorder=3)
            ax.add_patch(circle)

            # Mark start position with a square
            ax.scatter(agent.position[0][0], agent.position[0][1], marker='s', s=200, 
                      edgecolor='black', facecolor=color)

            # Mark target with a star
            ax.scatter(agent.target[0], agent.target[1], marker='*', s=300, 
                      edgecolor='black', facecolor=color)
        
        # Add legend showing only moving agents
        legend_handles, legend_labels = StandardizedEnvironment.create_standard_legend(n_moving)
        ax.legend(legend_handles, legend_labels,
                  loc='center left', bbox_to_anchor=(1.01, 0.5), fontsize=10, borderaxespad=0., markerscale=1.0)
        
        fig.canvas.draw()
        image = np.array(fig.canvas.renderer.buffer_rgba())[:, :, :3]
        frames.append(image)

    if agent_summary is None:
        agent_summary = f"{len(agent_list)}"

    save_gif_standardized(agent_list, r_min, filename=None, fps=StandardizedEnvironment.ANIMATION_FPS,
                         num_moving_agents=num_moving_agents,
                         scenario_type=scenario_type, agent_summary=agent_summary)
    plt.close(fig)

def setup_standardized_scenario(env_type):
    """Set up scenario using standardized environment configuration."""
    print(f"Setting up {env_type.capitalize()} Environment using standardized configuration...")
    
    if env_type == 'doorway':
        obstacles = StandardizedEnvironment.get_doorway_obstacles()
    elif env_type == 'hallway':
        obstacles = StandardizedEnvironment.get_hallway_obstacles()
    elif env_type == 'intersection':
        obstacles = StandardizedEnvironment.get_intersection_obstacles()
    elif env_type == 'landing_pad':
        obstacles = StandardizedEnvironment.get_landing_pad_obstacles()
    else:
        obstacles = []
    
    ini_x_obstacles = obstacles
    ini_v_obstacles = [np.zeros(2) for _ in ini_x_obstacles]
    # Stationary agents have their target set to their start position
    target_obstacles = ini_x_obstacles
    
    return ini_x_obstacles, ini_v_obstacles, target_obstacles

def main():
    track_name, trajectory_mode, env_type, scenario_config, config_path, verbose_mode = parse_command(sys.argv)
    if track_name in ("yield_control", "trajectory_planner"):
        env_type = "landing_pad"

    # Use standardized scenario setup
    obstacle_agents_x, obstacle_agents_v, obstacle_agents_target = setup_standardized_scenario(env_type)
    
    if scenario_config:
        # ---- Config-file mode: use parameters from file, skip all prompts ----
        num_moving_drones = scenario_config['num_moving_drones']
        min_radius = scenario_config['min_radius']
        wall_collision_multiplier = scenario_config['wall_collision_multiplier']
        epsilon = scenario_config['epsilon']
        step_size = scenario_config['step_size']
        k_value = scenario_config['k_value']
        max_steps = scenario_config['max_steps']

        # Read drone positions from config
        drones = scenario_config['drones']
        ini_x_moving = [np.array(d['start']) for d in drones]
        target_moving = [np.array(d['goal']) for d in drones]
        ini_v_moving = [np.zeros(2) for _ in range(num_moving_drones)]

        # Final submission branch: command selects the track first. Legacy
        # config-only runs still infer the track from the JSON fields.
        policy_recipe = scenario_config.get('policy') if env_type == 'landing_pad' else None
        use_planner = scenario_config.get('use_trajectory_planner', False)
        if track_name == "yield_control":
            if policy_recipe is None:
                raise ValueError("yield_control requires a config with a `policy` block.")
            use_planner = False
        elif track_name == "trajectory_planner":
            policy_recipe = None
            use_planner = True
        if policy_recipe is not None and use_planner:
            raise ValueError("Config must choose either `policy` or `use_trajectory_planner`, not both.")

        # Cargo configs -- read directly from each drone entry when a track
        # needs priority/expiry/acuity metadata.
        cargo_configs = None
        uses_policy_cargo = False
        if policy_recipe is not None:
            selector_name = policy_recipe.get('selector', 'closest_first')
            negotiators = policy_recipe.get('negotiators') or []
            uses_policy_cargo = (
                selector_name == 'priority'
                or 'expiry_guard' in negotiators
                or 'eta_switch' in negotiators
                or 'llm_negotiator' in negotiators
            )
        if env_type == 'landing_pad' and (
            use_planner
            or scenario_config.get('use_priority', False)
            or uses_policy_cargo
        ):
            cargo_configs = [
                {
                    'cargo_type': d['cargo_type'],
                    'time_to_expiry': d['time_to_expiry'],
                    'patient_acuity': d['patient_acuity']
                }
                for d in drones
            ]

        round_trip_params = None
        if use_planner and cargo_configs is not None:
            # Per-drone return points: explicit `return_point` field if
            # present, else fall back to the drone's start position. Used
            # for the OUTBOUND leg of every scenario (n_trips==1 still flies
            # home; n_trips>=2 keeps shuttling).
            return_points = [
                np.array(d.get('return_point', d['start']), dtype=float)
                for d in drones
            ]
            round_trip_params = {
                'use_trajectory_planner': True,
                'return_points':  return_points,
                'n_trips':        scenario_config.get('n_trips',      1),
                'unload_steps':   scenario_config.get('unload_steps', 5),
                'max_speed':      scenario_config.get('max_speed',      1.0),
                'min_separation': scenario_config.get('min_separation', 1.0),
                'safe_distance':  scenario_config.get('safe_distance',  1.2),
                'nominal_speed':  scenario_config.get('nominal_speed',  0.1),
            }
            if scenario_config.get('use_llm_advisor', False):
                round_trip_params['llm_advisor'] = {
                    'enabled': True,
                    'mode': scenario_config.get('llm_mode', 'explain'),
                    'cache_steps': scenario_config.get('llm_cache_steps', 25),
                    'model': scenario_config.get('llm_model'),
                }

        if policy_recipe is not None:
            if policy_recipe.get('lifecycle') == 'round_trip':
                policy_recipe = dict(policy_recipe)
                policy_recipe['return_points'] = [
                    np.array(d.get('return_point', d['start']), dtype=float)
                    for d in drones
                ]
            print(
                f"[Config mode] env={env_type}, drones={num_moving_drones}, "
                f"track=yield_control, selector={policy_recipe.get('selector')}, "
                f"yielder={policy_recipe.get('yielder')}, "
                f"lifecycle={policy_recipe.get('lifecycle')}, "
                f"negotiators={policy_recipe.get('negotiators') or []}"
            )
        else:
            print(f"[Config mode] env={env_type}, drones={num_moving_drones}, "
                  f"track=trajectory_planner, mode={trajectory_mode}, planner={use_planner}, "
                  f"llm_advisor={scenario_config.get('use_llm_advisor', False)}")
        for i, d in enumerate(drones):
            print(f"  Drone {i}: start={d['start']}, goal={d['goal']}")

    else:
        # ---- Interactive mode: original prompt-based flow ----

        # Get parameters for the moving drones
        num_moving_drones = get_input("Enter number of moving drones", 2, int)

        # Get simulation parameters from user
        min_radius = get_input("Enter minimum distance between drones", StandardizedEnvironment.DEFAULT_COLLISION_DISTANCE, float)

        # Add configurable wall collision distance
        wall_collision_multiplier = get_input("Enter wall collision distance multiplier (1.5-3.0 recommended)", 2.0, float)

        epsilon = get_input("Enter epsilon value", 0.1, float)
        step_size = get_input("Enter step size", 0.1, float)
        k_value = get_input("Enter k value", 10, int)
        max_steps = get_input("Enter maximum number of steps", 100, int)

        print("\nConfigure moving drones:")

        # Print environment-specific instructions using standardized coordinates
        if env_type == 'doorway':
            print("\nDoorway Configuration:")
            print("- The doorway has a vertical wall at x=0 with a gap between y=-2 and y=2")
            print("- X coordinates should be between -5 and 5")
            print("- Y coordinates should be between -7 and 7")
        elif env_type == 'hallway':
            print("\nHallway Configuration:")
            print("- The hallway has walls at y=-2 and y=2")
            print("- Robots should stay between y=-1.5 and y=1.5 (middle of hallway)")
            print("- X coordinates should be between -5 and 5")
        elif env_type == 'intersection':
            print("\nIntersection Configuration:")
            print("- The intersection has corridors with center at (0, 0)")
            print("- Corridor width extends from -2 to 2 in both directions")
            print("- X and Y coordinates should be between -5 and 5")
        elif env_type == 'landing_pad':
            print("\nLanding Pad Configuration:")
            print("- Single landing pad at (0, 0) — all drones share this goal")
            print("- Only one drone can occupy the pad at a time")
            print("- Drones should start from different approach directions")
            print("- X and Y coordinates should be between -4 and 4")

        # Get drone positions using standardized positions
        ini_x_moving = []
        target_moving = []

        # Get standardized default positions
        standard_positions = StandardizedEnvironment.get_standard_agent_positions(env_type, num_moving_drones)

        for i in range(num_moving_drones):
            print(f"\n--- Agent {i+1} Parameters ---")

            # Get default values for this drone
            defaults = standard_positions[i] if i < len(standard_positions) else standard_positions[0]

            # Get start position
            start_x = get_input(f"Start X position (default: {defaults['start'][0]})", defaults['start'][0], float)
            start_y = get_input(f"Start Y position (default: {defaults['start'][1]})", defaults['start'][1], float)

            # Get goal position  
            goal_x = get_input(f"Goal X position (default: {defaults['goal'][0]})", defaults['goal'][0], float)
            goal_y = get_input(f"Goal Y position (default: {defaults['goal'][1]})", defaults['goal'][1], float)

            # Store positions
            ini_x_moving.append(np.array([start_x, start_y]))
            target_moving.append(np.array([goal_x, goal_y]))

            print(f"Agent {i+1} configured: Start=({start_x}, {start_y}), Goal=({goal_x}, {goal_y})")

        ini_v_moving = [np.zeros(2) for _ in range(num_moving_drones)]

        # --- Cargo priority configuration (landing_pad only) ---
        # Track 2 interactive mode does not build planner/round-trip params;
        # use config-file mode (a `track_trajectory_*.json`) for actual runs.
        cargo_configs = None
        policy_recipe = None
        round_trip_params = None
        if env_type == 'landing_pad':
            default_cfg_path = Path(__file__).resolve().parent / 'configs' / 'track_trajectory_oneway.json'
            with open(default_cfg_path, 'r') as f:
                default_cfg = json.load(f)
            default_cargos = [
                {'cargo_type': d['cargo_type'], 'time_to_expiry': d['time_to_expiry'], 'patient_acuity': d['patient_acuity']}
                for d in default_cfg['drones']
            ]

            print("\n--- Cargo Priority Configuration ---")
            print("Cargo types: organ, blood_product, medication, equipment")
            print("Patient acuity: critical, urgent, routine")
            use_priority = get_input("Enable priority-based yielding? (y/n)", 'y', str)
            if use_priority.lower() == 'y':
                cargo_configs = []
                for i in range(num_moving_drones):
                    dfl = default_cargos[i] if i < len(default_cargos) else default_cargos[-1]
                    print(f"\n  Drone {i} cargo:")
                    ct = get_input(f"    Cargo type", dfl['cargo_type'], str)
                    te = get_input(f"    Time to expiry (steps)", dfl['time_to_expiry'], float)
                    pa = get_input(f"    Patient acuity", dfl['patient_acuity'], str)
                    cargo_configs.append({'cargo_type': ct, 'time_to_expiry': te, 'patient_acuity': pa})

    # --- Combine moving and stationary agents ---
    ini_x = ini_x_moving + obstacle_agents_x
    ini_v = ini_v_moving + obstacle_agents_v
    target = target_moving + obstacle_agents_target
    num_drones = len(ini_x)
    
    print("\nStarting simulation...")
    result, agent_list, completion_step, frame_log = PLAN(num_drones, ini_x, ini_v, target, min_radius, epsilon, step_size, k_value, max_steps, num_moving_drones=num_moving_drones, wall_collision_multiplier=wall_collision_multiplier, verbose=verbose_mode, env_type=env_type, cargo_configs=cargo_configs, policy_recipe=policy_recipe, round_trip_params=round_trip_params, trajectory_mode=trajectory_mode)
    
    # Save completion step for Flow Rate calculation
    with open("completion_step.txt", "w") as f:
        f.write(str(completion_step))
    
    if result:
        print("\nSimulation completed successfully!")
        if round_trip_params is not None:
            controller_tag = 'planner'
        elif policy_recipe is not None:
            controller_tag = (
                f"{policy_recipe.get('lifecycle','one_way')}"
                f"_{policy_recipe.get('selector','closest_first')}"
                f"_{policy_recipe.get('yielder','freeze')}"
            )
        else:
            controller_tag = 'base'
        # Include config file stem so different test scenarios don't overwrite each other
        if scenario_config and scenario_config.get('test_name'):
            gif_filename = f"{scenario_config['test_name']}.gif"
        elif config_path is not None:
            config_stem = config_path.stem
            gif_filename = f"{config_stem}.gif"
        else:
            gif_filename = f"{env_type}_{num_moving_drones}agents_{controller_tag}.gif"
        agent_summary = f"{num_moving_drones}_{controller_tag}"
        generate_animation_standardized(agent_list, min_radius, filename=gif_filename, num_moving_agents=num_moving_drones, scenario_type=env_type, agent_summary=agent_summary, frame_log=frame_log)
    else:
        print("\nSimulation failed to find a solution.")

if __name__ == "__main__":
    main() 

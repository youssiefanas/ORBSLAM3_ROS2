#!/usr/bin/env python3
import argparse
import sys
import os
import shutil
import tempfile
import subprocess
import numpy as np
from scipy import signal
import csv

# Try to import evo. If fail, re-exec with known pipx path or warn.
try:
    import evo
    from evo.core import trajectory, sync, metrics, geometry
    from evo.tools import file_interface
    from evo.core import lie_algebra as lie
    import copy
except ImportError:
    # Check if we can find the pipx environment
    pipx_python = os.path.expanduser("~/.local/share/pipx/venvs/evo/bin/python")
    if os.path.exists(pipx_python) and sys.executable != pipx_python:
        print(f"Evo not found in current environment. Re-executing with identified pipx python: {pipx_python}")
        os.execv(pipx_python, [pipx_python] + sys.argv)
    else:
        print("Error: 'evo' module not found and could not auto-detect pipx environment.")
        print("Please run this script using the python interpreter where evo is installed.")
        print("Example: ~/.local/share/pipx/venvs/evo/bin/python evaluate_session.py ...")
        sys.exit(1)

def run_command(cmd):
    """Run a shell command."""
    print(f"\n> {' '.join(cmd)}")
    subprocess.check_call(cmd)

def calculate_speed_profile(timestamps, positions, dt=0.1):
    duration = timestamps[-1] - timestamps[0]
    num_samples = int(duration / dt)
    if num_samples <= 0: return np.array([]), np.array([])
    uniform_ts = np.linspace(timestamps[0], timestamps[-1], num_samples)
    interp_pos = np.zeros((num_samples, 3))
    for i in range(3):
        interp_pos[:, i] = np.interp(uniform_ts, timestamps, positions[:, i])
    velocities = np.diff(interp_pos, axis=0) / dt
    speeds = np.linalg.norm(velocities, axis=1)
    return uniform_ts[:-1], speeds

def find_time_offset_correlation(traj_ref, traj_est, dt=0.1):
    print("\n--- Calculating Automatic Time Synchronization (Velocity Correlation) ---")
    
    # Extract arrays
    ref_ts = traj_ref.timestamps
    ref_pos = traj_ref.positions_xyz
    est_ts = traj_est.timestamps
    est_pos = traj_est.positions_xyz
    
    ref_start = ref_ts[0]
    est_start = est_ts[0]
    
    ref_ts_local = ref_ts - ref_start
    est_ts_local = est_ts - est_start
    
    ts_ref_u, speed_ref = calculate_speed_profile(ref_ts_local, ref_pos, dt)
    ts_est_u, speed_est = calculate_speed_profile(est_ts_local, est_pos, dt)
    
    if len(speed_ref) == 0 or len(speed_est) == 0:
        return 0.0

    speed_ref_n = (speed_ref - np.mean(speed_ref)) / (np.std(speed_ref) + 1e-9)
    speed_est_n = (speed_est - np.mean(speed_est)) / (np.std(speed_est) + 1e-9)
    
    swapped = False
    if len(speed_ref_n) < len(speed_est_n):
        swapped = True
        corr = signal.correlate(speed_est_n, speed_ref_n, mode='full')
        lags = signal.correlation_lags(len(speed_est_n), len(speed_ref_n), mode='full')
    else:
        corr = signal.correlate(speed_ref_n, speed_est_n, mode='full')
        lags = signal.correlation_lags(len(speed_ref_n), len(speed_est_n), mode='full')
    
    best_idx = np.argmax(corr)
    time_shift = lags[best_idx] * dt
    if swapped: time_shift = -time_shift
    
    total_offset = time_shift - est_start + ref_start
    print(f"Found best match offset: {total_offset:.2f}s")
    return total_offset

def stretch_trajectory(traj, start_time, end_time):
    print(f"\n--- Stretching Trajectory Time ---")
    n_poses = len(traj.timestamps)
    new_ts = np.linspace(start_time, end_time, n_poses)
    
    traj_stretched = trajectory.PoseTrajectory3D(
        positions_xyz=traj.positions_xyz,
        orientations_quat_wxyz=traj.orientations_quat_wxyz,
        timestamps=new_ts,
        meta=traj.meta
    )
    return traj_stretched

def save_stats_to_csv(stats, label, filepath):
    print(f"\n{label} Statistics:")
    
    # Sort keys for consistent output
    ordered_keys = ["max", "mean", "median", "min", "rmse", "sse", "std"]
    
    with open(filepath, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Statistic", "Value"])
        
        for key in ordered_keys:
            if key in stats:
                val = stats[key]
                print(f"{key:>10} {val:.6f}")
                writer.writerow([key, val])
        
        # Write any others not in the ordered list
        for key, val in stats.items():
            if key not in ordered_keys:
                print(f"{key:>10} {val:.6f}")
                writer.writerow([key, val])
                
    print(f"Saved stats to {filepath}")

def load_trajectory(file_path):
    """
    Load trajectory from file.
    Supports .csv (EuRoC or generic) and .txt (TUM).
    """
    ext = os.path.splitext(file_path)[1].lower()
    
    if ext == '.csv':
        try:
            print(f"Attempting to load {file_path} as EuRoC CSV...")
            return file_interface.read_euroc_csv_trajectory(file_path)
        except Exception as e:
            print(f"EuRoC load failed: {e}. Trying generic CSV...")
            
        # Generic CSV Loader
        # Expects either header with specific names or standard order
        # We'll assume a robust parser that looks for t,x,y,z,qx,qy,qz,qw columns
        try:
            timestamps = []
            poses = []
            
            with open(file_path, 'r') as f:
                reader = csv.reader(f)
                header = next(reader, None)
                
                # Simple heuristic: inspect header or first row
                # If header exists, look for column indices
                t_idx, x_idx, y_idx, z_idx = 0, 1, 2, 3
                qx_idx, qy_idx, qz_idx, qw_idx = 4, 5, 6, 7
                
                start_row = []
                if header:
                    # Check if header is actually data (all numbers)
                    try:
                        [float(x) for x in header]
                        # It's data, treat as first row
                        start_row = header
                        header = None
                    except ValueError:
                        # It is a header
                        pass
                
                if header:
                    # Map columns if names are present
                    lower_header = [h.strip().lower() for h in header]
                    # Map standard names
                    # TODO: Make this more robust if needed
                    pass 
                
                # Re-read to iterate properly
                f.seek(0)
                reader = csv.reader(f)
                if header:
                    next(reader) # skip header
                
                xyz = []
                quat = []
                times = []
                
                for row in reader:
                    if not row or row[0].startswith('#'): continue
                    if len(row) < 8: continue
                    
                    try:
                        # Assuming TUM order: timestamp x y z qx qy qz qw
                        ts = float(row[0])
                        # Handle nano-seconds if timestamp is huge (e.g. > 1e18)
                        if ts > 1e16: ts /= 1e9
                        
                        p = [float(row[1]), float(row[2]), float(row[3])]
                        q = [float(row[7]), float(row[4]), float(row[5]), float(row[6])] # wxyz for Evo? No, Evo uses wxyz internally but let's check input
                        # TUM format is qx qy qz qw. Evo PoseTrajectory3D expects orientations_quat_wxyz.
                        # Wait, file_interface.read_tum reads x y z qx qy qz qw.
                        # Let's align with TUM standard input: x y z qx qy qz qw
                        
                        times.append(ts)
                        xyz.append(p)
                        # Store as wxyz for numpy array creation below
                        quat.append([float(row[7]), float(row[4]), float(row[5]), float(row[6])]) 
                        
                    except ValueError:
                        continue
                        
                if not times:
                    raise ValueError("No valid data found in CSV")
                    
                return trajectory.PoseTrajectory3D(
                    positions_xyz=np.array(xyz),
                    orientations_quat_wxyz=np.array(quat),
                    timestamps=np.array(times)
                )

        except Exception as e:
            print(f"Generic CSV load failed: {e}")
            raise IOError(f"Could not load CSV file {file_path}")

    else:
        # Default to TUM
        return file_interface.read_tum_trajectory_file(file_path)

def main():
    parser = argparse.ArgumentParser(description="Evo-based Analysis Script")
    parser.add_argument("ref_file", help="Path to Reference/Ground Truth trajectory (TUM format or CSV)")
    parser.add_argument("est_file", help="Path to Estimated trajectory (TUM format)")
    parser.add_argument("--out_dir", help="Directory to save plots", default=".")
    
    # Alignment Options
    parser.add_argument("--align", action="store_true", help="Perform Sim3 alignment (Optimize Scale, Rot, Trans)")
    parser.add_argument("--align_se3", action="store_true", help="Perform SE3 alignment (Optimize Rot, Trans ONLY). Keeps scale fixed.")
    
    # Sync/Time Options
    parser.add_argument("--t_max_diff", type=float, default=0.1, help="Max timestamp difference for synchronization (default: 0.1)")
    parser.add_argument("--auto_time_sync", action="store_true", help="Automatically sync trajectories by velocity correlation")
    parser.add_argument("--match_duration", action="store_true", help="Force Estimated Trajectory to match Ground Truth duration")
    
    # Scale Options
    parser.add_argument("--scale", type=float, default=1.0, help="Manual scale factor for Estimated Trajectory")
    
    # Transformation Options
    parser.add_argument("--correct_optical_frame", action="store_true", help="Apply Optical (Z-fwd) to ROS (X-fwd) rotation correction (x=z, y=-x, z=-y)")
    
    parser.add_argument("--no_plot", action="store_true", help="Do not open plot windows (save only)")
    
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    
    print(f"Loading files using evo...")
    # UPDATED LOADING LOGIC
    traj_ref = load_trajectory(args.ref_file)
    traj_est = file_interface.read_tum_trajectory_file(args.est_file)
    
    # 0. Frame Correction (Optical -> ROS)
    if args.correct_optical_frame:
        print("Applying Optical->ROS Frame Correction...")
        # Rotation Matrix:
        # [ 0  0  1 ]
        # [-1  0  0 ]
        # [ 0 -1  0 ]
        R_optical_to_ros = np.array([
            [ 0,  0,  1],
            [-1,  0,  0],
            [ 0, -1,  0]
        ])
        
        # Helper to apply rotation to a PyEvo trajectory
        # Construct SE3 with this rotation and zero translation
        T_optical_to_ros = lie.se3(R_optical_to_ros, np.zeros(3))
        traj_est.transform(T_optical_to_ros)
    
    # 1. Manual Scale
    if abs(args.scale - 1.0) > 1e-5:
        print(f"Applying manual scale x{args.scale}")
        traj_est.scale(args.scale)
        
    # 2. Time Operations
    if args.match_duration:
        traj_est = stretch_trajectory(traj_est, traj_ref.timestamps[0], traj_ref.timestamps[-1])
        
        # Adjust t_max_diff if step is large
        step = (traj_est.timestamps[-1] - traj_est.timestamps[0]) / (traj_est.num_poses - 1)
        if args.t_max_diff < step / 2:
            print(f"Automatically increasing t_max_diff to {step/2:.3f}s due to stretching.")
            args.t_max_diff = step / 2
            
    else:
        # Offset Logic
        offset = 0.0
        if args.auto_time_sync:
            offset = find_time_offset_correlation(traj_ref, traj_est)
        else:
            diff = traj_est.timestamps[0] - traj_ref.timestamps[0]
            if abs(diff) > 100.0:
                offset = -diff
                
        if abs(offset) > 1e-5:
            print(f"Applying time offset {offset:.4f}s")
            # Create new trajectory with shifted timestamps
            traj_est = trajectory.PoseTrajectory3D(
                positions_xyz=traj_est.positions_xyz,
                orientations_quat_wxyz=traj_est.orientations_quat_wxyz,
                timestamps=traj_est.timestamps + offset,
                meta=traj_est.meta
            )

    # 3. Association / Sync (Using Evo API as requested)
    print(f"\n--- Synchronizing Trajectories (max_diff={args.t_max_diff}) ---")
    traj_ref_sync, traj_est_sync = sync.associate_trajectories(traj_ref, traj_est, args.t_max_diff)
    print(f"Associated {traj_est_sync.num_poses} poses.")
    
    # 4. Alignment
    
    traj_est_aligned = copy.deepcopy(traj_est)
    sim3_scale = 1.0 # Default scale if not aligned
    
    if args.align or args.align_se3:
        with_scale = args.align and not args.align_se3
        print(f"Aligning... (Sim3 via Evo={with_scale})")
        
        est_xyz = traj_est_sync.positions_xyz.T
        ref_xyz = traj_ref_sync.positions_xyz.T
        
        if est_xyz.shape[1] < 3:
            print("Warning: Not enough points for alignment (needs >= 3). Skipping alignment.")
        else:
            rotation, translation, scale = geometry.umeyama_alignment(est_xyz, ref_xyz, with_scale=with_scale)
            sim3_scale = scale
            
            print(f"Calculated Scale: {scale:.4f}")
            print(f"Calculated Translation: {translation}")
            
            # Apply to FULL trajectory (Corrected: Uncommented to ensure Python stats are correct)
            traj_est_aligned.scale(scale)
            traj_est_aligned.transform(lie.se3(rotation, translation))
        
    # 5. Saving Stats (using Evo Python API) and Plots (using Evo CLI)
    
    # Calculate stats for CSV saving
    # We need to re-associate the ALIGNED trajectory
    traj_ref_sync_final, traj_est_sync_final = sync.associate_trajectories(traj_ref, traj_est_aligned, args.t_max_diff)

    # APE Stats
    ape_metric = metrics.APE(metrics.PoseRelation.translation_part)
    ape_metric.process_data((traj_ref_sync_final, traj_est_sync_final))
    ape_stats = ape_metric.get_all_statistics()
    # ADD SCALE TO STATS
    ape_stats["sim3_scale"] = sim3_scale
    
    save_stats_to_csv(ape_stats, "APE Translation", os.path.join(args.out_dir, "ape_stats.csv"))


    # RPE Stats
    delta = 1
    delta_unit = metrics.Unit.frames
    rpe_metric = metrics.RPE(metrics.PoseRelation.translation_part, delta, delta_unit, all_pairs=False)
    rpe_metric.process_data((traj_ref_sync_final, traj_est_sync_final))
    rpe_stats = rpe_metric.get_all_statistics()
    save_stats_to_csv(rpe_stats, "RPE Translation", os.path.join(args.out_dir, "rpe_stats.csv"))


    print("\n--- Running Evo CLI for Plotting ---")
    with tempfile.TemporaryDirectory() as temp_dir:
        # Save the aligned trajectory
        final_est_file = os.path.join(temp_dir, "est_final.txt")
        file_interface.write_tum_trajectory_file(final_est_file, traj_est_aligned)
        
        # Save the reference trajectory to a temp TUM file (handles CSV input)
        final_ref_file = os.path.join(temp_dir, "ref_final.txt")
        file_interface.write_tum_trajectory_file(final_ref_file, traj_ref)
        
        common_flags = ["--t_max_diff", str(args.t_max_diff)] 
        
        # Traj Plot (Ref vs Est)
        run_command(["evo_traj", "tum", final_ref_file, final_est_file, "--ref", final_ref_file] + common_flags + ["--save_plot", os.path.join(args.out_dir, "trajectory_plot.pdf")] + (["--plot"] if not args.no_plot else []))
        
        # APE (Absolute Pose Error)
        run_command(["evo_ape", "tum", final_ref_file, final_est_file] + common_flags + ["--save_plot", os.path.join(args.out_dir, "ape_plot.pdf")] + (["--plot"] if not args.no_plot else []) + ["--pose_relation", "trans_part"])
        
        # RPE (Relative Pose Error)
        run_command(["evo_rpe", "tum", final_ref_file, final_est_file] + common_flags + ["--save_plot", os.path.join(args.out_dir, "rpe_plot.pdf")] + (["--plot"] if not args.no_plot else []))


if __name__ == "__main__":
    main()

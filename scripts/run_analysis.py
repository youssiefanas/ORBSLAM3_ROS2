#!/usr/bin/python3
import argparse
import os
import subprocess
import sys

def run_command(command):
    print(f"Running: {' '.join(command)}")
    try:
        subprocess.check_call(command)
        print("Success.")
    except subprocess.CalledProcessError as e:
        print(f"Error running command: {e}")
        # Don't exit, try to continue with other steps

def main():
    parser = argparse.ArgumentParser(description="Automate ORB-SLAM3 Analysis (ATE, RPE, Visualization)")
    parser.add_argument("session_dir", help="Path to the session directory (containing GroundTruth.txt and FrameTrajectory.txt)")
    parser.add_argument("--no_viz", action="store_true", help="Skip 3D visualization")
    parser.add_argument("--step", type=int, default=1, help="Step size for visualization (default: 1)")
    parser.add_argument("--align", action="store_true", help="Align estimated trajectory to ground truth")
    args = parser.parse_args()

    session_dir = os.path.abspath(args.session_dir)
    if not os.path.isdir(session_dir):
        print(f"Error: Session directory '{session_dir}' does not exist.")
        sys.exit(1)

    gt_path = os.path.join(session_dir, "GroundTruth.txt")
    
    # Check for FrameTrajectory or KeyFrameTrajectory
    frame_traj_path = os.path.join(session_dir, "FrameTrajectory.txt")
    keyframe_traj_path = os.path.join(session_dir, "KeyFrameTrajectory.txt")
    
    est_path = frame_traj_path if os.path.exists(frame_traj_path) else keyframe_traj_path
    
    if not os.path.exists(gt_path):
        print(f"Error: GroundTruth.txt not found in {session_dir}")
        sys.exit(1)
        
    if not os.path.exists(est_path):
         print(f"Error: Neither FrameTrajectory.txt nor KeyFrameTrajectory.txt found in {session_dir}")
         sys.exit(1)

    scripts_dir = os.path.dirname(os.path.realpath(__file__))
    ate_script = os.path.join(scripts_dir, "evaluate_ate_scale.py")
    rpe_script = os.path.join(scripts_dir, "evaluate_rpe.py")
    plot_script = os.path.join(scripts_dir, "plot_results.py")

    print(f"--- Analyzing Session: {session_dir} ---")
    print(f"Ground Truth: {os.path.basename(gt_path)}")
    print(f"Estimated:    {os.path.basename(est_path)}")
    print("-" * 40)

    # 1. ATE Evaluation
    print("\n[1/3] Calculating Absolute Trajectory Error (ATE)...")
    ate_plot_path = os.path.join(session_dir, "ate.pdf")
    ate_text_path = os.path.join(session_dir, "ate_results.txt")
    
    # Save text output to file
    with open(ate_text_path, "w") as f:
        subprocess.call([sys.executable, ate_script, gt_path, est_path, "--plot", ate_plot_path, "--verbose"], stdout=f)
    print(f"ATE saved to: {ate_plot_path} and {ate_text_path}")
    # Print summary to console
    subprocess.call([sys.executable, ate_script, gt_path, est_path])

    # 2. RPE Evaluation
    print("\n[2/3] Calculating Relative Pose Error (RPE)...")
    rpe_plot_path = os.path.join(session_dir, "rpe.png")
    rpe_text_path = os.path.join(session_dir, "rpe_results.txt")
    
    with open(rpe_text_path, "w") as f:
         subprocess.call([sys.executable, rpe_script, gt_path, est_path, "--fixed_delta", "--plot", rpe_plot_path, "--verbose"], stdout=f)
    print(f"RPE saved to: {rpe_plot_path} and {rpe_text_path}")
    # Print summary to console
    subprocess.call([sys.executable, rpe_script, gt_path, est_path, "--fixed_delta"])

    # 3. Visualization
    print("\n[3/3] Generating 3D Trajectory Plot...")
    traj_plot_path = os.path.join(session_dir, "trajectory_3d.png")
    
    plot_cmd = [sys.executable, plot_script, session_dir, "--step", str(args.step), "--save", traj_plot_path]
    if args.align:
        plot_cmd.append("--align")
    
    if args.no_viz:
        plot_cmd.append("--no_show")
        print("Saving plot without displaying window...")
    else:
        print("Launching visualization window...")
        
    run_command(plot_cmd)

    print("\nAnalysis Complete.")

if __name__ == "__main__":
    main()

# ORB-SLAM3 Analysis Scripts

This directory contains Python scripts for evaluating and visualizing the results of ORB-SLAM3.

## 🚀 Quick Start
To run a full analysis (ATE, RPE, and Visualization) on a session directory:

```bash
python3 run_analysis.py <path_to_session_directory>
```
Example:
```bash
python3 run_analysis.py ../session_2025_12_18_01_06_26
```

## 📜 Scripts Overview

### 1. `run_analysis.py` (Wrapper)
Automates the execution of all other scripts.
- **Input**: Session directory containing `GroundTruth.txt` and `FrameTrajectory.txt`.
- **Output**: 
  - `ate.pdf` (Absolute Trajectory Error plot)
  - `ate_results.txt` (Detailed ATE statistics)
  - `rpe.png` (Relative Pose Error plot)
  - `rpe_results.txt` (Detailed RPE statistics)
  - Opens 3D visualization window.
- **Usage**:
  ```bash
  python3 run_analysis.py <session_dir> [--no_viz] [--step N]
  ```

### 2. `plot_results.py` (Visualization)
Interactive 3D plot of the trajectory, ground truth, and point cloud.
- **Features**: 
  - Plots Camera Frustums (pyramids) to show orientation.
  - Plots Point Cloud map.
  - Plots Estimated vs Ground Truth paths.
  - Can save plot to file.
  - **[New]** Can align estimated trajectory to Ground Truth (Umeyama).
- **Usage**:
  ```bash
  python3 plot_results.py <session_dir> [--step 1] [--save plot.png] [--align] [--no_show] [--verbose]
  ```

### 3. `evaluate_ate_scale.py` (ATE)
Calculates the **Absolute Trajectory Error (ATE)**. This measures the global consistency of the trajectory.
- **Modified**: Compatible with Python 3.
- **Usage**:
  ```bash
  python3 evaluate_ate_scale.py <GroundTruth.txt> <FrameTrajectory.txt> --plot ate.pdf --verbose
  ```

### 4. `evaluate_rpe.py` (RPE)
Calculates the **Relative Pose Error (RPE)**. This measures the local accuracy (drift) of the trajectory over fixed intervals.
- **Modified**: Compatible with Python 3.
- **Usage**:
  ```bash
  python3 evaluate_rpe.py <GroundTruth.txt> <FrameTrajectory.txt> --fixed_delta --plot rpe.png --verbose
  ```

### 5. `associate.py` (Helper)
Helper script used by evaluation scripts to associate timestamps between two trajectory files based on closest time match.

## 📂 Data Format
The scripts expect the following file formats in the session directory:
- **GroundTruth.txt**: TUM format (`timestamp tx ty tz qx qy qz qw`)
- **FrameTrajectory.txt**: TUM format
- **KeyFrameTrajectory.txt**: TUM format
- **PointCloud.txt**: `X Y Z` coordinates (space separated)

## 📊 Using `evo` (Alternative)

For more advanced evaluation, you can use the [evo package](https://github.com/MichaelGrupp/evo).

### 1. Installation
```bash
pip3 install evo --upgrade --no-binary evo
```

### 2. Compare Trajectories (ATE)
Calculate Absolute Trajectory Error (ATE) and align:
```bash
evo_ape tum GroundTruth.txt FrameTrajectory.txt -va --plot --plot_mode=xyz
```
- `-v`: verbose
- `-a`: align (best fit)
- `--plot`: show plot

### 3. Compare Trajectories (RPE)
Calculate Relative Pose Error (RPE):
```bash
evo_rpe tum GroundTruth.txt FrameTrajectory.txt -va --plot --plot_mode=xyz
```

### 4. Plot Trajectories
Draw multiple trajectories:
```bash
evo_traj tum GroundTruth.txt FrameTrajectory.txt --ref=GroundTruth.txt -p --plot_mode=xyz
```

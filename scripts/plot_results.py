import argparse
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import sys

# Add current directory to path to import associate
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
try:
    import associate
except ImportError:
    print("Warning: associate.py not found. Alignment will not work.")
    associate = None

def get_rotation_matrix(qx, qy, qz, qw):
    """
    Convert quaternion to rotation matrix.
    Assumes quaternion is normalized.
    """
    R = np.zeros((3, 3))
    
    sqw = qw**2
    sqx = qx**2
    sqy = qy**2
    sqz = qz**2

    # Invs (inverse square length) is 1 if quaternion is normalized
    invs = 1.0 / (sqx + sqy + sqz + sqw)
    
    m00 = ( sqx - sqy - sqz + sqw)*invs
    m11 = (-sqx + sqy - sqz + sqw)*invs
    m22 = (-sqx - sqy + sqz + sqw)*invs
    
    tmp1 = qx*qy
    tmp2 = qz*qw
    m10 = 2.0 * (tmp1 + tmp2)*invs
    m01 = 2.0 * (tmp1 - tmp2)*invs
    
    tmp1 = qx*qz
    tmp2 = qy*qw
    m20 = 2.0 * (tmp1 - tmp2)*invs
    m02 = 2.0 * (tmp1 + tmp2)*invs
    
    tmp1 = qy*qz
    tmp2 = qx*qw
    m21 = 2.0 * (tmp1 + tmp2)*invs
    m12 = 2.0 * (tmp1 - tmp2)*invs
    
    return np.array([
        [m00, m01, m02],
        [m10, m11, m12],
        [m20, m21, m22]
    ])

def get_quaternion(R):
    """
    Convert rotation matrix to quaternion [x, y, z, w].
    """
    trace = np.trace(R)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 2] + R[2, 0]) / s
            qz = (R[0, 1] + R[1, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
    return np.array([qx, qy, qz, qw])

def align(model, data):
    """Align two trajectories using the method of Horn (closed-form).
    
    Input:
    model -- first trajectory (3xn)
    data -- second trajectory (3xn)
    
    Output:
    rot -- rotation matrix (3x3)
    trans -- translation vector (3x1)
    scale -- scale factor
    """
    np.set_printoptions(precision=3,suppress=True)
    model_zerocentered = model - model.mean(1)
    data_zerocentered = data - data.mean(1)
    
    W = np.zeros( (3,3) )
    for column in range(model.shape[1]):
        W += np.outer(model_zerocentered[:,column],data_zerocentered[:,column])
    U,d,Vh = np.linalg.svd(W.transpose())
    S = np.identity( 3 )
    if(np.linalg.det(U) * np.linalg.det(Vh)<0):
        S[2,2] = -1
    rot = U @ S @ Vh

    rotmodel = rot @ model_zerocentered
    dots = 0.0
    norms = 0.0

    for column in range(data_zerocentered.shape[1]):
        dots += np.dot(data_zerocentered[:,column].transpose(),rotmodel[:,column])
        normi = np.linalg.norm(model_zerocentered[:,column])
        norms += normi*normi

    s = float((dots/norms).item())
    
    trans = data.mean(1) - s * rot @ model.mean(1)
    
    return rot, trans, s

def transform_trajectory(traj, R, t, s):
    """
    Apply transformation T = s * R * p + t to trajectory.
    Updates positions and rotates orientations.
    traj: Nx8 (ts, tx, ty, tz, qx, qy, qz, qw)
    """
    new_traj = np.copy(traj)
    
    # 1. Transform positions
    # positions are at indices 1, 2, 3
    positions = traj[:, 0:3].T # 3xN
    new_positions = (s * R @ positions + t).T # Nx3
    new_traj[:, 0:3] = new_positions
    
    # 2. Transform orientations
    # quaternions are at indices 3, 4, 5, 6 (qx, qy, qz, qw)
    for i in range(len(traj)):
        qi = traj[i, 3:7]
        Ri = get_rotation_matrix(qi[0], qi[1], qi[2], qi[3])
        R_new = R @ Ri
        q_new = get_quaternion(R_new)
        new_traj[i, 3:7] = q_new
        
    return new_traj

def transform_point_cloud(pcd, R, t, s):
    """
    Apply transformation to point cloud Nx3.
    """
    new_pcd = np.copy(pcd).T # 3xN
    new_pcd = (s * R @ new_pcd + t).T
    return new_pcd


def load_trajectory(file_path):
    if not os.path.exists(file_path):
        print(f"File not found: {file_path}")
        return None
    
    data = []
    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith('#'): continue
            parts = line.strip().split()
            if len(parts) >= 8: # timestamp tx ty tz qx qy qz qw
                # Store tx, ty, tz, qx, qy, qz, qw, timestamp
                data.append([float(parts[1]), float(parts[2]), float(parts[3]),
                             float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7]),
                             float(parts[0])])
    return np.array(data)

def load_point_cloud(file_path):
    if not os.path.exists(file_path):
        print(f"File not found: {file_path}")
        return None
    
    data = []
    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith('#'): continue
            parts = line.strip().split()
            if len(parts) >= 3:
                data.append([float(parts[0]), float(parts[1]), float(parts[2])])
    return np.array(data)

def plot_frustums(ax, poses, step=1, scale=0.1, color='k'):
    """
    Plot camera frustums (pyramids) for each pose.
    """
    # Define frustum corners in camera frame (Z-forward, X-right, Y-down)
    w = scale
    h = scale * 0.75 # 4:3 aspect ratio
    z = scale
    
    # Corners: Top-Left, Top-Right, Bottom-Right, Bottom-Left, Center
    # Camera frame: Right(X), Down(Y), Forward(Z)
    corners_cam = np.array([
        [-w, -h, z], # TL
        [ w, -h, z], # TR
        [ w,  h, z], # BR
        [-w,  h, z], # BL
        [ 0,  0, 0]  # Center
    ])

    for i in range(0, len(poses), step):
        tx, ty, tz = poses[i, 0:3]
        qx, qy, qz, qw = poses[i, 3:7]
        
        R = get_rotation_matrix(qx, qy, qz, qw)
        
        # Transform corners to world frame
        corners_world = (R @ corners_cam.T).T + np.array([tx, ty, tz])
        
        # Unpack for readability
        tl, tr, br, bl, center = corners_world
        
        # Plot Image Plane (Rectangle)
        ax.plot([tl[0], tr[0]], [tl[1], tr[1]], [tl[2], tr[2]], color=color, linewidth=0.5)
        ax.plot([tr[0], br[0]], [tr[1], br[1]], [tr[2], br[2]], color=color, linewidth=0.5)
        ax.plot([br[0], bl[0]], [br[1], bl[1]], [br[2], bl[2]], color=color, linewidth=0.5)
        ax.plot([bl[0], tl[0]], [bl[1], tl[1]], [bl[2], tl[2]], color=color, linewidth=0.5)
        
        # Plot lines from center to corners (Pyramid edges) specific colors to denote orientation?
        # Let's keep it simple or use axes.
        # Use RGB for orientation on the frustum?
        # Top-edge (Red - X), Right-edge (Green - Y)? No, standard is X-Right Y-Down.
        # Let's draw center-to-corners lines
        ax.plot([center[0], tl[0]], [center[1], tl[1]], [center[2], tl[2]], color=color, linewidth=0.5)
        ax.plot([center[0], tr[0]], [center[1], tr[1]], [center[2], tr[2]], color=color, linewidth=0.5)
        ax.plot([center[0], br[0]], [center[1], br[1]], [center[2], br[2]], color=color, linewidth=0.5)
        ax.plot([center[0], bl[0]], [center[1], bl[1]], [center[2], bl[2]], color=color, linewidth=0.5)
        
        # Add small axes at center for orientation
        x_axis = R[:, 0] * scale * 0.5
        y_axis = R[:, 1] * scale * 0.5
        z_axis = R[:, 2] * scale * 0.5
        ax.plot([tx, tx + x_axis[0]], [ty, ty + x_axis[1]], [tz, tz + x_axis[2]], 'r-', linewidth=1) # X
        ax.plot([tx, tx + y_axis[0]], [ty, ty + y_axis[1]], [tz, tz + y_axis[2]], 'g-', linewidth=1) # Y
        ax.plot([tx, tx + z_axis[0]], [ty, ty + z_axis[1]], [tz, tz + z_axis[2]], 'b-', linewidth=1) # Z


def main():
    parser = argparse.ArgumentParser(description='Plot ORB-SLAM3 Results')
    parser.add_argument('session_dir', type=str, help='Path to the session directory')
    parser.add_argument('--step', type=int, default=1, help='Plot every Nth frame')
    parser.add_argument("--scale", type=float, default=1.0, help="Scale for camera frustums (default: 1.0)")
    parser.add_argument('--verbose', action='store_true', help='Print details of plotted frames')
    parser.add_argument("--save", type=str, help="Path to save the 3D plot image")
    parser.add_argument("--no_show", action="store_true", help="Do not display the plot window")
    parser.add_argument("--align", action="store_true", help="Align estimated trajectory to ground truth")
    args = parser.parse_args()

    session_dir = args.session_dir
    
    frame_traj_path = os.path.join(session_dir, 'FrameTrajectory.txt')
    keyframe_traj_path = os.path.join(session_dir, 'KeyFrameTrajectory.txt')
    gt_path = os.path.join(session_dir, 'GroundTruth.txt')
    pcd_path = os.path.join(session_dir, 'PointCloud.txt')
    
    # Load Data
    frame_traj = load_trajectory(frame_traj_path)
    keyframe_traj = load_trajectory(keyframe_traj_path)
    gt_traj = load_trajectory(gt_path)
    # pcd = load_point_cloud(pcd_path)

    # Alignment Logic
    if args.align and gt_traj is not None and frame_traj is not None:
        if associate is None:
            print("Error: cannot align because associate.py is missing.")
        else:
            print("Aligning Estimated Trajectory to Ground Truth using Umeyama...")
            # Create dictionaries for association
            # Traj format: [tx, ty, tz, qx, qy, qz, qw, ts]
            # associate expects dict: ts -> [tx, ty, tz, qx, qy, qz, qw]
            
            # Using FrameTrajectory for alignment
            first_list = { row[7]: row[0:7] for row in gt_traj }
            second_list = { row[7]: row[0:7] for row in frame_traj }
            
            matches = associate.associate(first_list, second_list, 0.0, 0.02)
            if len(matches) < 2:
                print("Error: Not enough matches found for alignment.")
            else:
                first_xyz = np.matrix([[float(value) for value in first_list[a][0:3]] for a,b in matches]).transpose()
                second_xyz = np.matrix([[float(value) for value in second_list[b][0:3]] for a,b in matches]).transpose()
                
                rot, trans, scale = align(second_xyz, first_xyz)
                print(f"Alignment calculated. Scale: {scale:.4f}")
                
                # Apply alignment to all estimated data
                frame_traj = transform_trajectory(frame_traj, rot, trans, scale)
                if keyframe_traj is not None:
                    keyframe_traj = transform_trajectory(keyframe_traj, rot, trans, scale)
                # if pcd is not None:
                #     pcd = transform_point_cloud(pcd, rot, trans, scale)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Load and Plot Point Cloud
    # if pcd is not None and len(pcd) > 0:
    #     print(f"Loaded {len(pcd)} points from PointCloud.txt")
    #     if len(pcd) > 10000:
    #          idx = np.random.choice(len(pcd), 10000, replace=False)
    #          pcd_plot = pcd[idx]
    #     else:
    #          pcd_plot = pcd
    #     ax.scatter(pcd_plot[:, 0], pcd_plot[:, 1], pcd_plot[:, 2], s=1, c='k', alpha=0.1, label='Map Points')

    # Load and Plot Trajectories
    if frame_traj is not None and len(frame_traj) > 0:
        print(f"Loaded {len(frame_traj)} poses from FrameTrajectory.txt")
        # Calculate how many frames will be plotted
        indices = range(0, len(frame_traj), args.step)
        num_plotted = len(indices)
        print(f"Plotting {num_plotted} frames (step={args.step})")
        
        if args.verbose:
            print("Plotted Frame Timestamps:")
            for idx in indices:
                ts = frame_traj[idx, 7] # timestamp is at index 7 now
                print(f"  Frame {idx}: {ts:.6f}")
        
        # Plot trajectory line (Trajectory path)
        ax.plot(frame_traj[:, 0], frame_traj[:, 1], frame_traj[:, 2], 'k--', linewidth=0.5, alpha=0.5, label='Estimated Path')
        # Plot camera frustums
        # plot_frustums(ax, frame_traj, step=args.step, scale=args.scale, color='k')
        
    if keyframe_traj is not None and len(keyframe_traj) > 0:
        if keyframe_traj is not None and len(keyframe_traj) > 0:
             print(f"Loaded {len(keyframe_traj)} poses from KeyFrameTrajectory.txt")
             # Plot KeyFrames path in distinct color
             ax.plot(keyframe_traj[:, 0], keyframe_traj[:, 1], keyframe_traj[:, 2], 'b-', linewidth=1.5, label='KeyFrame Path')
             # Plot KeyFrame frustums (Blue)
            #  plot_frustums(ax, keyframe_traj, step=1, scale=args.scale*1.5, color='b') 

    # Load and Plot Ground Truth
    gt_traj = load_trajectory(gt_path)
    if gt_traj is not None and len(gt_traj) > 0:
        print(f"Loaded {len(gt_traj)} poses from GroundTruth.txt")
        # Just plot line for GT to reduce clutter, or maybe small axes
        ax.plot(gt_traj[:, 0], gt_traj[:, 1], gt_traj[:, 2], 'm--', linewidth=1, label='Ground Truth')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.legend()
    ax.set_title( f'ORB-SLAM3 Results: {os.path.basename(session_dir)}\n(Step={args.step}, Scale={args.scale}, Aligned={args.align})')
    
    # Equal aspect ratio hack
    all_points = []
    # if pcd is not None: all_points.append(pcd) # Use possibly aligned pcd
    if frame_traj is not None: all_points.append(frame_traj[:, 0:3])
    if gt_traj is not None: all_points.append(gt_traj[:, 0:3])
    
    if all_points:
        all_points = np.vstack(all_points)
        max_range = np.array([all_points[:,0].max()-all_points[:,0].min(), 
                              all_points[:,1].max()-all_points[:,1].min(), 
                              all_points[:,2].max()-all_points[:,2].min()]).max() / 2.0
        mid_x = (all_points[:,0].max()+all_points[:,0].min()) * 0.5
        mid_y = (all_points[:,1].max()+all_points[:,1].min()) * 0.5
        mid_z = (all_points[:,2].max()+all_points[:,2].min()) * 0.5
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

    if args.save:
        plt.savefig(args.save)
        print(f"Plot saved to {args.save}")

    if not args.no_show:
        plt.show()

if __name__ == '__main__':
    main()

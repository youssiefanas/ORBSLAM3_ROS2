#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
# Try importing point_cloud2 from sensor_msgs_py, fallback gracefully if needed (though standard in ROS2)
try:
    from sensor_msgs_py import point_cloud2
except ImportError:
    import sensor_msgs.point_cloud2 as point_cloud2

import sys
import os
import argparse
import numpy as np

# Add current directory to path to import associate
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
try:
    import associate
except ImportError:
    print("Warning: associate.py not found. Alignment will not work.")
    associate = None

# Helper functions from plot_results.py
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
    traj: Nx8 (tx, ty, tz, qx, qy, qz, qw, ts)
    """
    new_traj = np.copy(traj)
    
    # 1. Transform positions
    # positions are at indices 0, 1, 2
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
    # R is matrix, t is matrix (3,1). Result is matrix.
    new_pcd = (s * R @ new_pcd + t).T
    return np.asarray(new_pcd)

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

class OTPublisher(Node):
    def __init__(self, gt_file_path, traj_file_paths, pcd_file_path=None, do_align=False):
        super().__init__('gt_publisher')
        
        self.publisher_ = self.create_publisher(Path, '/gt_path', 10)
        self.traj_publishers = {}
        
        # Point Cloud Publisher
        self.pcd_publisher_ = None
        if pcd_file_path:
             self.pcd_publisher_ = self.create_publisher(PointCloud2, '/point_cloud', 10)
        
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.gt_file_path = gt_file_path
        self.traj_file_paths = traj_file_paths or []
        self.pcd_file_path = pcd_file_path
        self.do_align = do_align
        
        self.gt_msg = Path()
        self.traj_msgs = {} # topic -> Path
        self.pcd_msg = None

        # Load Data
        self.load_data()

    def load_data(self):
        # 1. Load Ground Truth
        gt_data = load_trajectory(self.gt_file_path)
        if gt_data is not None:
            self.get_logger().info(f"Loaded GT from: {self.gt_file_path} ({len(gt_data)} poses)")
            self.gt_msg = self.numpy_to_path_msg(gt_data, "world")
        else:
            self.get_logger().error(f"Failed to load GT from {self.gt_file_path}")

        # 2. Load Point Cloud
        pcd_data = None
        if self.pcd_file_path:
            pcd_data = load_point_cloud(self.pcd_file_path)
            if pcd_data is not None:
                self.get_logger().info(f"Loaded Point Cloud from: {self.pcd_file_path} ({len(pcd_data)} points)")
            else:
                self.get_logger().error(f"Failed to load PCD from {self.pcd_file_path}")

        # 3. Load and (optionally) Align Trajectories
        alignment_transform = None # (rot, trans, scale)

        for idx, traj_path in enumerate(self.traj_file_paths, 1):
            traj_data = load_trajectory(traj_path)
            topic_name = f'/trajectory_{idx}'
            frame_id = "world"

            if traj_data is not None:
                self.get_logger().info(f"Loaded Trajectory {idx} from: {traj_path} ({len(traj_data)} poses)")
                
                if self.do_align and gt_data is not None:
                     if associate is None:
                         pass 
                     else:
                         self.get_logger().info(f"Aligning Trajectory {idx} to GT...")
                         first_list = { row[7]: row[0:7] for row in gt_data }
                         second_list = { row[7]: row[0:7] for row in traj_data }
                         
                         matches = associate.associate(first_list, second_list, 0.0, 0.02)
                         if len(matches) < 2:
                             self.get_logger().warning(f"Not enough matches to align Trajectory {idx}")
                         else:
                             first_xyz = np.matrix([[float(value) for value in first_list[a][0:3]] for a,b in matches]).transpose()
                             second_xyz = np.matrix([[float(value) for value in second_list[b][0:3]] for a,b in matches]).transpose()
                             
                             rot, trans, scale = align(second_xyz, first_xyz)
                             self.get_logger().info(f"Alignment calculated for Traj {idx}. Scale: {scale:.4f}")
                             traj_data = transform_trajectory(traj_data, rot, trans, scale)
                             
                             # If this is the first trajectory, save transform for point cloud
                             if idx == 1:
                                 alignment_transform = (rot, trans, scale)

                # Create Publisher and Msg
                self.traj_publishers[topic_name] = self.create_publisher(Path, topic_name, 10)
                self.traj_msgs[topic_name] = self.numpy_to_path_msg(traj_data, frame_id)
            else:
                self.get_logger().error(f"Failed to load Trajectory {idx} from {traj_path}")

        # 4. Apply alignment to Point Cloud if available
        if pcd_data is not None:
            if alignment_transform is not None:
                self.get_logger().info("Applying Trajectory 1 alignment to Point Cloud...")
                rot, trans, scale = alignment_transform
                pcd_data = transform_point_cloud(pcd_data, rot, trans, scale)
            elif self.do_align and gt_data is not None and not self.traj_file_paths:
                pass
            
            # Convert to PointCloud2
            header = Header()
            header.frame_id = "world"
            self.pcd_msg = point_cloud2.create_cloud_xyz32(header, pcd_data)

    def numpy_to_path_msg(self, data, frame_id):
        path_msg = Path()
        path_msg.header.frame_id = frame_id
        
        for row in data:
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = row[0]
            pose.pose.position.y = row[1]
            pose.pose.position.z = row[2]
            pose.pose.orientation.x = row[3]
            pose.pose.orientation.y = row[4]
            pose.pose.orientation.z = row[5]
            pose.pose.orientation.w = row[6]
            path_msg.poses.append(pose)
            
        return path_msg

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        
        # Publish GT
        if len(self.gt_msg.poses) > 0:
            self.gt_msg.header.stamp = now
            self.publisher_.publish(self.gt_msg)
            
        # Publish Trajectories
        for topic_name, msg in self.traj_msgs.items():
            msg.header.stamp = now
            self.traj_publishers[topic_name].publish(msg)
            
        # Publish Point Cloud
        if self.pcd_publisher_ and self.pcd_msg:
            self.pcd_msg.header.stamp = now
            self.pcd_publisher_.publish(self.pcd_msg)

def main(args=None):
    rclpy.init(args=args)
    
    parser = argparse.ArgumentParser(description='Publish GT, Trajectories, and Point Cloud to RViz')
    parser.add_argument('gt_file', type=str, help='Path to Ground Truth file (TUM format)')
    parser.add_argument('traj_files', type=str, nargs='*', help='paths to estimated trajectory files (TUM format)')
    parser.add_argument('--pcd', type=str, help='Path to PointCloud file (optional)', default=None)
    parser.add_argument('--align', action='store_true', help='Align trajectories and point cloud to ground truth')
    
    clean_argv = [arg for arg in sys.argv[1:] if not arg.startswith('--ros')]
    
    parsed_args = parser.parse_args(clean_argv)
    
    node = OTPublisher(parsed_args.gt_file, parsed_args.traj_files, pcd_file_path=parsed_args.pcd, do_align=parsed_args.align)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

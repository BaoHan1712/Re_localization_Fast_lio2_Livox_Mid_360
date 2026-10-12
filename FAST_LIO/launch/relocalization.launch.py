import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():

  # 1. Transform publisher (map -> odom)
  map_odom_trans = Node(
      package='icp_relocalization',
      executable='transform_publisher',
      name='transform_publisher',
      output='screen'
  )

  # 2. ICP Relocalization Node
  icp_node = Node(
      package='icp_relocalization',
      executable='icp_node',
      name='icp_node',
      output='screen',
      parameters=[
          {'initial_x': 0.0},
          {'initial_y': 0.0},
          {'initial_z': 0.0},
          {'initial_a': 0.0},

          {'map_voxel_leaf_size': 0.5},
          {'cloud_voxel_leaf_size': 0.3},
          {'map_frame_id': 'map'},
          {'solver_max_iter': 100},
          {'max_correspondence_distance': 0.2},
          {'RANSAC_outlier_rejection_threshold': 0.5},
          {'map_path': '/home/hanbao/Downloads/localization_ws/src/FAST_LIO/PCD/scans.pcd'}, 
          {'fitness_score_thre': 0.5}, 
          {'converged_count_thre': 20},  # Giảm xuống 5 để publish pose ngay
          {'pcl_type': 'livox'},
      ],
  )
  
  # 3. FAST-LIO Relocalization Node   
  config = os.path.join(
      get_package_share_directory('fast_lio'), 'config', 'fast_lio_relocalization_param.yaml')
  
  fast_lio_node = Node(
      package='fast_lio',
      executable='fastlio_mapping',
      parameters=[config],
      output='screen',
      remappings=[
          ('/Odometry', '/state_estimation'),
          ('/icp_result', '/icp_result') # Giữ khớp topic giao tiếp
      ]
  )
        
  # 4. RViz2
  rviz_config_file = os.path.join(
    get_package_share_directory('fast_lio'), 'rviz', 'loam_livox.rviz')
  start_rviz = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'warn'],
    output='screen'
  )

  # Delay 5s chờ Driver Livox sẵn sàng
  delayed_start_lio = TimerAction(
    period=5.0,
    actions=[
      icp_node,
      fast_lio_node
    ]
  )

  ld = LaunchDescription()

  ld.add_action(map_odom_trans)
  ld.add_action(start_rviz)
  ld.add_action(delayed_start_lio)

  return ld

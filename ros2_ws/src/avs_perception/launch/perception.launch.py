import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch configurations
    model_param_path = LaunchConfiguration('model_param_path')
    model_bin_path   = LaunchConfiguration('model_bin_path')
    video_path       = LaunchConfiguration('video_path')
    output_path      = LaunchConfiguration('output_path')
    prob_threshold   = LaunchConfiguration('prob_threshold')
    nms_threshold    = LaunchConfiguration('nms_threshold')
    mode             = LaunchConfiguration('mode')  # 'live' or 'test'
    
    use_vulkan_compute = LaunchConfiguration('use_vulkan_compute')
    use_fp16_packed    = LaunchConfiguration('use_fp16_packed')
    use_fp16_storage   = LaunchConfiguration('use_fp16_storage')
    use_fp16_arithmetic = LaunchConfiguration('use_fp16_arithmetic')
    use_packing_layout = LaunchConfiguration('use_packing_layout')
    use_int8_inference = LaunchConfiguration('use_int8_inference')
    target_size        = LaunchConfiguration('target_size')
    num_threads        = LaunchConfiguration('num_threads')

    return LaunchDescription([
        # ── Declare arguments ──────────────────────────────────────────────
        DeclareLaunchArgument(
            'model_param_path',
            default_value='/workspace/models/best_ncnn_model/model.ncnn.param',
            description='Path to NCNN model param file'
        ),
        DeclareLaunchArgument(
            'model_bin_path',
            default_value='/workspace/models/best_ncnn_model/model.ncnn.bin',
            description='Path to NCNN model bin file'
        ),
        DeclareLaunchArgument(
            'video_path',
            default_value='/workspace/test/test_video/video_test1.mp4',
            description='Path to input video for testing'
        ),
        DeclareLaunchArgument(
            'output_path',
            default_value='/workspace/test/test_video_output/output_video_test1.mp4',
            description='Path to output video for testing'
        ),
        DeclareLaunchArgument(
            'prob_threshold',
            default_value='0.25',
            description='Probability threshold'
        ),
        DeclareLaunchArgument(
            'nms_threshold',
            default_value='0.45',
            description='NMS threshold'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='live',
            description='Execution mode: live or test'
        ),
        DeclareLaunchArgument('use_vulkan_compute', default_value='false'),
        DeclareLaunchArgument('use_fp16_packed', default_value='true'),
        DeclareLaunchArgument('use_fp16_storage', default_value='true'),
        DeclareLaunchArgument('use_fp16_arithmetic', default_value='true'),
        DeclareLaunchArgument('use_packing_layout', default_value='true'),
        DeclareLaunchArgument('use_int8_inference', default_value='false'),
        DeclareLaunchArgument('target_size', default_value='320'),
        DeclareLaunchArgument('num_threads', default_value='3'),

        # ── Live Inference Node ────────────────────────────────────────────
        Node(
            package='avs_perception',
            executable='ncnn_inference_node',
            name='ncnn_inference_node',
            output='screen',
            parameters=[{
                'model_param_path': model_param_path,
                'model_bin_path':   model_bin_path,
                'prob_threshold':   prob_threshold,
                'nms_threshold':    nms_threshold,
                'use_vulkan_compute': use_vulkan_compute,
                'use_fp16_packed': use_fp16_packed,
                'use_fp16_storage': use_fp16_storage,
                'use_fp16_arithmetic': use_fp16_arithmetic,
                'use_packing_layout': use_packing_layout,
                'use_int8_inference': use_int8_inference,
                'target_size': target_size,
                'num_threads': num_threads,
            }],
            condition=IfCondition(PythonExpression(["'", mode, "' == 'live'"]))
        ),

        # ── Video Profiling/Test Node ──────────────────────────────────────
        Node(
            package='avs_perception',
            executable='video_test_node',
            name='video_test_node',
            output='screen',
            parameters=[{
                'model_param_path': model_param_path,
                'model_bin_path':   model_bin_path,
                'video_path':       video_path,
                'output_path':      output_path,
                'prob_threshold':   prob_threshold,
                'nms_threshold':    nms_threshold,
                'use_vulkan_compute': use_vulkan_compute,
                'use_fp16_packed': use_fp16_packed,
                'use_fp16_storage': use_fp16_storage,
                'use_fp16_arithmetic': use_fp16_arithmetic,
                'use_packing_layout': use_packing_layout,
                'use_int8_inference': use_int8_inference,
                'target_size': target_size,
                'num_threads': num_threads,
            }],
            condition=IfCondition(PythonExpression(["'", mode, "' == 'test'"]))
        ),

        # ── IPM Transform Node (pixel → real-world mm + look-ahead) ───────
        Node(
            package='avs_perception',
            executable='ipm_transform_node',
            name='ipm_transform_node',
            output='screen',
            parameters=[{
                'calibration_file_path': '/workspace/config/calibration.json',
                'lookahead_T_preview':   0.15,   # seconds
                'lookahead_d_min_mm':    120.0,  # mm
                'lookahead_d_max_mm':    450.0,  # mm
            }]
        ),

        # ── Lane Error Publisher Node ──────────────────────────────────────
        # Computes epsilon_x, epsilon_y, theta in vehicle frame.
        # The PD controller is a separate node that reads /avs/control_error.
        Node(
            package='avs_perception',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[{
                # Turn state-machine thresholds
                'turn_proximity_mm': 500.0,  # mm — distance to arm turn transition
                'turn_done_mm':      200.0,  # mm — past-turn detection threshold
                'theta_done_rad':    0.1,    # rad — heading threshold for turn completion
            }]
        ),
    ])

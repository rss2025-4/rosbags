# Copyright 2020 - 2025 Ternaris
# SPDX-License-Identifier: Apache-2.0
#
# THIS FILE IS GENERATED, DO NOT EDIT
"""Message type definitions."""

# ruff: noqa: N801,N814,N816,TCH004

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from rosbags.interfaces import Nodetype as T

from . import ros2_humble as base

if TYPE_CHECKING:
    from typing import ClassVar

    import numpy as np

    from rosbags.interfaces.typing import Typesdict


action_msgs__msg__GoalInfo = base.action_msgs__msg__GoalInfo
action_msgs__msg__GoalStatus = base.action_msgs__msg__GoalStatus
action_msgs__msg__GoalStatusArray = base.action_msgs__msg__GoalStatusArray
actionlib_msgs__msg__GoalID = base.actionlib_msgs__msg__GoalID
actionlib_msgs__msg__GoalStatus = base.actionlib_msgs__msg__GoalStatus
actionlib_msgs__msg__GoalStatusArray = base.actionlib_msgs__msg__GoalStatusArray
builtin_interfaces__msg__Duration = base.builtin_interfaces__msg__Duration
builtin_interfaces__msg__Time = base.builtin_interfaces__msg__Time
diagnostic_msgs__msg__DiagnosticArray = base.diagnostic_msgs__msg__DiagnosticArray
diagnostic_msgs__msg__DiagnosticStatus = base.diagnostic_msgs__msg__DiagnosticStatus
diagnostic_msgs__msg__KeyValue = base.diagnostic_msgs__msg__KeyValue
geometry_msgs__msg__Accel = base.geometry_msgs__msg__Accel
geometry_msgs__msg__AccelStamped = base.geometry_msgs__msg__AccelStamped
geometry_msgs__msg__AccelWithCovariance = base.geometry_msgs__msg__AccelWithCovariance
geometry_msgs__msg__AccelWithCovarianceStamped = base.geometry_msgs__msg__AccelWithCovarianceStamped
geometry_msgs__msg__Inertia = base.geometry_msgs__msg__Inertia
geometry_msgs__msg__InertiaStamped = base.geometry_msgs__msg__InertiaStamped
geometry_msgs__msg__Point = base.geometry_msgs__msg__Point
geometry_msgs__msg__Point32 = base.geometry_msgs__msg__Point32
geometry_msgs__msg__PointStamped = base.geometry_msgs__msg__PointStamped
geometry_msgs__msg__Polygon = base.geometry_msgs__msg__Polygon
geometry_msgs__msg__PolygonStamped = base.geometry_msgs__msg__PolygonStamped
geometry_msgs__msg__Pose = base.geometry_msgs__msg__Pose
geometry_msgs__msg__Pose2D = base.geometry_msgs__msg__Pose2D
geometry_msgs__msg__PoseArray = base.geometry_msgs__msg__PoseArray
geometry_msgs__msg__PoseStamped = base.geometry_msgs__msg__PoseStamped
geometry_msgs__msg__PoseWithCovariance = base.geometry_msgs__msg__PoseWithCovariance
geometry_msgs__msg__PoseWithCovarianceStamped = base.geometry_msgs__msg__PoseWithCovarianceStamped
geometry_msgs__msg__Quaternion = base.geometry_msgs__msg__Quaternion
geometry_msgs__msg__QuaternionStamped = base.geometry_msgs__msg__QuaternionStamped
geometry_msgs__msg__Transform = base.geometry_msgs__msg__Transform
geometry_msgs__msg__TransformStamped = base.geometry_msgs__msg__TransformStamped
geometry_msgs__msg__Twist = base.geometry_msgs__msg__Twist
geometry_msgs__msg__TwistStamped = base.geometry_msgs__msg__TwistStamped
geometry_msgs__msg__TwistWithCovariance = base.geometry_msgs__msg__TwistWithCovariance
geometry_msgs__msg__TwistWithCovarianceStamped = base.geometry_msgs__msg__TwistWithCovarianceStamped
geometry_msgs__msg__Vector3 = base.geometry_msgs__msg__Vector3
geometry_msgs__msg__Vector3Stamped = base.geometry_msgs__msg__Vector3Stamped
geometry_msgs__msg__Wrench = base.geometry_msgs__msg__Wrench
geometry_msgs__msg__WrenchStamped = base.geometry_msgs__msg__WrenchStamped
lifecycle_msgs__msg__State = base.lifecycle_msgs__msg__State
lifecycle_msgs__msg__Transition = base.lifecycle_msgs__msg__Transition
lifecycle_msgs__msg__TransitionDescription = base.lifecycle_msgs__msg__TransitionDescription
lifecycle_msgs__msg__TransitionEvent = base.lifecycle_msgs__msg__TransitionEvent
nav_msgs__msg__GridCells = base.nav_msgs__msg__GridCells
nav_msgs__msg__MapMetaData = base.nav_msgs__msg__MapMetaData
nav_msgs__msg__OccupancyGrid = base.nav_msgs__msg__OccupancyGrid
nav_msgs__msg__Odometry = base.nav_msgs__msg__Odometry
nav_msgs__msg__Path = base.nav_msgs__msg__Path
rcl_interfaces__msg__FloatingPointRange = base.rcl_interfaces__msg__FloatingPointRange
rcl_interfaces__msg__IntegerRange = base.rcl_interfaces__msg__IntegerRange
rcl_interfaces__msg__ListParametersResult = base.rcl_interfaces__msg__ListParametersResult
rcl_interfaces__msg__Log = base.rcl_interfaces__msg__Log
rcl_interfaces__msg__Parameter = base.rcl_interfaces__msg__Parameter
rcl_interfaces__msg__ParameterDescriptor = base.rcl_interfaces__msg__ParameterDescriptor
rcl_interfaces__msg__ParameterEvent = base.rcl_interfaces__msg__ParameterEvent
rcl_interfaces__msg__ParameterEventDescriptors = base.rcl_interfaces__msg__ParameterEventDescriptors
rcl_interfaces__msg__ParameterType = base.rcl_interfaces__msg__ParameterType
rcl_interfaces__msg__SetParametersResult = base.rcl_interfaces__msg__SetParametersResult
rmw_dds_common__msg__Gid = base.rmw_dds_common__msg__Gid
rmw_dds_common__msg__NodeEntitiesInfo = base.rmw_dds_common__msg__NodeEntitiesInfo
rmw_dds_common__msg__ParticipantEntitiesInfo = base.rmw_dds_common__msg__ParticipantEntitiesInfo
rosbag2_interfaces__msg__ReadSplitEvent = base.rosbag2_interfaces__msg__ReadSplitEvent
rosbag2_interfaces__msg__WriteSplitEvent = base.rosbag2_interfaces__msg__WriteSplitEvent
rosgraph_msgs__msg__Clock = base.rosgraph_msgs__msg__Clock
sensor_msgs__msg__BatteryState = base.sensor_msgs__msg__BatteryState
sensor_msgs__msg__CameraInfo = base.sensor_msgs__msg__CameraInfo
sensor_msgs__msg__ChannelFloat32 = base.sensor_msgs__msg__ChannelFloat32
sensor_msgs__msg__CompressedImage = base.sensor_msgs__msg__CompressedImage
sensor_msgs__msg__FluidPressure = base.sensor_msgs__msg__FluidPressure
sensor_msgs__msg__Illuminance = base.sensor_msgs__msg__Illuminance
sensor_msgs__msg__Image = base.sensor_msgs__msg__Image
sensor_msgs__msg__Imu = base.sensor_msgs__msg__Imu
sensor_msgs__msg__JointState = base.sensor_msgs__msg__JointState
sensor_msgs__msg__Joy = base.sensor_msgs__msg__Joy
sensor_msgs__msg__JoyFeedback = base.sensor_msgs__msg__JoyFeedback
sensor_msgs__msg__JoyFeedbackArray = base.sensor_msgs__msg__JoyFeedbackArray
sensor_msgs__msg__LaserEcho = base.sensor_msgs__msg__LaserEcho
sensor_msgs__msg__LaserScan = base.sensor_msgs__msg__LaserScan
sensor_msgs__msg__MagneticField = base.sensor_msgs__msg__MagneticField
sensor_msgs__msg__MultiDOFJointState = base.sensor_msgs__msg__MultiDOFJointState
sensor_msgs__msg__MultiEchoLaserScan = base.sensor_msgs__msg__MultiEchoLaserScan
sensor_msgs__msg__NavSatFix = base.sensor_msgs__msg__NavSatFix
sensor_msgs__msg__NavSatStatus = base.sensor_msgs__msg__NavSatStatus
sensor_msgs__msg__PointCloud = base.sensor_msgs__msg__PointCloud
sensor_msgs__msg__PointCloud2 = base.sensor_msgs__msg__PointCloud2
sensor_msgs__msg__PointField = base.sensor_msgs__msg__PointField
sensor_msgs__msg__Range = base.sensor_msgs__msg__Range
sensor_msgs__msg__RegionOfInterest = base.sensor_msgs__msg__RegionOfInterest
sensor_msgs__msg__RelativeHumidity = base.sensor_msgs__msg__RelativeHumidity
sensor_msgs__msg__Temperature = base.sensor_msgs__msg__Temperature
sensor_msgs__msg__TimeReference = base.sensor_msgs__msg__TimeReference
shape_msgs__msg__Mesh = base.shape_msgs__msg__Mesh
shape_msgs__msg__MeshTriangle = base.shape_msgs__msg__MeshTriangle
shape_msgs__msg__Plane = base.shape_msgs__msg__Plane
shape_msgs__msg__SolidPrimitive = base.shape_msgs__msg__SolidPrimitive
statistics_msgs__msg__MetricsMessage = base.statistics_msgs__msg__MetricsMessage
statistics_msgs__msg__StatisticDataPoint = base.statistics_msgs__msg__StatisticDataPoint
statistics_msgs__msg__StatisticDataType = base.statistics_msgs__msg__StatisticDataType
std_msgs__msg__Bool = base.std_msgs__msg__Bool
std_msgs__msg__Byte = base.std_msgs__msg__Byte
std_msgs__msg__ColorRGBA = base.std_msgs__msg__ColorRGBA
std_msgs__msg__Empty = base.std_msgs__msg__Empty
std_msgs__msg__Float32 = base.std_msgs__msg__Float32
std_msgs__msg__Float32MultiArray = base.std_msgs__msg__Float32MultiArray
std_msgs__msg__Float64 = base.std_msgs__msg__Float64
std_msgs__msg__Float64MultiArray = base.std_msgs__msg__Float64MultiArray
std_msgs__msg__Header = base.std_msgs__msg__Header
std_msgs__msg__Int16 = base.std_msgs__msg__Int16
std_msgs__msg__Int16MultiArray = base.std_msgs__msg__Int16MultiArray
std_msgs__msg__Int32 = base.std_msgs__msg__Int32
std_msgs__msg__Int32MultiArray = base.std_msgs__msg__Int32MultiArray
std_msgs__msg__Int64 = base.std_msgs__msg__Int64
std_msgs__msg__Int64MultiArray = base.std_msgs__msg__Int64MultiArray
std_msgs__msg__Int8 = base.std_msgs__msg__Int8
std_msgs__msg__Int8MultiArray = base.std_msgs__msg__Int8MultiArray
std_msgs__msg__MultiArrayDimension = base.std_msgs__msg__MultiArrayDimension
std_msgs__msg__MultiArrayLayout = base.std_msgs__msg__MultiArrayLayout
std_msgs__msg__String = base.std_msgs__msg__String
std_msgs__msg__UInt16 = base.std_msgs__msg__UInt16
std_msgs__msg__UInt16MultiArray = base.std_msgs__msg__UInt16MultiArray
std_msgs__msg__UInt32 = base.std_msgs__msg__UInt32
std_msgs__msg__UInt32MultiArray = base.std_msgs__msg__UInt32MultiArray
std_msgs__msg__UInt64 = base.std_msgs__msg__UInt64
std_msgs__msg__UInt64MultiArray = base.std_msgs__msg__UInt64MultiArray
std_msgs__msg__UInt8 = base.std_msgs__msg__UInt8
std_msgs__msg__UInt8MultiArray = base.std_msgs__msg__UInt8MultiArray
stereo_msgs__msg__DisparityImage = base.stereo_msgs__msg__DisparityImage
tf2_msgs__msg__TF2Error = base.tf2_msgs__msg__TF2Error
tf2_msgs__msg__TFMessage = base.tf2_msgs__msg__TFMessage
trajectory_msgs__msg__JointTrajectory = base.trajectory_msgs__msg__JointTrajectory
trajectory_msgs__msg__JointTrajectoryPoint = base.trajectory_msgs__msg__JointTrajectoryPoint
trajectory_msgs__msg__MultiDOFJointTrajectory = base.trajectory_msgs__msg__MultiDOFJointTrajectory
trajectory_msgs__msg__MultiDOFJointTrajectoryPoint = base.trajectory_msgs__msg__MultiDOFJointTrajectoryPoint
unique_identifier_msgs__msg__UUID = base.unique_identifier_msgs__msg__UUID
visualization_msgs__msg__ImageMarker = base.visualization_msgs__msg__ImageMarker
visualization_msgs__msg__InteractiveMarker = base.visualization_msgs__msg__InteractiveMarker
visualization_msgs__msg__InteractiveMarkerControl = base.visualization_msgs__msg__InteractiveMarkerControl
visualization_msgs__msg__InteractiveMarkerFeedback = base.visualization_msgs__msg__InteractiveMarkerFeedback
visualization_msgs__msg__InteractiveMarkerInit = base.visualization_msgs__msg__InteractiveMarkerInit
visualization_msgs__msg__InteractiveMarkerPose = base.visualization_msgs__msg__InteractiveMarkerPose
visualization_msgs__msg__InteractiveMarkerUpdate = base.visualization_msgs__msg__InteractiveMarkerUpdate
visualization_msgs__msg__Marker = base.visualization_msgs__msg__Marker
visualization_msgs__msg__MarkerArray = base.visualization_msgs__msg__MarkerArray
visualization_msgs__msg__MenuEntry = base.visualization_msgs__msg__MenuEntry
visualization_msgs__msg__MeshFile = base.visualization_msgs__msg__MeshFile
visualization_msgs__msg__UVCoordinate = base.visualization_msgs__msg__UVCoordinate


@dataclass
class ackermann_msgs__msg__AckermannDrive:
    """Class for ackermann_msgs/msg/AckermannDrive."""

    steering_angle: float
    steering_angle_velocity: float
    speed: float
    acceleration: float
    jerk: float
    __msgtype__: ClassVar[str] = 'ackermann_msgs/msg/AckermannDrive'


@dataclass
class ackermann_msgs__msg__AckermannDriveStamped:
    """Class for ackermann_msgs/msg/AckermannDriveStamped."""

    header: std_msgs__msg__Header
    drive: ackermann_msgs__msg__AckermannDrive
    __msgtype__: ClassVar[str] = 'ackermann_msgs/msg/AckermannDriveStamped'


@dataclass
class action_msgs__srv__msg__CancelGoal_Request:
    """Class for action_msgs/srv/msg/CancelGoal_Request."""

    goal_info: action_msgs__srv__msg__GoalInfo
    __msgtype__: ClassVar[str] = 'action_msgs/srv/msg/CancelGoal_Request'


@dataclass
class action_msgs__srv__msg__CancelGoal_Response:
    """Class for action_msgs/srv/msg/CancelGoal_Response."""

    return_code: int
    goals_canceling: list[action_msgs__srv__msg__GoalInfo]
    ERROR_NONE: ClassVar[int] = 0
    ERROR_REJECTED: ClassVar[int] = 1
    ERROR_UNKNOWN_GOAL_ID: ClassVar[int] = 2
    ERROR_GOAL_TERMINATED: ClassVar[int] = 3
    __msgtype__: ClassVar[str] = 'action_msgs/srv/msg/CancelGoal_Response'


@dataclass
class action_tutorials_interfaces__action__Fibonacci_Feedback:
    """Class for action_tutorials_interfaces/action/Fibonacci_Feedback."""

    partial_sequence: np.ndarray[tuple[int, ...], np.dtype[np.int32]]
    __msgtype__: ClassVar[str] = 'action_tutorials_interfaces/action/Fibonacci_Feedback'


@dataclass
class action_tutorials_interfaces__action__Fibonacci_Goal:
    """Class for action_tutorials_interfaces/action/Fibonacci_Goal."""

    order: int
    __msgtype__: ClassVar[str] = 'action_tutorials_interfaces/action/Fibonacci_Goal'


@dataclass
class action_tutorials_interfaces__action__Fibonacci_Result:
    """Class for action_tutorials_interfaces/action/Fibonacci_Result."""

    sequence: np.ndarray[tuple[int, ...], np.dtype[np.int32]]
    __msgtype__: ClassVar[str] = 'action_tutorials_interfaces/action/Fibonacci_Result'


@dataclass
class bond__msg__Constants:
    """Class for bond/msg/Constants."""

    structure_needs_at_least_one_member: int = 0
    DEAD_PUBLISH_PERIOD: ClassVar[float] = 0.05
    DEFAULT_CONNECT_TIMEOUT: ClassVar[float] = 10.0
    DEFAULT_HEARTBEAT_TIMEOUT: ClassVar[float] = 4.0
    DEFAULT_DISCONNECT_TIMEOUT: ClassVar[float] = 2.0
    DEFAULT_HEARTBEAT_PERIOD: ClassVar[float] = 1.0
    DISABLE_HEARTBEAT_TIMEOUT_PARAM: ClassVar[str] = '/bond_disable_heartbeat_timeout'
    __msgtype__: ClassVar[str] = 'bond/msg/Constants'


@dataclass
class bond__msg__Status:
    """Class for bond/msg/Status."""

    header: std_msgs__msg__Header
    id: str
    instance_id: str
    active: bool
    heartbeat_timeout: float
    heartbeat_period: float
    __msgtype__: ClassVar[str] = 'bond/msg/Status'


@dataclass
class composition_interfaces__srv__ListNodes_Request:
    """Class for composition_interfaces/srv/ListNodes_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'composition_interfaces/srv/ListNodes_Request'


@dataclass
class composition_interfaces__srv__ListNodes_Response:
    """Class for composition_interfaces/srv/ListNodes_Response."""

    full_node_names: list[str]
    unique_ids: np.ndarray[tuple[int, ...], np.dtype[np.uint64]]
    __msgtype__: ClassVar[str] = 'composition_interfaces/srv/ListNodes_Response'


@dataclass
class composition_interfaces__srv__LoadNode_Request:
    """Class for composition_interfaces/srv/LoadNode_Request."""

    package_name: str
    plugin_name: str
    node_name: str
    node_namespace: str
    log_level: int
    remap_rules: list[str]
    parameters: list[rcl_interfaces__msg__Parameter]
    extra_arguments: list[rcl_interfaces__msg__Parameter]
    __msgtype__: ClassVar[str] = 'composition_interfaces/srv/LoadNode_Request'


@dataclass
class composition_interfaces__srv__LoadNode_Response:
    """Class for composition_interfaces/srv/LoadNode_Response."""

    success: bool
    error_message: str
    full_node_name: str
    unique_id: int
    __msgtype__: ClassVar[str] = 'composition_interfaces/srv/LoadNode_Response'


@dataclass
class composition_interfaces__srv__UnloadNode_Request:
    """Class for composition_interfaces/srv/UnloadNode_Request."""

    unique_id: int
    __msgtype__: ClassVar[str] = 'composition_interfaces/srv/UnloadNode_Request'


@dataclass
class composition_interfaces__srv__UnloadNode_Response:
    """Class for composition_interfaces/srv/UnloadNode_Response."""

    success: bool
    error_message: str
    __msgtype__: ClassVar[str] = 'composition_interfaces/srv/UnloadNode_Response'


@dataclass
class composition_interfaces__srv__msg__ListNodes_Request:
    """Class for composition_interfaces/srv/msg/ListNodes_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'composition_interfaces/srv/msg/ListNodes_Request'


@dataclass
class composition_interfaces__srv__msg__ListNodes_Response:
    """Class for composition_interfaces/srv/msg/ListNodes_Response."""

    full_node_names: list[str]
    unique_ids: np.ndarray[tuple[int, ...], np.dtype[np.uint64]]
    __msgtype__: ClassVar[str] = 'composition_interfaces/srv/msg/ListNodes_Response'


@dataclass
class composition_interfaces__srv__msg__LoadNode_Request:
    """Class for composition_interfaces/srv/msg/LoadNode_Request."""

    package_name: str
    plugin_name: str
    node_name: str
    node_namespace: str
    log_level: int
    remap_rules: list[str]
    parameters: list[rcl_interfaces__msg__Parameter]
    extra_arguments: list[rcl_interfaces__msg__Parameter]
    __msgtype__: ClassVar[str] = 'composition_interfaces/srv/msg/LoadNode_Request'


@dataclass
class composition_interfaces__srv__msg__LoadNode_Response:
    """Class for composition_interfaces/srv/msg/LoadNode_Response."""

    success: bool
    error_message: str
    full_node_name: str
    unique_id: int
    __msgtype__: ClassVar[str] = 'composition_interfaces/srv/msg/LoadNode_Response'


@dataclass
class composition_interfaces__srv__msg__UnloadNode_Request:
    """Class for composition_interfaces/srv/msg/UnloadNode_Request."""

    unique_id: int
    __msgtype__: ClassVar[str] = 'composition_interfaces/srv/msg/UnloadNode_Request'


@dataclass
class composition_interfaces__srv__msg__UnloadNode_Response:
    """Class for composition_interfaces/srv/msg/UnloadNode_Response."""

    success: bool
    error_message: str
    __msgtype__: ClassVar[str] = 'composition_interfaces/srv/msg/UnloadNode_Response'


@dataclass
class diagnostic_msgs__srv__AddDiagnostics_Request:
    """Class for diagnostic_msgs/srv/AddDiagnostics_Request."""

    load_namespace: str
    __msgtype__: ClassVar[str] = 'diagnostic_msgs/srv/AddDiagnostics_Request'


@dataclass
class diagnostic_msgs__srv__AddDiagnostics_Response:
    """Class for diagnostic_msgs/srv/AddDiagnostics_Response."""

    success: bool
    message: str
    __msgtype__: ClassVar[str] = 'diagnostic_msgs/srv/AddDiagnostics_Response'


@dataclass
class diagnostic_msgs__srv__SelfTest_Request:
    """Class for diagnostic_msgs/srv/SelfTest_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'diagnostic_msgs/srv/SelfTest_Request'


@dataclass
class diagnostic_msgs__srv__SelfTest_Response:
    """Class for diagnostic_msgs/srv/SelfTest_Response."""

    id: str
    passed: octet
    status: list[diagnostic_msgs__msg__DiagnosticStatus]
    __msgtype__: ClassVar[str] = 'diagnostic_msgs/srv/SelfTest_Response'


@dataclass
class diagnostic_msgs__srv__msg__AddDiagnostics_Request:
    """Class for diagnostic_msgs/srv/msg/AddDiagnostics_Request."""

    load_namespace: str
    __msgtype__: ClassVar[str] = 'diagnostic_msgs/srv/msg/AddDiagnostics_Request'


@dataclass
class diagnostic_msgs__srv__msg__AddDiagnostics_Response:
    """Class for diagnostic_msgs/srv/msg/AddDiagnostics_Response."""

    success: bool
    message: str
    __msgtype__: ClassVar[str] = 'diagnostic_msgs/srv/msg/AddDiagnostics_Response'


@dataclass
class diagnostic_msgs__srv__msg__SelfTest_Request:
    """Class for diagnostic_msgs/srv/msg/SelfTest_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'diagnostic_msgs/srv/msg/SelfTest_Request'


@dataclass
class diagnostic_msgs__srv__msg__SelfTest_Response:
    """Class for diagnostic_msgs/srv/msg/SelfTest_Response."""

    id: str
    passed: int
    status: list[diagnostic_msgs__srv__msg__DiagnosticStatus]
    __msgtype__: ClassVar[str] = 'diagnostic_msgs/srv/msg/SelfTest_Response'


@dataclass
class dwb_msgs__msg__CriticScore:
    """Class for dwb_msgs/msg/CriticScore."""

    name: str
    raw_score: float
    scale: float
    __msgtype__: ClassVar[str] = 'dwb_msgs/msg/CriticScore'


@dataclass
class dwb_msgs__msg__LocalPlanEvaluation:
    """Class for dwb_msgs/msg/LocalPlanEvaluation."""

    header: std_msgs__msg__Header
    twists: list[dwb_msgs__msg__TrajectoryScore]
    best_index: int
    worst_index: int
    __msgtype__: ClassVar[str] = 'dwb_msgs/msg/LocalPlanEvaluation'


@dataclass
class dwb_msgs__msg__Trajectory2D:
    """Class for dwb_msgs/msg/Trajectory2D."""

    velocity: nav_2d_msgs__msg__Twist2D
    time_offsets: list[builtin_interfaces__msg__Duration]
    poses: list[geometry_msgs__msg__Pose2D]
    __msgtype__: ClassVar[str] = 'dwb_msgs/msg/Trajectory2D'


@dataclass
class dwb_msgs__msg__TrajectoryScore:
    """Class for dwb_msgs/msg/TrajectoryScore."""

    traj: dwb_msgs__msg__Trajectory2D
    scores: list[dwb_msgs__msg__CriticScore]
    total: float
    __msgtype__: ClassVar[str] = 'dwb_msgs/msg/TrajectoryScore'


@dataclass
class dwb_msgs__srv__DebugLocalPlan_Request:
    """Class for dwb_msgs/srv/DebugLocalPlan_Request."""

    pose: nav_2d_msgs__msg__Pose2DStamped
    velocity: nav_2d_msgs__msg__Twist2D
    global_plan: nav_2d_msgs__msg__Path2D
    __msgtype__: ClassVar[str] = 'dwb_msgs/srv/DebugLocalPlan_Request'


@dataclass
class dwb_msgs__srv__DebugLocalPlan_Response:
    """Class for dwb_msgs/srv/DebugLocalPlan_Response."""

    results: dwb_msgs__msg__LocalPlanEvaluation
    __msgtype__: ClassVar[str] = 'dwb_msgs/srv/DebugLocalPlan_Response'


@dataclass
class dwb_msgs__srv__GenerateTrajectory_Request:
    """Class for dwb_msgs/srv/GenerateTrajectory_Request."""

    start_pose: geometry_msgs__msg__Pose2D
    start_vel: nav_2d_msgs__msg__Twist2D
    cmd_vel: nav_2d_msgs__msg__Twist2D
    __msgtype__: ClassVar[str] = 'dwb_msgs/srv/GenerateTrajectory_Request'


@dataclass
class dwb_msgs__srv__GenerateTrajectory_Response:
    """Class for dwb_msgs/srv/GenerateTrajectory_Response."""

    traj: dwb_msgs__msg__Trajectory2D
    __msgtype__: ClassVar[str] = 'dwb_msgs/srv/GenerateTrajectory_Response'


@dataclass
class dwb_msgs__srv__GenerateTwists_Request:
    """Class for dwb_msgs/srv/GenerateTwists_Request."""

    current_vel: nav_2d_msgs__msg__Twist2D
    __msgtype__: ClassVar[str] = 'dwb_msgs/srv/GenerateTwists_Request'


@dataclass
class dwb_msgs__srv__GenerateTwists_Response:
    """Class for dwb_msgs/srv/GenerateTwists_Response."""

    twists: list[nav_2d_msgs__msg__Twist2D]
    __msgtype__: ClassVar[str] = 'dwb_msgs/srv/GenerateTwists_Response'


@dataclass
class dwb_msgs__srv__GetCriticScore_Request:
    """Class for dwb_msgs/srv/GetCriticScore_Request."""

    pose: nav_2d_msgs__msg__Pose2DStamped
    velocity: nav_2d_msgs__msg__Twist2D
    global_plan: nav_2d_msgs__msg__Path2D
    traj: dwb_msgs__msg__Trajectory2D
    critic_name: str
    __msgtype__: ClassVar[str] = 'dwb_msgs/srv/GetCriticScore_Request'


@dataclass
class dwb_msgs__srv__GetCriticScore_Response:
    """Class for dwb_msgs/srv/GetCriticScore_Response."""

    score: dwb_msgs__msg__CriticScore
    __msgtype__: ClassVar[str] = 'dwb_msgs/srv/GetCriticScore_Response'


@dataclass
class dwb_msgs__srv__ScoreTrajectory_Request:
    """Class for dwb_msgs/srv/ScoreTrajectory_Request."""

    pose: nav_2d_msgs__msg__Pose2DStamped
    velocity: nav_2d_msgs__msg__Twist2D
    global_plan: nav_2d_msgs__msg__Path2D
    traj: dwb_msgs__msg__Trajectory2D
    __msgtype__: ClassVar[str] = 'dwb_msgs/srv/ScoreTrajectory_Request'


@dataclass
class dwb_msgs__srv__ScoreTrajectory_Response:
    """Class for dwb_msgs/srv/ScoreTrajectory_Response."""

    score: dwb_msgs__msg__TrajectoryScore
    __msgtype__: ClassVar[str] = 'dwb_msgs/srv/ScoreTrajectory_Response'


@dataclass
class dwb_msgs__srv__msg__DebugLocalPlan_Request:
    """Class for dwb_msgs/srv/msg/DebugLocalPlan_Request."""

    pose: nav_2d_msgs__msg__Pose2DStamped
    velocity: nav_2d_msgs__msg__Twist2D
    global_plan: nav_2d_msgs__msg__Path2D
    __msgtype__: ClassVar[str] = 'dwb_msgs/srv/msg/DebugLocalPlan_Request'


@dataclass
class dwb_msgs__srv__msg__DebugLocalPlan_Response:
    """Class for dwb_msgs/srv/msg/DebugLocalPlan_Response."""

    results: dwb_msgs__srv__msg__LocalPlanEvaluation
    __msgtype__: ClassVar[str] = 'dwb_msgs/srv/msg/DebugLocalPlan_Response'


@dataclass
class dwb_msgs__srv__msg__GenerateTrajectory_Request:
    """Class for dwb_msgs/srv/msg/GenerateTrajectory_Request."""

    start_pose: geometry_msgs__msg__Pose2D
    start_vel: nav_2d_msgs__msg__Twist2D
    cmd_vel: nav_2d_msgs__msg__Twist2D
    __msgtype__: ClassVar[str] = 'dwb_msgs/srv/msg/GenerateTrajectory_Request'


@dataclass
class dwb_msgs__srv__msg__GenerateTrajectory_Response:
    """Class for dwb_msgs/srv/msg/GenerateTrajectory_Response."""

    traj: dwb_msgs__srv__msg__Trajectory2D
    __msgtype__: ClassVar[str] = 'dwb_msgs/srv/msg/GenerateTrajectory_Response'


@dataclass
class dwb_msgs__srv__msg__GenerateTwists_Request:
    """Class for dwb_msgs/srv/msg/GenerateTwists_Request."""

    current_vel: nav_2d_msgs__msg__Twist2D
    __msgtype__: ClassVar[str] = 'dwb_msgs/srv/msg/GenerateTwists_Request'


@dataclass
class dwb_msgs__srv__msg__GenerateTwists_Response:
    """Class for dwb_msgs/srv/msg/GenerateTwists_Response."""

    twists: list[nav_2d_msgs__msg__Twist2D]
    __msgtype__: ClassVar[str] = 'dwb_msgs/srv/msg/GenerateTwists_Response'


@dataclass
class dwb_msgs__srv__msg__GetCriticScore_Request:
    """Class for dwb_msgs/srv/msg/GetCriticScore_Request."""

    pose: nav_2d_msgs__msg__Pose2DStamped
    velocity: nav_2d_msgs__msg__Twist2D
    global_plan: nav_2d_msgs__msg__Path2D
    traj: dwb_msgs__srv__msg__Trajectory2D
    critic_name: str
    __msgtype__: ClassVar[str] = 'dwb_msgs/srv/msg/GetCriticScore_Request'


@dataclass
class dwb_msgs__srv__msg__GetCriticScore_Response:
    """Class for dwb_msgs/srv/msg/GetCriticScore_Response."""

    score: dwb_msgs__srv__msg__CriticScore
    __msgtype__: ClassVar[str] = 'dwb_msgs/srv/msg/GetCriticScore_Response'


@dataclass
class dwb_msgs__srv__msg__ScoreTrajectory_Request:
    """Class for dwb_msgs/srv/msg/ScoreTrajectory_Request."""

    pose: nav_2d_msgs__msg__Pose2DStamped
    velocity: nav_2d_msgs__msg__Twist2D
    global_plan: nav_2d_msgs__msg__Path2D
    traj: dwb_msgs__srv__msg__Trajectory2D
    __msgtype__: ClassVar[str] = 'dwb_msgs/srv/msg/ScoreTrajectory_Request'


@dataclass
class dwb_msgs__srv__msg__ScoreTrajectory_Response:
    """Class for dwb_msgs/srv/msg/ScoreTrajectory_Response."""

    score: dwb_msgs__srv__msg__TrajectoryScore
    __msgtype__: ClassVar[str] = 'dwb_msgs/srv/msg/ScoreTrajectory_Response'


@dataclass
class example_interfaces__action__Fibonacci_Feedback:
    """Class for example_interfaces/action/Fibonacci_Feedback."""

    sequence: np.ndarray[tuple[int, ...], np.dtype[np.int32]]
    __msgtype__: ClassVar[str] = 'example_interfaces/action/Fibonacci_Feedback'


@dataclass
class example_interfaces__action__Fibonacci_Goal:
    """Class for example_interfaces/action/Fibonacci_Goal."""

    order: int
    __msgtype__: ClassVar[str] = 'example_interfaces/action/Fibonacci_Goal'


@dataclass
class example_interfaces__action__Fibonacci_Result:
    """Class for example_interfaces/action/Fibonacci_Result."""

    sequence: np.ndarray[tuple[int, ...], np.dtype[np.int32]]
    __msgtype__: ClassVar[str] = 'example_interfaces/action/Fibonacci_Result'


@dataclass
class example_interfaces__msg__Bool:
    """Class for example_interfaces/msg/Bool."""

    data: bool
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/Bool'


@dataclass
class example_interfaces__msg__Byte:
    """Class for example_interfaces/msg/Byte."""

    data: int
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/Byte'


@dataclass
class example_interfaces__msg__ByteMultiArray:
    """Class for example_interfaces/msg/ByteMultiArray."""

    layout: example_interfaces__msg__MultiArrayLayout
    data: list[octet]
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/ByteMultiArray'


@dataclass
class example_interfaces__msg__Char:
    """Class for example_interfaces/msg/Char."""

    data: int
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/Char'


@dataclass
class example_interfaces__msg__Empty:
    """Class for example_interfaces/msg/Empty."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/Empty'


@dataclass
class example_interfaces__msg__Float32:
    """Class for example_interfaces/msg/Float32."""

    data: float
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/Float32'


@dataclass
class example_interfaces__msg__Float32MultiArray:
    """Class for example_interfaces/msg/Float32MultiArray."""

    layout: example_interfaces__msg__MultiArrayLayout
    data: np.ndarray[tuple[int, ...], np.dtype[np.float32]]
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/Float32MultiArray'


@dataclass
class example_interfaces__msg__Float64:
    """Class for example_interfaces/msg/Float64."""

    data: float
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/Float64'


@dataclass
class example_interfaces__msg__Float64MultiArray:
    """Class for example_interfaces/msg/Float64MultiArray."""

    layout: example_interfaces__msg__MultiArrayLayout
    data: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/Float64MultiArray'


@dataclass
class example_interfaces__msg__Int16:
    """Class for example_interfaces/msg/Int16."""

    data: int
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/Int16'


@dataclass
class example_interfaces__msg__Int16MultiArray:
    """Class for example_interfaces/msg/Int16MultiArray."""

    layout: example_interfaces__msg__MultiArrayLayout
    data: np.ndarray[tuple[int, ...], np.dtype[np.int16]]
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/Int16MultiArray'


@dataclass
class example_interfaces__msg__Int32:
    """Class for example_interfaces/msg/Int32."""

    data: int
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/Int32'


@dataclass
class example_interfaces__msg__Int32MultiArray:
    """Class for example_interfaces/msg/Int32MultiArray."""

    layout: example_interfaces__msg__MultiArrayLayout
    data: np.ndarray[tuple[int, ...], np.dtype[np.int32]]
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/Int32MultiArray'


@dataclass
class example_interfaces__msg__Int64:
    """Class for example_interfaces/msg/Int64."""

    data: int
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/Int64'


@dataclass
class example_interfaces__msg__Int64MultiArray:
    """Class for example_interfaces/msg/Int64MultiArray."""

    layout: example_interfaces__msg__MultiArrayLayout
    data: np.ndarray[tuple[int, ...], np.dtype[np.int64]]
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/Int64MultiArray'


@dataclass
class example_interfaces__msg__Int8:
    """Class for example_interfaces/msg/Int8."""

    data: int
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/Int8'


@dataclass
class example_interfaces__msg__Int8MultiArray:
    """Class for example_interfaces/msg/Int8MultiArray."""

    layout: example_interfaces__msg__MultiArrayLayout
    data: np.ndarray[tuple[int, ...], np.dtype[np.int8]]
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/Int8MultiArray'


@dataclass
class example_interfaces__msg__MultiArrayDimension:
    """Class for example_interfaces/msg/MultiArrayDimension."""

    label: str
    size: int
    stride: int
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/MultiArrayDimension'


@dataclass
class example_interfaces__msg__MultiArrayLayout:
    """Class for example_interfaces/msg/MultiArrayLayout."""

    dim: list[example_interfaces__msg__MultiArrayDimension]
    data_offset: int
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/MultiArrayLayout'


@dataclass
class example_interfaces__msg__String:
    """Class for example_interfaces/msg/String."""

    data: str
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/String'


@dataclass
class example_interfaces__msg__UInt16:
    """Class for example_interfaces/msg/UInt16."""

    data: int
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/UInt16'


@dataclass
class example_interfaces__msg__UInt16MultiArray:
    """Class for example_interfaces/msg/UInt16MultiArray."""

    layout: example_interfaces__msg__MultiArrayLayout
    data: np.ndarray[tuple[int, ...], np.dtype[np.uint16]]
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/UInt16MultiArray'


@dataclass
class example_interfaces__msg__UInt32:
    """Class for example_interfaces/msg/UInt32."""

    data: int
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/UInt32'


@dataclass
class example_interfaces__msg__UInt32MultiArray:
    """Class for example_interfaces/msg/UInt32MultiArray."""

    layout: example_interfaces__msg__MultiArrayLayout
    data: np.ndarray[tuple[int, ...], np.dtype[np.uint32]]
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/UInt32MultiArray'


@dataclass
class example_interfaces__msg__UInt64:
    """Class for example_interfaces/msg/UInt64."""

    data: int
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/UInt64'


@dataclass
class example_interfaces__msg__UInt64MultiArray:
    """Class for example_interfaces/msg/UInt64MultiArray."""

    layout: example_interfaces__msg__MultiArrayLayout
    data: np.ndarray[tuple[int, ...], np.dtype[np.uint64]]
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/UInt64MultiArray'


@dataclass
class example_interfaces__msg__UInt8:
    """Class for example_interfaces/msg/UInt8."""

    data: int
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/UInt8'


@dataclass
class example_interfaces__msg__UInt8MultiArray:
    """Class for example_interfaces/msg/UInt8MultiArray."""

    layout: example_interfaces__msg__MultiArrayLayout
    data: np.ndarray[tuple[int, ...], np.dtype[np.uint8]]
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/UInt8MultiArray'


@dataclass
class example_interfaces__msg__WString:
    """Class for example_interfaces/msg/WString."""

    data: example_interfaces__msg__wstring
    __msgtype__: ClassVar[str] = 'example_interfaces/msg/WString'


@dataclass
class example_interfaces__srv__AddTwoInts_Request:
    """Class for example_interfaces/srv/AddTwoInts_Request."""

    a: int
    b: int
    __msgtype__: ClassVar[str] = 'example_interfaces/srv/AddTwoInts_Request'


@dataclass
class example_interfaces__srv__AddTwoInts_Response:
    """Class for example_interfaces/srv/AddTwoInts_Response."""

    sum: int
    __msgtype__: ClassVar[str] = 'example_interfaces/srv/AddTwoInts_Response'


@dataclass
class example_interfaces__srv__SetBool_Request:
    """Class for example_interfaces/srv/SetBool_Request."""

    data: bool
    __msgtype__: ClassVar[str] = 'example_interfaces/srv/SetBool_Request'


@dataclass
class example_interfaces__srv__SetBool_Response:
    """Class for example_interfaces/srv/SetBool_Response."""

    success: bool
    message: str
    __msgtype__: ClassVar[str] = 'example_interfaces/srv/SetBool_Response'


@dataclass
class example_interfaces__srv__Trigger_Request:
    """Class for example_interfaces/srv/Trigger_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'example_interfaces/srv/Trigger_Request'


@dataclass
class example_interfaces__srv__Trigger_Response:
    """Class for example_interfaces/srv/Trigger_Response."""

    success: bool
    message: str
    __msgtype__: ClassVar[str] = 'example_interfaces/srv/Trigger_Response'


@dataclass
class example_interfaces__srv__msg__AddTwoInts_Request:
    """Class for example_interfaces/srv/msg/AddTwoInts_Request."""

    a: int
    b: int
    __msgtype__: ClassVar[str] = 'example_interfaces/srv/msg/AddTwoInts_Request'


@dataclass
class example_interfaces__srv__msg__AddTwoInts_Response:
    """Class for example_interfaces/srv/msg/AddTwoInts_Response."""

    sum: int
    __msgtype__: ClassVar[str] = 'example_interfaces/srv/msg/AddTwoInts_Response'


@dataclass
class example_interfaces__srv__msg__SetBool_Request:
    """Class for example_interfaces/srv/msg/SetBool_Request."""

    data: bool
    __msgtype__: ClassVar[str] = 'example_interfaces/srv/msg/SetBool_Request'


@dataclass
class example_interfaces__srv__msg__SetBool_Response:
    """Class for example_interfaces/srv/msg/SetBool_Response."""

    success: bool
    message: str
    __msgtype__: ClassVar[str] = 'example_interfaces/srv/msg/SetBool_Response'


@dataclass
class example_interfaces__srv__msg__Trigger_Request:
    """Class for example_interfaces/srv/msg/Trigger_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'example_interfaces/srv/msg/Trigger_Request'


@dataclass
class example_interfaces__srv__msg__Trigger_Response:
    """Class for example_interfaces/srv/msg/Trigger_Response."""

    success: bool
    message: str
    __msgtype__: ClassVar[str] = 'example_interfaces/srv/msg/Trigger_Response'


@dataclass
class geometry_msgs__msg__VelocityStamped:
    """Class for geometry_msgs/msg/VelocityStamped."""

    header: std_msgs__msg__Header
    body_frame_id: str
    reference_frame_id: str
    velocity: geometry_msgs__msg__Twist
    __msgtype__: ClassVar[str] = 'geometry_msgs/msg/VelocityStamped'


@dataclass
class lifecycle_msgs__srv__ChangeState_Request:
    """Class for lifecycle_msgs/srv/ChangeState_Request."""

    transition: lifecycle_msgs__msg__Transition
    __msgtype__: ClassVar[str] = 'lifecycle_msgs/srv/ChangeState_Request'


@dataclass
class lifecycle_msgs__srv__ChangeState_Response:
    """Class for lifecycle_msgs/srv/ChangeState_Response."""

    success: bool
    __msgtype__: ClassVar[str] = 'lifecycle_msgs/srv/ChangeState_Response'


@dataclass
class lifecycle_msgs__srv__GetAvailableStates_Request:
    """Class for lifecycle_msgs/srv/GetAvailableStates_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'lifecycle_msgs/srv/GetAvailableStates_Request'


@dataclass
class lifecycle_msgs__srv__GetAvailableStates_Response:
    """Class for lifecycle_msgs/srv/GetAvailableStates_Response."""

    available_states: list[lifecycle_msgs__msg__State]
    __msgtype__: ClassVar[str] = 'lifecycle_msgs/srv/GetAvailableStates_Response'


@dataclass
class lifecycle_msgs__srv__GetAvailableTransitions_Request:
    """Class for lifecycle_msgs/srv/GetAvailableTransitions_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'lifecycle_msgs/srv/GetAvailableTransitions_Request'


@dataclass
class lifecycle_msgs__srv__GetAvailableTransitions_Response:
    """Class for lifecycle_msgs/srv/GetAvailableTransitions_Response."""

    available_transitions: list[lifecycle_msgs__msg__TransitionDescription]
    __msgtype__: ClassVar[str] = 'lifecycle_msgs/srv/GetAvailableTransitions_Response'


@dataclass
class lifecycle_msgs__srv__GetState_Request:
    """Class for lifecycle_msgs/srv/GetState_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'lifecycle_msgs/srv/GetState_Request'


@dataclass
class lifecycle_msgs__srv__GetState_Response:
    """Class for lifecycle_msgs/srv/GetState_Response."""

    current_state: lifecycle_msgs__msg__State
    __msgtype__: ClassVar[str] = 'lifecycle_msgs/srv/GetState_Response'


@dataclass
class lifecycle_msgs__srv__msg__ChangeState_Request:
    """Class for lifecycle_msgs/srv/msg/ChangeState_Request."""

    transition: lifecycle_msgs__srv__msg__Transition
    __msgtype__: ClassVar[str] = 'lifecycle_msgs/srv/msg/ChangeState_Request'


@dataclass
class lifecycle_msgs__srv__msg__ChangeState_Response:
    """Class for lifecycle_msgs/srv/msg/ChangeState_Response."""

    success: bool
    __msgtype__: ClassVar[str] = 'lifecycle_msgs/srv/msg/ChangeState_Response'


@dataclass
class lifecycle_msgs__srv__msg__GetAvailableStates_Request:
    """Class for lifecycle_msgs/srv/msg/GetAvailableStates_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'lifecycle_msgs/srv/msg/GetAvailableStates_Request'


@dataclass
class lifecycle_msgs__srv__msg__GetAvailableStates_Response:
    """Class for lifecycle_msgs/srv/msg/GetAvailableStates_Response."""

    available_states: list[lifecycle_msgs__srv__msg__State]
    __msgtype__: ClassVar[str] = 'lifecycle_msgs/srv/msg/GetAvailableStates_Response'


@dataclass
class lifecycle_msgs__srv__msg__GetAvailableTransitions_Request:
    """Class for lifecycle_msgs/srv/msg/GetAvailableTransitions_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'lifecycle_msgs/srv/msg/GetAvailableTransitions_Request'


@dataclass
class lifecycle_msgs__srv__msg__GetAvailableTransitions_Response:
    """Class for lifecycle_msgs/srv/msg/GetAvailableTransitions_Response."""

    available_transitions: list[lifecycle_msgs__srv__msg__TransitionDescription]
    __msgtype__: ClassVar[str] = 'lifecycle_msgs/srv/msg/GetAvailableTransitions_Response'


@dataclass
class lifecycle_msgs__srv__msg__GetState_Request:
    """Class for lifecycle_msgs/srv/msg/GetState_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'lifecycle_msgs/srv/msg/GetState_Request'


@dataclass
class lifecycle_msgs__srv__msg__GetState_Response:
    """Class for lifecycle_msgs/srv/msg/GetState_Response."""

    current_state: lifecycle_msgs__srv__msg__State
    __msgtype__: ClassVar[str] = 'lifecycle_msgs/srv/msg/GetState_Response'


@dataclass
class logging_demo__srv__ConfigLogger_Request:
    """Class for logging_demo/srv/ConfigLogger_Request."""

    logger_name: str
    level: str
    __msgtype__: ClassVar[str] = 'logging_demo/srv/ConfigLogger_Request'


@dataclass
class logging_demo__srv__ConfigLogger_Response:
    """Class for logging_demo/srv/ConfigLogger_Response."""

    success: bool
    __msgtype__: ClassVar[str] = 'logging_demo/srv/ConfigLogger_Response'


@dataclass
class logging_demo__srv__msg__ConfigLogger_Request:
    """Class for logging_demo/srv/msg/ConfigLogger_Request."""

    logger_name: str
    level: str
    __msgtype__: ClassVar[str] = 'logging_demo/srv/msg/ConfigLogger_Request'


@dataclass
class logging_demo__srv__msg__ConfigLogger_Response:
    """Class for logging_demo/srv/msg/ConfigLogger_Response."""

    success: bool
    __msgtype__: ClassVar[str] = 'logging_demo/srv/msg/ConfigLogger_Response'


@dataclass
class map_msgs__msg__OccupancyGridUpdate:
    """Class for map_msgs/msg/OccupancyGridUpdate."""

    header: std_msgs__msg__Header
    x: int
    y: int
    width: int
    height: int
    data: np.ndarray[tuple[int, ...], np.dtype[np.int8]]
    __msgtype__: ClassVar[str] = 'map_msgs/msg/OccupancyGridUpdate'


@dataclass
class map_msgs__msg__PointCloud2Update:
    """Class for map_msgs/msg/PointCloud2Update."""

    header: std_msgs__msg__Header
    type: int
    points: sensor_msgs__msg__PointCloud2
    ADD: ClassVar[int] = 0
    DELETE: ClassVar[int] = 1
    __msgtype__: ClassVar[str] = 'map_msgs/msg/PointCloud2Update'


@dataclass
class map_msgs__msg__ProjectedMap:
    """Class for map_msgs/msg/ProjectedMap."""

    map: nav_msgs__msg__OccupancyGrid
    min_z: float
    max_z: float
    __msgtype__: ClassVar[str] = 'map_msgs/msg/ProjectedMap'


@dataclass
class map_msgs__msg__ProjectedMapInfo:
    """Class for map_msgs/msg/ProjectedMapInfo."""

    frame_id: str
    x: float
    y: float
    width: float
    height: float
    min_z: float
    max_z: float
    __msgtype__: ClassVar[str] = 'map_msgs/msg/ProjectedMapInfo'


@dataclass
class map_msgs__srv__GetMapROI_Request:
    """Class for map_msgs/srv/GetMapROI_Request."""

    x: float
    y: float
    l_x: float
    l_y: float
    __msgtype__: ClassVar[str] = 'map_msgs/srv/GetMapROI_Request'


@dataclass
class map_msgs__srv__GetMapROI_Response:
    """Class for map_msgs/srv/GetMapROI_Response."""

    sub_map: nav_msgs__msg__OccupancyGrid
    __msgtype__: ClassVar[str] = 'map_msgs/srv/GetMapROI_Response'


@dataclass
class map_msgs__srv__GetPointMapROI_Request:
    """Class for map_msgs/srv/GetPointMapROI_Request."""

    x: float
    y: float
    z: float
    r: float
    l_x: float
    l_y: float
    l_z: float
    __msgtype__: ClassVar[str] = 'map_msgs/srv/GetPointMapROI_Request'


@dataclass
class map_msgs__srv__GetPointMapROI_Response:
    """Class for map_msgs/srv/GetPointMapROI_Response."""

    sub_map: sensor_msgs__msg__PointCloud2
    __msgtype__: ClassVar[str] = 'map_msgs/srv/GetPointMapROI_Response'


@dataclass
class map_msgs__srv__GetPointMap_Request:
    """Class for map_msgs/srv/GetPointMap_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'map_msgs/srv/GetPointMap_Request'


@dataclass
class map_msgs__srv__GetPointMap_Response:
    """Class for map_msgs/srv/GetPointMap_Response."""

    map: sensor_msgs__msg__PointCloud2
    __msgtype__: ClassVar[str] = 'map_msgs/srv/GetPointMap_Response'


@dataclass
class map_msgs__srv__ProjectedMapsInfo_Request:
    """Class for map_msgs/srv/ProjectedMapsInfo_Request."""

    projected_maps_info: list[map_msgs__msg__ProjectedMapInfo]
    __msgtype__: ClassVar[str] = 'map_msgs/srv/ProjectedMapsInfo_Request'


@dataclass
class map_msgs__srv__ProjectedMapsInfo_Response:
    """Class for map_msgs/srv/ProjectedMapsInfo_Response."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'map_msgs/srv/ProjectedMapsInfo_Response'


@dataclass
class map_msgs__srv__SaveMap_Request:
    """Class for map_msgs/srv/SaveMap_Request."""

    filename: std_msgs__msg__String
    __msgtype__: ClassVar[str] = 'map_msgs/srv/SaveMap_Request'


@dataclass
class map_msgs__srv__SaveMap_Response:
    """Class for map_msgs/srv/SaveMap_Response."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'map_msgs/srv/SaveMap_Response'


@dataclass
class map_msgs__srv__SetMapProjections_Request:
    """Class for map_msgs/srv/SetMapProjections_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'map_msgs/srv/SetMapProjections_Request'


@dataclass
class map_msgs__srv__SetMapProjections_Response:
    """Class for map_msgs/srv/SetMapProjections_Response."""

    projected_maps_info: list[map_msgs__msg__ProjectedMapInfo]
    __msgtype__: ClassVar[str] = 'map_msgs/srv/SetMapProjections_Response'


@dataclass
class map_msgs__srv__msg__GetMapROI_Request:
    """Class for map_msgs/srv/msg/GetMapROI_Request."""

    x: float
    y: float
    l_x: float
    l_y: float
    __msgtype__: ClassVar[str] = 'map_msgs/srv/msg/GetMapROI_Request'


@dataclass
class map_msgs__srv__msg__GetMapROI_Response:
    """Class for map_msgs/srv/msg/GetMapROI_Response."""

    sub_map: nav_msgs__msg__OccupancyGrid
    __msgtype__: ClassVar[str] = 'map_msgs/srv/msg/GetMapROI_Response'


@dataclass
class map_msgs__srv__msg__GetPointMapROI_Request:
    """Class for map_msgs/srv/msg/GetPointMapROI_Request."""

    x: float
    y: float
    z: float
    r: float
    l_x: float
    l_y: float
    l_z: float
    __msgtype__: ClassVar[str] = 'map_msgs/srv/msg/GetPointMapROI_Request'


@dataclass
class map_msgs__srv__msg__GetPointMapROI_Response:
    """Class for map_msgs/srv/msg/GetPointMapROI_Response."""

    sub_map: sensor_msgs__msg__PointCloud2
    __msgtype__: ClassVar[str] = 'map_msgs/srv/msg/GetPointMapROI_Response'


@dataclass
class map_msgs__srv__msg__GetPointMap_Request:
    """Class for map_msgs/srv/msg/GetPointMap_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'map_msgs/srv/msg/GetPointMap_Request'


@dataclass
class map_msgs__srv__msg__GetPointMap_Response:
    """Class for map_msgs/srv/msg/GetPointMap_Response."""

    map: sensor_msgs__msg__PointCloud2
    __msgtype__: ClassVar[str] = 'map_msgs/srv/msg/GetPointMap_Response'


@dataclass
class map_msgs__srv__msg__ProjectedMapsInfo_Request:
    """Class for map_msgs/srv/msg/ProjectedMapsInfo_Request."""

    projected_maps_info: list[map_msgs__msg__ProjectedMapInfo]
    __msgtype__: ClassVar[str] = 'map_msgs/srv/msg/ProjectedMapsInfo_Request'


@dataclass
class map_msgs__srv__msg__ProjectedMapsInfo_Response:
    """Class for map_msgs/srv/msg/ProjectedMapsInfo_Response."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'map_msgs/srv/msg/ProjectedMapsInfo_Response'


@dataclass
class map_msgs__srv__msg__SaveMap_Request:
    """Class for map_msgs/srv/msg/SaveMap_Request."""

    filename: std_msgs__msg__String
    __msgtype__: ClassVar[str] = 'map_msgs/srv/msg/SaveMap_Request'


@dataclass
class map_msgs__srv__msg__SaveMap_Response:
    """Class for map_msgs/srv/msg/SaveMap_Response."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'map_msgs/srv/msg/SaveMap_Response'


@dataclass
class map_msgs__srv__msg__SetMapProjections_Request:
    """Class for map_msgs/srv/msg/SetMapProjections_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'map_msgs/srv/msg/SetMapProjections_Request'


@dataclass
class map_msgs__srv__msg__SetMapProjections_Response:
    """Class for map_msgs/srv/msg/SetMapProjections_Response."""

    projected_maps_info: list[map_msgs__msg__ProjectedMapInfo]
    __msgtype__: ClassVar[str] = 'map_msgs/srv/msg/SetMapProjections_Response'


@dataclass
class nav2_msgs__action__AssistedTeleop_Feedback:
    """Class for nav2_msgs/action/AssistedTeleop_Feedback."""

    current_teleop_duration: builtin_interfaces__msg__Duration
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/AssistedTeleop_Feedback'


@dataclass
class nav2_msgs__action__AssistedTeleop_Goal:
    """Class for nav2_msgs/action/AssistedTeleop_Goal."""

    time_allowance: builtin_interfaces__msg__Duration
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/AssistedTeleop_Goal'


@dataclass
class nav2_msgs__action__AssistedTeleop_Result:
    """Class for nav2_msgs/action/AssistedTeleop_Result."""

    total_elapsed_time: builtin_interfaces__msg__Duration
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/AssistedTeleop_Result'


@dataclass
class nav2_msgs__action__BackUp_Feedback:
    """Class for nav2_msgs/action/BackUp_Feedback."""

    distance_traveled: float
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/BackUp_Feedback'


@dataclass
class nav2_msgs__action__BackUp_Goal:
    """Class for nav2_msgs/action/BackUp_Goal."""

    target: geometry_msgs__msg__Point
    speed: float
    time_allowance: builtin_interfaces__msg__Duration
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/BackUp_Goal'


@dataclass
class nav2_msgs__action__BackUp_Result:
    """Class for nav2_msgs/action/BackUp_Result."""

    total_elapsed_time: builtin_interfaces__msg__Duration
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/BackUp_Result'


@dataclass
class nav2_msgs__action__ComputePathThroughPoses_Feedback:
    """Class for nav2_msgs/action/ComputePathThroughPoses_Feedback."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/ComputePathThroughPoses_Feedback'


@dataclass
class nav2_msgs__action__ComputePathThroughPoses_Goal:
    """Class for nav2_msgs/action/ComputePathThroughPoses_Goal."""

    goals: list[geometry_msgs__msg__PoseStamped]
    start: geometry_msgs__msg__PoseStamped
    planner_id: str
    use_start: bool
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/ComputePathThroughPoses_Goal'


@dataclass
class nav2_msgs__action__ComputePathThroughPoses_Result:
    """Class for nav2_msgs/action/ComputePathThroughPoses_Result."""

    path: nav_msgs__msg__Path
    planning_time: builtin_interfaces__msg__Duration
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/ComputePathThroughPoses_Result'


@dataclass
class nav2_msgs__action__ComputePathToPose_Feedback:
    """Class for nav2_msgs/action/ComputePathToPose_Feedback."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/ComputePathToPose_Feedback'


@dataclass
class nav2_msgs__action__ComputePathToPose_Goal:
    """Class for nav2_msgs/action/ComputePathToPose_Goal."""

    goal: geometry_msgs__msg__PoseStamped
    start: geometry_msgs__msg__PoseStamped
    planner_id: str
    use_start: bool
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/ComputePathToPose_Goal'


@dataclass
class nav2_msgs__action__ComputePathToPose_Result:
    """Class for nav2_msgs/action/ComputePathToPose_Result."""

    path: nav_msgs__msg__Path
    planning_time: builtin_interfaces__msg__Duration
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/ComputePathToPose_Result'


@dataclass
class nav2_msgs__action__DriveOnHeading_Feedback:
    """Class for nav2_msgs/action/DriveOnHeading_Feedback."""

    distance_traveled: float
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/DriveOnHeading_Feedback'


@dataclass
class nav2_msgs__action__DriveOnHeading_Goal:
    """Class for nav2_msgs/action/DriveOnHeading_Goal."""

    target: geometry_msgs__msg__Point
    speed: float
    time_allowance: builtin_interfaces__msg__Duration
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/DriveOnHeading_Goal'


@dataclass
class nav2_msgs__action__DriveOnHeading_Result:
    """Class for nav2_msgs/action/DriveOnHeading_Result."""

    total_elapsed_time: builtin_interfaces__msg__Duration
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/DriveOnHeading_Result'


@dataclass
class nav2_msgs__action__DummyBehavior_Feedback:
    """Class for nav2_msgs/action/DummyBehavior_Feedback."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/DummyBehavior_Feedback'


@dataclass
class nav2_msgs__action__DummyBehavior_Goal:
    """Class for nav2_msgs/action/DummyBehavior_Goal."""

    command: std_msgs__msg__String
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/DummyBehavior_Goal'


@dataclass
class nav2_msgs__action__DummyBehavior_Result:
    """Class for nav2_msgs/action/DummyBehavior_Result."""

    total_elapsed_time: builtin_interfaces__msg__Duration
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/DummyBehavior_Result'


@dataclass
class nav2_msgs__action__FollowPath_Feedback:
    """Class for nav2_msgs/action/FollowPath_Feedback."""

    distance_to_goal: float
    speed: float
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/FollowPath_Feedback'


@dataclass
class nav2_msgs__action__FollowPath_Goal:
    """Class for nav2_msgs/action/FollowPath_Goal."""

    path: nav_msgs__msg__Path
    controller_id: str
    goal_checker_id: str
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/FollowPath_Goal'


@dataclass
class nav2_msgs__action__FollowPath_Result:
    """Class for nav2_msgs/action/FollowPath_Result."""

    result: std_msgs__msg__Empty
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/FollowPath_Result'


@dataclass
class nav2_msgs__action__FollowWaypoints_Feedback:
    """Class for nav2_msgs/action/FollowWaypoints_Feedback."""

    current_waypoint: int
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/FollowWaypoints_Feedback'


@dataclass
class nav2_msgs__action__FollowWaypoints_Goal:
    """Class for nav2_msgs/action/FollowWaypoints_Goal."""

    poses: list[geometry_msgs__msg__PoseStamped]
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/FollowWaypoints_Goal'


@dataclass
class nav2_msgs__action__FollowWaypoints_Result:
    """Class for nav2_msgs/action/FollowWaypoints_Result."""

    missed_waypoints: np.ndarray[tuple[int, ...], np.dtype[np.int32]]
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/FollowWaypoints_Result'


@dataclass
class nav2_msgs__action__NavigateThroughPoses_Feedback:
    """Class for nav2_msgs/action/NavigateThroughPoses_Feedback."""

    current_pose: geometry_msgs__msg__PoseStamped
    navigation_time: builtin_interfaces__msg__Duration
    estimated_time_remaining: builtin_interfaces__msg__Duration
    number_of_recoveries: int
    distance_remaining: float
    number_of_poses_remaining: int
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/NavigateThroughPoses_Feedback'


@dataclass
class nav2_msgs__action__NavigateThroughPoses_Goal:
    """Class for nav2_msgs/action/NavigateThroughPoses_Goal."""

    poses: list[geometry_msgs__msg__PoseStamped]
    behavior_tree: str
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/NavigateThroughPoses_Goal'


@dataclass
class nav2_msgs__action__NavigateThroughPoses_Result:
    """Class for nav2_msgs/action/NavigateThroughPoses_Result."""

    result: std_msgs__msg__Empty
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/NavigateThroughPoses_Result'


@dataclass
class nav2_msgs__action__NavigateToPose_Feedback:
    """Class for nav2_msgs/action/NavigateToPose_Feedback."""

    current_pose: geometry_msgs__msg__PoseStamped
    navigation_time: builtin_interfaces__msg__Duration
    estimated_time_remaining: builtin_interfaces__msg__Duration
    number_of_recoveries: int
    distance_remaining: float
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/NavigateToPose_Feedback'


@dataclass
class nav2_msgs__action__NavigateToPose_Goal:
    """Class for nav2_msgs/action/NavigateToPose_Goal."""

    pose: geometry_msgs__msg__PoseStamped
    behavior_tree: str
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/NavigateToPose_Goal'


@dataclass
class nav2_msgs__action__NavigateToPose_Result:
    """Class for nav2_msgs/action/NavigateToPose_Result."""

    result: std_msgs__msg__Empty
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/NavigateToPose_Result'


@dataclass
class nav2_msgs__action__SmoothPath_Feedback:
    """Class for nav2_msgs/action/SmoothPath_Feedback."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/SmoothPath_Feedback'


@dataclass
class nav2_msgs__action__SmoothPath_Goal:
    """Class for nav2_msgs/action/SmoothPath_Goal."""

    path: nav_msgs__msg__Path
    smoother_id: str
    max_smoothing_duration: builtin_interfaces__msg__Duration
    check_for_collisions: bool
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/SmoothPath_Goal'


@dataclass
class nav2_msgs__action__SmoothPath_Result:
    """Class for nav2_msgs/action/SmoothPath_Result."""

    path: nav_msgs__msg__Path
    smoothing_duration: builtin_interfaces__msg__Duration
    was_completed: bool
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/SmoothPath_Result'


@dataclass
class nav2_msgs__action__Spin_Feedback:
    """Class for nav2_msgs/action/Spin_Feedback."""

    angular_distance_traveled: float
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/Spin_Feedback'


@dataclass
class nav2_msgs__action__Spin_Goal:
    """Class for nav2_msgs/action/Spin_Goal."""

    target_yaw: float
    time_allowance: builtin_interfaces__msg__Duration
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/Spin_Goal'


@dataclass
class nav2_msgs__action__Spin_Result:
    """Class for nav2_msgs/action/Spin_Result."""

    total_elapsed_time: builtin_interfaces__msg__Duration
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/Spin_Result'


@dataclass
class nav2_msgs__action__Wait_Feedback:
    """Class for nav2_msgs/action/Wait_Feedback."""

    time_left: builtin_interfaces__msg__Duration
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/Wait_Feedback'


@dataclass
class nav2_msgs__action__Wait_Goal:
    """Class for nav2_msgs/action/Wait_Goal."""

    time: builtin_interfaces__msg__Duration
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/Wait_Goal'


@dataclass
class nav2_msgs__action__Wait_Result:
    """Class for nav2_msgs/action/Wait_Result."""

    total_elapsed_time: builtin_interfaces__msg__Duration
    __msgtype__: ClassVar[str] = 'nav2_msgs/action/Wait_Result'


@dataclass
class nav2_msgs__msg__BehaviorTreeLog:
    """Class for nav2_msgs/msg/BehaviorTreeLog."""

    timestamp: builtin_interfaces__msg__Time
    event_log: list[nav2_msgs__msg__BehaviorTreeStatusChange]
    __msgtype__: ClassVar[str] = 'nav2_msgs/msg/BehaviorTreeLog'


@dataclass
class nav2_msgs__msg__BehaviorTreeStatusChange:
    """Class for nav2_msgs/msg/BehaviorTreeStatusChange."""

    timestamp: builtin_interfaces__msg__Time
    node_name: str
    previous_status: str
    current_status: str
    __msgtype__: ClassVar[str] = 'nav2_msgs/msg/BehaviorTreeStatusChange'


@dataclass
class nav2_msgs__msg__CollisionMonitorState:
    """Class for nav2_msgs/msg/CollisionMonitorState."""

    action_type: int
    polygon_name: str
    DO_NOTHING: ClassVar[int] = 0
    STOP: ClassVar[int] = 1
    SLOWDOWN: ClassVar[int] = 2
    APPROACH: ClassVar[int] = 3
    __msgtype__: ClassVar[str] = 'nav2_msgs/msg/CollisionMonitorState'


@dataclass
class nav2_msgs__msg__Costmap:
    """Class for nav2_msgs/msg/Costmap."""

    header: std_msgs__msg__Header
    metadata: nav2_msgs__msg__CostmapMetaData
    data: np.ndarray[tuple[int, ...], np.dtype[np.uint8]]
    __msgtype__: ClassVar[str] = 'nav2_msgs/msg/Costmap'


@dataclass
class nav2_msgs__msg__CostmapFilterInfo:
    """Class for nav2_msgs/msg/CostmapFilterInfo."""

    header: std_msgs__msg__Header
    type: int
    filter_mask_topic: str
    base: float
    multiplier: float
    __msgtype__: ClassVar[str] = 'nav2_msgs/msg/CostmapFilterInfo'


@dataclass
class nav2_msgs__msg__CostmapMetaData:
    """Class for nav2_msgs/msg/CostmapMetaData."""

    map_load_time: builtin_interfaces__msg__Time
    update_time: builtin_interfaces__msg__Time
    layer: str
    resolution: float
    size_x: int
    size_y: int
    origin: geometry_msgs__msg__Pose
    __msgtype__: ClassVar[str] = 'nav2_msgs/msg/CostmapMetaData'


@dataclass
class nav2_msgs__msg__Particle:
    """Class for nav2_msgs/msg/Particle."""

    pose: geometry_msgs__msg__Pose
    weight: float
    __msgtype__: ClassVar[str] = 'nav2_msgs/msg/Particle'


@dataclass
class nav2_msgs__msg__ParticleCloud:
    """Class for nav2_msgs/msg/ParticleCloud."""

    header: std_msgs__msg__Header
    particles: list[nav2_msgs__msg__Particle]
    __msgtype__: ClassVar[str] = 'nav2_msgs/msg/ParticleCloud'


@dataclass
class nav2_msgs__msg__SpeedLimit:
    """Class for nav2_msgs/msg/SpeedLimit."""

    header: std_msgs__msg__Header
    percentage: bool
    speed_limit: float
    __msgtype__: ClassVar[str] = 'nav2_msgs/msg/SpeedLimit'


@dataclass
class nav2_msgs__msg__VoxelGrid:
    """Class for nav2_msgs/msg/VoxelGrid."""

    header: std_msgs__msg__Header
    data: np.ndarray[tuple[int, ...], np.dtype[np.uint32]]
    origin: geometry_msgs__msg__Point32
    resolutions: geometry_msgs__msg__Vector3
    size_x: int
    size_y: int
    size_z: int
    __msgtype__: ClassVar[str] = 'nav2_msgs/msg/VoxelGrid'


@dataclass
class nav2_msgs__srv__ClearCostmapAroundRobot_Request:
    """Class for nav2_msgs/srv/ClearCostmapAroundRobot_Request."""

    reset_distance: float
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/ClearCostmapAroundRobot_Request'


@dataclass
class nav2_msgs__srv__ClearCostmapAroundRobot_Response:
    """Class for nav2_msgs/srv/ClearCostmapAroundRobot_Response."""

    response: std_msgs__msg__Empty
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/ClearCostmapAroundRobot_Response'


@dataclass
class nav2_msgs__srv__ClearCostmapExceptRegion_Request:
    """Class for nav2_msgs/srv/ClearCostmapExceptRegion_Request."""

    reset_distance: float
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/ClearCostmapExceptRegion_Request'


@dataclass
class nav2_msgs__srv__ClearCostmapExceptRegion_Response:
    """Class for nav2_msgs/srv/ClearCostmapExceptRegion_Response."""

    response: std_msgs__msg__Empty
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/ClearCostmapExceptRegion_Response'


@dataclass
class nav2_msgs__srv__ClearEntireCostmap_Request:
    """Class for nav2_msgs/srv/ClearEntireCostmap_Request."""

    request: std_msgs__msg__Empty
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/ClearEntireCostmap_Request'


@dataclass
class nav2_msgs__srv__ClearEntireCostmap_Response:
    """Class for nav2_msgs/srv/ClearEntireCostmap_Response."""

    response: std_msgs__msg__Empty
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/ClearEntireCostmap_Response'


@dataclass
class nav2_msgs__srv__GetCostmap_Request:
    """Class for nav2_msgs/srv/GetCostmap_Request."""

    specs: nav2_msgs__msg__CostmapMetaData
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/GetCostmap_Request'


@dataclass
class nav2_msgs__srv__GetCostmap_Response:
    """Class for nav2_msgs/srv/GetCostmap_Response."""

    map: nav2_msgs__msg__Costmap
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/GetCostmap_Response'


@dataclass
class nav2_msgs__srv__IsPathValid_Request:
    """Class for nav2_msgs/srv/IsPathValid_Request."""

    path: nav_msgs__msg__Path
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/IsPathValid_Request'


@dataclass
class nav2_msgs__srv__IsPathValid_Response:
    """Class for nav2_msgs/srv/IsPathValid_Response."""

    is_valid: bool
    invalid_pose_indices: np.ndarray[tuple[int, ...], np.dtype[np.int32]]
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/IsPathValid_Response'


@dataclass
class nav2_msgs__srv__LoadMap_Request:
    """Class for nav2_msgs/srv/LoadMap_Request."""

    map_url: str
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/LoadMap_Request'


@dataclass
class nav2_msgs__srv__LoadMap_Response:
    """Class for nav2_msgs/srv/LoadMap_Response."""

    map: nav_msgs__msg__OccupancyGrid
    result: int
    RESULT_SUCCESS: ClassVar[int] = 0
    RESULT_MAP_DOES_NOT_EXIST: ClassVar[int] = 1
    RESULT_INVALID_MAP_DATA: ClassVar[int] = 2
    RESULT_INVALID_MAP_METADATA: ClassVar[int] = 3
    RESULT_UNDEFINED_FAILURE: ClassVar[int] = 255
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/LoadMap_Response'


@dataclass
class nav2_msgs__srv__ManageLifecycleNodes_Request:
    """Class for nav2_msgs/srv/ManageLifecycleNodes_Request."""

    command: int
    STARTUP: ClassVar[int] = 0
    PAUSE: ClassVar[int] = 1
    RESUME: ClassVar[int] = 2
    RESET: ClassVar[int] = 3
    SHUTDOWN: ClassVar[int] = 4
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/ManageLifecycleNodes_Request'


@dataclass
class nav2_msgs__srv__ManageLifecycleNodes_Response:
    """Class for nav2_msgs/srv/ManageLifecycleNodes_Response."""

    success: bool
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/ManageLifecycleNodes_Response'


@dataclass
class nav2_msgs__srv__SaveMap_Request:
    """Class for nav2_msgs/srv/SaveMap_Request."""

    map_topic: str
    map_url: str
    image_format: str
    map_mode: str
    free_thresh: float
    occupied_thresh: float
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/SaveMap_Request'


@dataclass
class nav2_msgs__srv__SaveMap_Response:
    """Class for nav2_msgs/srv/SaveMap_Response."""

    result: bool
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/SaveMap_Response'


@dataclass
class nav2_msgs__srv__SetInitialPose_Request:
    """Class for nav2_msgs/srv/SetInitialPose_Request."""

    pose: geometry_msgs__msg__PoseWithCovarianceStamped
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/SetInitialPose_Request'


@dataclass
class nav2_msgs__srv__SetInitialPose_Response:
    """Class for nav2_msgs/srv/SetInitialPose_Response."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/SetInitialPose_Response'


@dataclass
class nav2_msgs__srv__msg__ClearCostmapAroundRobot_Request:
    """Class for nav2_msgs/srv/msg/ClearCostmapAroundRobot_Request."""

    reset_distance: float
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/msg/ClearCostmapAroundRobot_Request'


@dataclass
class nav2_msgs__srv__msg__ClearCostmapAroundRobot_Response:
    """Class for nav2_msgs/srv/msg/ClearCostmapAroundRobot_Response."""

    response: std_msgs__msg__Empty
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/msg/ClearCostmapAroundRobot_Response'


@dataclass
class nav2_msgs__srv__msg__ClearCostmapExceptRegion_Request:
    """Class for nav2_msgs/srv/msg/ClearCostmapExceptRegion_Request."""

    reset_distance: float
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/msg/ClearCostmapExceptRegion_Request'


@dataclass
class nav2_msgs__srv__msg__ClearCostmapExceptRegion_Response:
    """Class for nav2_msgs/srv/msg/ClearCostmapExceptRegion_Response."""

    response: std_msgs__msg__Empty
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/msg/ClearCostmapExceptRegion_Response'


@dataclass
class nav2_msgs__srv__msg__ClearEntireCostmap_Request:
    """Class for nav2_msgs/srv/msg/ClearEntireCostmap_Request."""

    request: std_msgs__msg__Empty
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/msg/ClearEntireCostmap_Request'


@dataclass
class nav2_msgs__srv__msg__ClearEntireCostmap_Response:
    """Class for nav2_msgs/srv/msg/ClearEntireCostmap_Response."""

    response: std_msgs__msg__Empty
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/msg/ClearEntireCostmap_Response'


@dataclass
class nav2_msgs__srv__msg__GetCostmap_Request:
    """Class for nav2_msgs/srv/msg/GetCostmap_Request."""

    specs: nav2_msgs__msg__CostmapMetaData
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/msg/GetCostmap_Request'


@dataclass
class nav2_msgs__srv__msg__GetCostmap_Response:
    """Class for nav2_msgs/srv/msg/GetCostmap_Response."""

    map: nav2_msgs__msg__Costmap
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/msg/GetCostmap_Response'


@dataclass
class nav2_msgs__srv__msg__IsPathValid_Request:
    """Class for nav2_msgs/srv/msg/IsPathValid_Request."""

    path: nav_msgs__msg__Path
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/msg/IsPathValid_Request'


@dataclass
class nav2_msgs__srv__msg__IsPathValid_Response:
    """Class for nav2_msgs/srv/msg/IsPathValid_Response."""

    is_valid: bool
    invalid_pose_indices: np.ndarray[tuple[int, ...], np.dtype[np.int32]]
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/msg/IsPathValid_Response'


@dataclass
class nav2_msgs__srv__msg__LoadMap_Request:
    """Class for nav2_msgs/srv/msg/LoadMap_Request."""

    map_url: str
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/msg/LoadMap_Request'


@dataclass
class nav2_msgs__srv__msg__LoadMap_Response:
    """Class for nav2_msgs/srv/msg/LoadMap_Response."""

    map: nav_msgs__msg__OccupancyGrid
    result: int
    RESULT_SUCCESS: ClassVar[int] = 0
    RESULT_MAP_DOES_NOT_EXIST: ClassVar[int] = 1
    RESULT_INVALID_MAP_DATA: ClassVar[int] = 2
    RESULT_INVALID_MAP_METADATA: ClassVar[int] = 3
    RESULT_UNDEFINED_FAILURE: ClassVar[int] = 255
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/msg/LoadMap_Response'


@dataclass
class nav2_msgs__srv__msg__ManageLifecycleNodes_Request:
    """Class for nav2_msgs/srv/msg/ManageLifecycleNodes_Request."""

    command: int
    STARTUP: ClassVar[int] = 0
    PAUSE: ClassVar[int] = 1
    RESUME: ClassVar[int] = 2
    RESET: ClassVar[int] = 3
    SHUTDOWN: ClassVar[int] = 4
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/msg/ManageLifecycleNodes_Request'


@dataclass
class nav2_msgs__srv__msg__ManageLifecycleNodes_Response:
    """Class for nav2_msgs/srv/msg/ManageLifecycleNodes_Response."""

    success: bool
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/msg/ManageLifecycleNodes_Response'


@dataclass
class nav2_msgs__srv__msg__SaveMap_Request:
    """Class for nav2_msgs/srv/msg/SaveMap_Request."""

    map_topic: str
    map_url: str
    image_format: str
    map_mode: str
    free_thresh: float
    occupied_thresh: float
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/msg/SaveMap_Request'


@dataclass
class nav2_msgs__srv__msg__SaveMap_Response:
    """Class for nav2_msgs/srv/msg/SaveMap_Response."""

    result: bool
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/msg/SaveMap_Response'


@dataclass
class nav2_msgs__srv__msg__SetInitialPose_Request:
    """Class for nav2_msgs/srv/msg/SetInitialPose_Request."""

    pose: geometry_msgs__msg__PoseWithCovarianceStamped
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/msg/SetInitialPose_Request'


@dataclass
class nav2_msgs__srv__msg__SetInitialPose_Response:
    """Class for nav2_msgs/srv/msg/SetInitialPose_Response."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'nav2_msgs/srv/msg/SetInitialPose_Response'


@dataclass
class nav_2d_msgs__msg__Path2D:
    """Class for nav_2d_msgs/msg/Path2D."""

    header: std_msgs__msg__Header
    poses: list[geometry_msgs__msg__Pose2D]
    __msgtype__: ClassVar[str] = 'nav_2d_msgs/msg/Path2D'


@dataclass
class nav_2d_msgs__msg__Pose2D32:
    """Class for nav_2d_msgs/msg/Pose2D32."""

    x: float
    y: float
    theta: float
    __msgtype__: ClassVar[str] = 'nav_2d_msgs/msg/Pose2D32'


@dataclass
class nav_2d_msgs__msg__Pose2DStamped:
    """Class for nav_2d_msgs/msg/Pose2DStamped."""

    header: std_msgs__msg__Header
    pose: geometry_msgs__msg__Pose2D
    __msgtype__: ClassVar[str] = 'nav_2d_msgs/msg/Pose2DStamped'


@dataclass
class nav_2d_msgs__msg__Twist2D:
    """Class for nav_2d_msgs/msg/Twist2D."""

    x: float
    y: float
    theta: float
    __msgtype__: ClassVar[str] = 'nav_2d_msgs/msg/Twist2D'


@dataclass
class nav_2d_msgs__msg__Twist2D32:
    """Class for nav_2d_msgs/msg/Twist2D32."""

    x: float
    y: float
    theta: float
    __msgtype__: ClassVar[str] = 'nav_2d_msgs/msg/Twist2D32'


@dataclass
class nav_2d_msgs__msg__Twist2DStamped:
    """Class for nav_2d_msgs/msg/Twist2DStamped."""

    header: std_msgs__msg__Header
    velocity: nav_2d_msgs__msg__Twist2D
    __msgtype__: ClassVar[str] = 'nav_2d_msgs/msg/Twist2DStamped'


@dataclass
class nav_msgs__msg__Goals:
    """Class for nav_msgs/msg/Goals."""

    header: std_msgs__msg__Header
    goals: list[geometry_msgs__msg__PoseStamped]
    __msgtype__: ClassVar[str] = 'nav_msgs/msg/Goals'


@dataclass
class nav_msgs__srv__GetMap_Request:
    """Class for nav_msgs/srv/GetMap_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'nav_msgs/srv/GetMap_Request'


@dataclass
class nav_msgs__srv__GetMap_Response:
    """Class for nav_msgs/srv/GetMap_Response."""

    map: nav_msgs__msg__OccupancyGrid
    __msgtype__: ClassVar[str] = 'nav_msgs/srv/GetMap_Response'


@dataclass
class nav_msgs__srv__GetPlan_Request:
    """Class for nav_msgs/srv/GetPlan_Request."""

    start: geometry_msgs__msg__PoseStamped
    goal: geometry_msgs__msg__PoseStamped
    tolerance: float
    __msgtype__: ClassVar[str] = 'nav_msgs/srv/GetPlan_Request'


@dataclass
class nav_msgs__srv__GetPlan_Response:
    """Class for nav_msgs/srv/GetPlan_Response."""

    plan: nav_msgs__msg__Path
    __msgtype__: ClassVar[str] = 'nav_msgs/srv/GetPlan_Response'


@dataclass
class nav_msgs__srv__LoadMap_Request:
    """Class for nav_msgs/srv/LoadMap_Request."""

    map_url: str
    __msgtype__: ClassVar[str] = 'nav_msgs/srv/LoadMap_Request'


@dataclass
class nav_msgs__srv__LoadMap_Response:
    """Class for nav_msgs/srv/LoadMap_Response."""

    map: nav_msgs__msg__OccupancyGrid
    result: int
    RESULT_SUCCESS: ClassVar[int] = 0
    RESULT_MAP_DOES_NOT_EXIST: ClassVar[int] = 1
    RESULT_INVALID_MAP_DATA: ClassVar[int] = 2
    RESULT_INVALID_MAP_METADATA: ClassVar[int] = 3
    RESULT_UNDEFINED_FAILURE: ClassVar[int] = 255
    __msgtype__: ClassVar[str] = 'nav_msgs/srv/LoadMap_Response'


@dataclass
class nav_msgs__srv__SetMap_Request:
    """Class for nav_msgs/srv/SetMap_Request."""

    map: nav_msgs__msg__OccupancyGrid
    initial_pose: geometry_msgs__msg__PoseWithCovarianceStamped
    __msgtype__: ClassVar[str] = 'nav_msgs/srv/SetMap_Request'


@dataclass
class nav_msgs__srv__SetMap_Response:
    """Class for nav_msgs/srv/SetMap_Response."""

    success: bool
    __msgtype__: ClassVar[str] = 'nav_msgs/srv/SetMap_Response'


@dataclass
class nav_msgs__srv__msg__GetMap_Request:
    """Class for nav_msgs/srv/msg/GetMap_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'nav_msgs/srv/msg/GetMap_Request'


@dataclass
class nav_msgs__srv__msg__GetMap_Response:
    """Class for nav_msgs/srv/msg/GetMap_Response."""

    map: nav_msgs__srv__msg__OccupancyGrid
    __msgtype__: ClassVar[str] = 'nav_msgs/srv/msg/GetMap_Response'


@dataclass
class nav_msgs__srv__msg__GetPlan_Request:
    """Class for nav_msgs/srv/msg/GetPlan_Request."""

    start: geometry_msgs__msg__PoseStamped
    goal: geometry_msgs__msg__PoseStamped
    tolerance: float
    __msgtype__: ClassVar[str] = 'nav_msgs/srv/msg/GetPlan_Request'


@dataclass
class nav_msgs__srv__msg__GetPlan_Response:
    """Class for nav_msgs/srv/msg/GetPlan_Response."""

    plan: nav_msgs__srv__msg__Path
    __msgtype__: ClassVar[str] = 'nav_msgs/srv/msg/GetPlan_Response'


@dataclass
class nav_msgs__srv__msg__LoadMap_Request:
    """Class for nav_msgs/srv/msg/LoadMap_Request."""

    map_url: str
    __msgtype__: ClassVar[str] = 'nav_msgs/srv/msg/LoadMap_Request'


@dataclass
class nav_msgs__srv__msg__LoadMap_Response:
    """Class for nav_msgs/srv/msg/LoadMap_Response."""

    map: nav_msgs__msg__OccupancyGrid
    result: int
    RESULT_SUCCESS: ClassVar[int] = 0
    RESULT_MAP_DOES_NOT_EXIST: ClassVar[int] = 1
    RESULT_INVALID_MAP_DATA: ClassVar[int] = 2
    RESULT_INVALID_MAP_METADATA: ClassVar[int] = 3
    RESULT_UNDEFINED_FAILURE: ClassVar[int] = 255
    __msgtype__: ClassVar[str] = 'nav_msgs/srv/msg/LoadMap_Response'


@dataclass
class nav_msgs__srv__msg__SetMap_Request:
    """Class for nav_msgs/srv/msg/SetMap_Request."""

    map: nav_msgs__msg__OccupancyGrid
    initial_pose: geometry_msgs__msg__PoseWithCovarianceStamped
    __msgtype__: ClassVar[str] = 'nav_msgs/srv/msg/SetMap_Request'


@dataclass
class nav_msgs__srv__msg__SetMap_Response:
    """Class for nav_msgs/srv/msg/SetMap_Response."""

    success: bool
    __msgtype__: ClassVar[str] = 'nav_msgs/srv/msg/SetMap_Response'


@dataclass
class pcl_msgs__msg__ModelCoefficients:
    """Class for pcl_msgs/msg/ModelCoefficients."""

    header: std_msgs__msg__Header
    values: np.ndarray[tuple[int, ...], np.dtype[np.float32]]
    __msgtype__: ClassVar[str] = 'pcl_msgs/msg/ModelCoefficients'


@dataclass
class pcl_msgs__msg__PointIndices:
    """Class for pcl_msgs/msg/PointIndices."""

    header: std_msgs__msg__Header
    indices: np.ndarray[tuple[int, ...], np.dtype[np.int32]]
    __msgtype__: ClassVar[str] = 'pcl_msgs/msg/PointIndices'


@dataclass
class pcl_msgs__msg__PolygonMesh:
    """Class for pcl_msgs/msg/PolygonMesh."""

    header: std_msgs__msg__Header
    cloud: sensor_msgs__msg__PointCloud2
    polygons: list[pcl_msgs__msg__Vertices]
    __msgtype__: ClassVar[str] = 'pcl_msgs/msg/PolygonMesh'


@dataclass
class pcl_msgs__msg__Vertices:
    """Class for pcl_msgs/msg/Vertices."""

    vertices: np.ndarray[tuple[int, ...], np.dtype[np.uint32]]
    __msgtype__: ClassVar[str] = 'pcl_msgs/msg/Vertices'


@dataclass
class pcl_msgs__srv__UpdateFilename_Request:
    """Class for pcl_msgs/srv/UpdateFilename_Request."""

    filename: str
    __msgtype__: ClassVar[str] = 'pcl_msgs/srv/UpdateFilename_Request'


@dataclass
class pcl_msgs__srv__UpdateFilename_Response:
    """Class for pcl_msgs/srv/UpdateFilename_Response."""

    success: bool
    __msgtype__: ClassVar[str] = 'pcl_msgs/srv/UpdateFilename_Response'


@dataclass
class pcl_msgs__srv__msg__UpdateFilename_Request:
    """Class for pcl_msgs/srv/msg/UpdateFilename_Request."""

    filename: str
    __msgtype__: ClassVar[str] = 'pcl_msgs/srv/msg/UpdateFilename_Request'


@dataclass
class pcl_msgs__srv__msg__UpdateFilename_Response:
    """Class for pcl_msgs/srv/msg/UpdateFilename_Response."""

    success: bool
    __msgtype__: ClassVar[str] = 'pcl_msgs/srv/msg/UpdateFilename_Response'


@dataclass
class pendulum_msgs__msg__JointCommand:
    """Class for pendulum_msgs/msg/JointCommand."""

    position: float
    __msgtype__: ClassVar[str] = 'pendulum_msgs/msg/JointCommand'


@dataclass
class pendulum_msgs__msg__JointState:
    """Class for pendulum_msgs/msg/JointState."""

    position: float
    velocity: float
    effort: float
    __msgtype__: ClassVar[str] = 'pendulum_msgs/msg/JointState'


@dataclass
class pendulum_msgs__msg__RttestResults:
    """Class for pendulum_msgs/msg/RttestResults."""

    stamp: builtin_interfaces__msg__Time
    command: pendulum_msgs__msg__JointCommand
    state: pendulum_msgs__msg__JointState
    cur_latency: int
    mean_latency: float
    min_latency: int
    max_latency: int
    minor_pagefaults: int
    major_pagefaults: int
    __msgtype__: ClassVar[str] = 'pendulum_msgs/msg/RttestResults'


@dataclass
class rcl_interfaces__msg__ParameterValue:
    """Class for rcl_interfaces/msg/ParameterValue."""

    type: int
    bool_value: bool
    integer_value: int
    double_value: float
    string_value: str
    byte_array_value: list[octet]
    bool_array_value: np.ndarray[tuple[int, ...], np.dtype[np.bool_]]
    integer_array_value: np.ndarray[tuple[int, ...], np.dtype[np.int64]]
    double_array_value: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    string_array_value: list[str]
    __msgtype__: ClassVar[str] = 'rcl_interfaces/msg/ParameterValue'


@dataclass
class rcl_interfaces__srv__DescribeParameters_Request:
    """Class for rcl_interfaces/srv/DescribeParameters_Request."""

    names: list[str]
    __msgtype__: ClassVar[str] = 'rcl_interfaces/srv/DescribeParameters_Request'


@dataclass
class rcl_interfaces__srv__DescribeParameters_Response:
    """Class for rcl_interfaces/srv/DescribeParameters_Response."""

    descriptors: list[rcl_interfaces__msg__ParameterDescriptor]
    __msgtype__: ClassVar[str] = 'rcl_interfaces/srv/DescribeParameters_Response'


@dataclass
class rcl_interfaces__srv__GetParameterTypes_Request:
    """Class for rcl_interfaces/srv/GetParameterTypes_Request."""

    names: list[str]
    __msgtype__: ClassVar[str] = 'rcl_interfaces/srv/GetParameterTypes_Request'


@dataclass
class rcl_interfaces__srv__GetParameterTypes_Response:
    """Class for rcl_interfaces/srv/GetParameterTypes_Response."""

    types: np.ndarray[tuple[int, ...], np.dtype[np.uint8]]
    __msgtype__: ClassVar[str] = 'rcl_interfaces/srv/GetParameterTypes_Response'


@dataclass
class rcl_interfaces__srv__GetParameters_Request:
    """Class for rcl_interfaces/srv/GetParameters_Request."""

    names: list[str]
    __msgtype__: ClassVar[str] = 'rcl_interfaces/srv/GetParameters_Request'


@dataclass
class rcl_interfaces__srv__GetParameters_Response:
    """Class for rcl_interfaces/srv/GetParameters_Response."""

    values: list[rcl_interfaces__msg__ParameterValue]
    __msgtype__: ClassVar[str] = 'rcl_interfaces/srv/GetParameters_Response'


@dataclass
class rcl_interfaces__srv__ListParameters_Request:
    """Class for rcl_interfaces/srv/ListParameters_Request."""

    prefixes: list[str]
    depth: int
    DEPTH_RECURSIVE: ClassVar[int] = 0
    __msgtype__: ClassVar[str] = 'rcl_interfaces/srv/ListParameters_Request'


@dataclass
class rcl_interfaces__srv__ListParameters_Response:
    """Class for rcl_interfaces/srv/ListParameters_Response."""

    result: rcl_interfaces__msg__ListParametersResult
    __msgtype__: ClassVar[str] = 'rcl_interfaces/srv/ListParameters_Response'


@dataclass
class rcl_interfaces__srv__SetParametersAtomically_Request:
    """Class for rcl_interfaces/srv/SetParametersAtomically_Request."""

    parameters: list[rcl_interfaces__msg__Parameter]
    __msgtype__: ClassVar[str] = 'rcl_interfaces/srv/SetParametersAtomically_Request'


@dataclass
class rcl_interfaces__srv__SetParametersAtomically_Response:
    """Class for rcl_interfaces/srv/SetParametersAtomically_Response."""

    result: rcl_interfaces__msg__SetParametersResult
    __msgtype__: ClassVar[str] = 'rcl_interfaces/srv/SetParametersAtomically_Response'


@dataclass
class rcl_interfaces__srv__SetParameters_Request:
    """Class for rcl_interfaces/srv/SetParameters_Request."""

    parameters: list[rcl_interfaces__msg__Parameter]
    __msgtype__: ClassVar[str] = 'rcl_interfaces/srv/SetParameters_Request'


@dataclass
class rcl_interfaces__srv__SetParameters_Response:
    """Class for rcl_interfaces/srv/SetParameters_Response."""

    results: list[rcl_interfaces__msg__SetParametersResult]
    __msgtype__: ClassVar[str] = 'rcl_interfaces/srv/SetParameters_Response'


@dataclass
class rcl_interfaces__srv__msg__DescribeParameters_Request:
    """Class for rcl_interfaces/srv/msg/DescribeParameters_Request."""

    names: list[str]
    __msgtype__: ClassVar[str] = 'rcl_interfaces/srv/msg/DescribeParameters_Request'


@dataclass
class rcl_interfaces__srv__msg__DescribeParameters_Response:
    """Class for rcl_interfaces/srv/msg/DescribeParameters_Response."""

    descriptors: list[rcl_interfaces__srv__msg__ParameterDescriptor]
    __msgtype__: ClassVar[str] = 'rcl_interfaces/srv/msg/DescribeParameters_Response'


@dataclass
class rcl_interfaces__srv__msg__GetParameterTypes_Request:
    """Class for rcl_interfaces/srv/msg/GetParameterTypes_Request."""

    names: list[str]
    __msgtype__: ClassVar[str] = 'rcl_interfaces/srv/msg/GetParameterTypes_Request'


@dataclass
class rcl_interfaces__srv__msg__GetParameterTypes_Response:
    """Class for rcl_interfaces/srv/msg/GetParameterTypes_Response."""

    types: np.ndarray[tuple[int, ...], np.dtype[np.uint8]]
    __msgtype__: ClassVar[str] = 'rcl_interfaces/srv/msg/GetParameterTypes_Response'


@dataclass
class rcl_interfaces__srv__msg__GetParameters_Request:
    """Class for rcl_interfaces/srv/msg/GetParameters_Request."""

    names: list[str]
    __msgtype__: ClassVar[str] = 'rcl_interfaces/srv/msg/GetParameters_Request'


@dataclass
class rcl_interfaces__srv__msg__GetParameters_Response:
    """Class for rcl_interfaces/srv/msg/GetParameters_Response."""

    values: list[rcl_interfaces__srv__msg__ParameterValue]
    __msgtype__: ClassVar[str] = 'rcl_interfaces/srv/msg/GetParameters_Response'


@dataclass
class rcl_interfaces__srv__msg__ListParameters_Request:
    """Class for rcl_interfaces/srv/msg/ListParameters_Request."""

    prefixes: list[str]
    depth: int
    DEPTH_RECURSIVE: ClassVar[int] = 0
    __msgtype__: ClassVar[str] = 'rcl_interfaces/srv/msg/ListParameters_Request'


@dataclass
class rcl_interfaces__srv__msg__ListParameters_Response:
    """Class for rcl_interfaces/srv/msg/ListParameters_Response."""

    result: rcl_interfaces__srv__msg__ListParametersResult
    __msgtype__: ClassVar[str] = 'rcl_interfaces/srv/msg/ListParameters_Response'


@dataclass
class rcl_interfaces__srv__msg__SetParametersAtomically_Request:
    """Class for rcl_interfaces/srv/msg/SetParametersAtomically_Request."""

    parameters: list[rcl_interfaces__srv__msg__Parameter]
    __msgtype__: ClassVar[str] = 'rcl_interfaces/srv/msg/SetParametersAtomically_Request'


@dataclass
class rcl_interfaces__srv__msg__SetParametersAtomically_Response:
    """Class for rcl_interfaces/srv/msg/SetParametersAtomically_Response."""

    result: rcl_interfaces__srv__msg__SetParametersResult
    __msgtype__: ClassVar[str] = 'rcl_interfaces/srv/msg/SetParametersAtomically_Response'


@dataclass
class rcl_interfaces__srv__msg__SetParameters_Request:
    """Class for rcl_interfaces/srv/msg/SetParameters_Request."""

    parameters: list[rcl_interfaces__srv__msg__Parameter]
    __msgtype__: ClassVar[str] = 'rcl_interfaces/srv/msg/SetParameters_Request'


@dataclass
class rcl_interfaces__srv__msg__SetParameters_Response:
    """Class for rcl_interfaces/srv/msg/SetParameters_Response."""

    results: list[rcl_interfaces__srv__msg__SetParametersResult]
    __msgtype__: ClassVar[str] = 'rcl_interfaces/srv/msg/SetParameters_Response'


@dataclass
class rosbag2_interfaces__srv__Burst_Request:
    """Class for rosbag2_interfaces/srv/Burst_Request."""

    num_messages: int
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/Burst_Request'


@dataclass
class rosbag2_interfaces__srv__Burst_Response:
    """Class for rosbag2_interfaces/srv/Burst_Response."""

    actually_burst: int
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/Burst_Response'


@dataclass
class rosbag2_interfaces__srv__GetRate_Request:
    """Class for rosbag2_interfaces/srv/GetRate_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/GetRate_Request'


@dataclass
class rosbag2_interfaces__srv__GetRate_Response:
    """Class for rosbag2_interfaces/srv/GetRate_Response."""

    rate: float
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/GetRate_Response'


@dataclass
class rosbag2_interfaces__srv__IsPaused_Request:
    """Class for rosbag2_interfaces/srv/IsPaused_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/IsPaused_Request'


@dataclass
class rosbag2_interfaces__srv__IsPaused_Response:
    """Class for rosbag2_interfaces/srv/IsPaused_Response."""

    paused: bool
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/IsPaused_Response'


@dataclass
class rosbag2_interfaces__srv__Pause_Request:
    """Class for rosbag2_interfaces/srv/Pause_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/Pause_Request'


@dataclass
class rosbag2_interfaces__srv__Pause_Response:
    """Class for rosbag2_interfaces/srv/Pause_Response."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/Pause_Response'


@dataclass
class rosbag2_interfaces__srv__PlayNext_Request:
    """Class for rosbag2_interfaces/srv/PlayNext_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/PlayNext_Request'


@dataclass
class rosbag2_interfaces__srv__PlayNext_Response:
    """Class for rosbag2_interfaces/srv/PlayNext_Response."""

    success: bool
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/PlayNext_Response'


@dataclass
class rosbag2_interfaces__srv__Resume_Request:
    """Class for rosbag2_interfaces/srv/Resume_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/Resume_Request'


@dataclass
class rosbag2_interfaces__srv__Resume_Response:
    """Class for rosbag2_interfaces/srv/Resume_Response."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/Resume_Response'


@dataclass
class rosbag2_interfaces__srv__Seek_Request:
    """Class for rosbag2_interfaces/srv/Seek_Request."""

    time: builtin_interfaces__msg__Time
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/Seek_Request'


@dataclass
class rosbag2_interfaces__srv__Seek_Response:
    """Class for rosbag2_interfaces/srv/Seek_Response."""

    success: bool
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/Seek_Response'


@dataclass
class rosbag2_interfaces__srv__SetRate_Request:
    """Class for rosbag2_interfaces/srv/SetRate_Request."""

    rate: float
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/SetRate_Request'


@dataclass
class rosbag2_interfaces__srv__SetRate_Response:
    """Class for rosbag2_interfaces/srv/SetRate_Response."""

    success: bool
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/SetRate_Response'


@dataclass
class rosbag2_interfaces__srv__Snapshot_Request:
    """Class for rosbag2_interfaces/srv/Snapshot_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/Snapshot_Request'


@dataclass
class rosbag2_interfaces__srv__Snapshot_Response:
    """Class for rosbag2_interfaces/srv/Snapshot_Response."""

    success: bool
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/Snapshot_Response'


@dataclass
class rosbag2_interfaces__srv__TogglePaused_Request:
    """Class for rosbag2_interfaces/srv/TogglePaused_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/TogglePaused_Request'


@dataclass
class rosbag2_interfaces__srv__TogglePaused_Response:
    """Class for rosbag2_interfaces/srv/TogglePaused_Response."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/TogglePaused_Response'


@dataclass
class rosbag2_interfaces__srv__msg__Burst_Request:
    """Class for rosbag2_interfaces/srv/msg/Burst_Request."""

    num_messages: int
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/msg/Burst_Request'


@dataclass
class rosbag2_interfaces__srv__msg__Burst_Response:
    """Class for rosbag2_interfaces/srv/msg/Burst_Response."""

    actually_burst: int
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/msg/Burst_Response'


@dataclass
class rosbag2_interfaces__srv__msg__GetRate_Request:
    """Class for rosbag2_interfaces/srv/msg/GetRate_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/msg/GetRate_Request'


@dataclass
class rosbag2_interfaces__srv__msg__GetRate_Response:
    """Class for rosbag2_interfaces/srv/msg/GetRate_Response."""

    rate: float
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/msg/GetRate_Response'


@dataclass
class rosbag2_interfaces__srv__msg__IsPaused_Request:
    """Class for rosbag2_interfaces/srv/msg/IsPaused_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/msg/IsPaused_Request'


@dataclass
class rosbag2_interfaces__srv__msg__IsPaused_Response:
    """Class for rosbag2_interfaces/srv/msg/IsPaused_Response."""

    paused: bool
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/msg/IsPaused_Response'


@dataclass
class rosbag2_interfaces__srv__msg__Pause_Request:
    """Class for rosbag2_interfaces/srv/msg/Pause_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/msg/Pause_Request'


@dataclass
class rosbag2_interfaces__srv__msg__Pause_Response:
    """Class for rosbag2_interfaces/srv/msg/Pause_Response."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/msg/Pause_Response'


@dataclass
class rosbag2_interfaces__srv__msg__PlayNext_Request:
    """Class for rosbag2_interfaces/srv/msg/PlayNext_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/msg/PlayNext_Request'


@dataclass
class rosbag2_interfaces__srv__msg__PlayNext_Response:
    """Class for rosbag2_interfaces/srv/msg/PlayNext_Response."""

    success: bool
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/msg/PlayNext_Response'


@dataclass
class rosbag2_interfaces__srv__msg__Resume_Request:
    """Class for rosbag2_interfaces/srv/msg/Resume_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/msg/Resume_Request'


@dataclass
class rosbag2_interfaces__srv__msg__Resume_Response:
    """Class for rosbag2_interfaces/srv/msg/Resume_Response."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/msg/Resume_Response'


@dataclass
class rosbag2_interfaces__srv__msg__Seek_Request:
    """Class for rosbag2_interfaces/srv/msg/Seek_Request."""

    time: builtin_interfaces__msg__Time
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/msg/Seek_Request'


@dataclass
class rosbag2_interfaces__srv__msg__Seek_Response:
    """Class for rosbag2_interfaces/srv/msg/Seek_Response."""

    success: bool
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/msg/Seek_Response'


@dataclass
class rosbag2_interfaces__srv__msg__SetRate_Request:
    """Class for rosbag2_interfaces/srv/msg/SetRate_Request."""

    rate: float
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/msg/SetRate_Request'


@dataclass
class rosbag2_interfaces__srv__msg__SetRate_Response:
    """Class for rosbag2_interfaces/srv/msg/SetRate_Response."""

    success: bool
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/msg/SetRate_Response'


@dataclass
class rosbag2_interfaces__srv__msg__Snapshot_Request:
    """Class for rosbag2_interfaces/srv/msg/Snapshot_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/msg/Snapshot_Request'


@dataclass
class rosbag2_interfaces__srv__msg__Snapshot_Response:
    """Class for rosbag2_interfaces/srv/msg/Snapshot_Response."""

    success: bool
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/msg/Snapshot_Response'


@dataclass
class rosbag2_interfaces__srv__msg__TogglePaused_Request:
    """Class for rosbag2_interfaces/srv/msg/TogglePaused_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/msg/TogglePaused_Request'


@dataclass
class rosbag2_interfaces__srv__msg__TogglePaused_Response:
    """Class for rosbag2_interfaces/srv/msg/TogglePaused_Response."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'rosbag2_interfaces/srv/msg/TogglePaused_Response'


@dataclass
class sensor_msgs__srv__SetCameraInfo_Request:
    """Class for sensor_msgs/srv/SetCameraInfo_Request."""

    camera_info: sensor_msgs__msg__CameraInfo
    __msgtype__: ClassVar[str] = 'sensor_msgs/srv/SetCameraInfo_Request'


@dataclass
class sensor_msgs__srv__SetCameraInfo_Response:
    """Class for sensor_msgs/srv/SetCameraInfo_Response."""

    success: bool
    status_message: str
    __msgtype__: ClassVar[str] = 'sensor_msgs/srv/SetCameraInfo_Response'


@dataclass
class sensor_msgs__srv__msg__SetCameraInfo_Request:
    """Class for sensor_msgs/srv/msg/SetCameraInfo_Request."""

    camera_info: sensor_msgs__msg__CameraInfo
    __msgtype__: ClassVar[str] = 'sensor_msgs/srv/msg/SetCameraInfo_Request'


@dataclass
class sensor_msgs__srv__msg__SetCameraInfo_Response:
    """Class for sensor_msgs/srv/msg/SetCameraInfo_Response."""

    success: bool
    status_message: str
    __msgtype__: ClassVar[str] = 'sensor_msgs/srv/msg/SetCameraInfo_Response'


@dataclass
class std_msgs__msg__ByteMultiArray:
    """Class for std_msgs/msg/ByteMultiArray."""

    layout: std_msgs__msg__MultiArrayLayout
    data: list[octet]
    __msgtype__: ClassVar[str] = 'std_msgs/msg/ByteMultiArray'


@dataclass
class std_msgs__msg__Char:
    """Class for std_msgs/msg/Char."""

    data: int
    __msgtype__: ClassVar[str] = 'std_msgs/msg/Char'


@dataclass
class std_srvs__srv__Empty_Request:
    """Class for std_srvs/srv/Empty_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'std_srvs/srv/Empty_Request'


@dataclass
class std_srvs__srv__Empty_Response:
    """Class for std_srvs/srv/Empty_Response."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'std_srvs/srv/Empty_Response'


@dataclass
class std_srvs__srv__SetBool_Request:
    """Class for std_srvs/srv/SetBool_Request."""

    data: bool
    __msgtype__: ClassVar[str] = 'std_srvs/srv/SetBool_Request'


@dataclass
class std_srvs__srv__SetBool_Response:
    """Class for std_srvs/srv/SetBool_Response."""

    success: bool
    message: str
    __msgtype__: ClassVar[str] = 'std_srvs/srv/SetBool_Response'


@dataclass
class std_srvs__srv__Trigger_Request:
    """Class for std_srvs/srv/Trigger_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'std_srvs/srv/Trigger_Request'


@dataclass
class std_srvs__srv__Trigger_Response:
    """Class for std_srvs/srv/Trigger_Response."""

    success: bool
    message: str
    __msgtype__: ClassVar[str] = 'std_srvs/srv/Trigger_Response'


@dataclass
class std_srvs__srv__msg__Empty_Request:
    """Class for std_srvs/srv/msg/Empty_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'std_srvs/srv/msg/Empty_Request'


@dataclass
class std_srvs__srv__msg__Empty_Response:
    """Class for std_srvs/srv/msg/Empty_Response."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'std_srvs/srv/msg/Empty_Response'


@dataclass
class std_srvs__srv__msg__SetBool_Request:
    """Class for std_srvs/srv/msg/SetBool_Request."""

    data: bool
    __msgtype__: ClassVar[str] = 'std_srvs/srv/msg/SetBool_Request'


@dataclass
class std_srvs__srv__msg__SetBool_Response:
    """Class for std_srvs/srv/msg/SetBool_Response."""

    success: bool
    message: str
    __msgtype__: ClassVar[str] = 'std_srvs/srv/msg/SetBool_Response'


@dataclass
class std_srvs__srv__msg__Trigger_Request:
    """Class for std_srvs/srv/msg/Trigger_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'std_srvs/srv/msg/Trigger_Request'


@dataclass
class std_srvs__srv__msg__Trigger_Response:
    """Class for std_srvs/srv/msg/Trigger_Response."""

    success: bool
    message: str
    __msgtype__: ClassVar[str] = 'std_srvs/srv/msg/Trigger_Response'


@dataclass
class tf2_msgs__action__LookupTransform_Feedback:
    """Class for tf2_msgs/action/LookupTransform_Feedback."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'tf2_msgs/action/LookupTransform_Feedback'


@dataclass
class tf2_msgs__action__LookupTransform_Goal:
    """Class for tf2_msgs/action/LookupTransform_Goal."""

    target_frame: str
    source_frame: str
    source_time: builtin_interfaces__msg__Time
    timeout: builtin_interfaces__msg__Duration
    target_time: builtin_interfaces__msg__Time
    fixed_frame: str
    advanced: bool
    __msgtype__: ClassVar[str] = 'tf2_msgs/action/LookupTransform_Goal'


@dataclass
class tf2_msgs__action__LookupTransform_Result:
    """Class for tf2_msgs/action/LookupTransform_Result."""

    transform: geometry_msgs__msg__TransformStamped
    error: tf2_msgs__msg__TF2Error
    __msgtype__: ClassVar[str] = 'tf2_msgs/action/LookupTransform_Result'


@dataclass
class tf2_msgs__srv__FrameGraph_Request:
    """Class for tf2_msgs/srv/FrameGraph_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'tf2_msgs/srv/FrameGraph_Request'


@dataclass
class tf2_msgs__srv__FrameGraph_Response:
    """Class for tf2_msgs/srv/FrameGraph_Response."""

    frame_yaml: str
    __msgtype__: ClassVar[str] = 'tf2_msgs/srv/FrameGraph_Response'


@dataclass
class tf2_msgs__srv__msg__FrameGraph_Request:
    """Class for tf2_msgs/srv/msg/FrameGraph_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'tf2_msgs/srv/msg/FrameGraph_Request'


@dataclass
class tf2_msgs__srv__msg__FrameGraph_Response:
    """Class for tf2_msgs/srv/msg/FrameGraph_Response."""

    frame_yaml: str
    __msgtype__: ClassVar[str] = 'tf2_msgs/srv/msg/FrameGraph_Response'


@dataclass
class turtlesim__action__RotateAbsolute_Feedback:
    """Class for turtlesim/action/RotateAbsolute_Feedback."""

    remaining: float
    __msgtype__: ClassVar[str] = 'turtlesim/action/RotateAbsolute_Feedback'


@dataclass
class turtlesim__action__RotateAbsolute_Goal:
    """Class for turtlesim/action/RotateAbsolute_Goal."""

    theta: float
    __msgtype__: ClassVar[str] = 'turtlesim/action/RotateAbsolute_Goal'


@dataclass
class turtlesim__action__RotateAbsolute_Result:
    """Class for turtlesim/action/RotateAbsolute_Result."""

    delta: float
    __msgtype__: ClassVar[str] = 'turtlesim/action/RotateAbsolute_Result'


@dataclass
class turtlesim__msg__Color:
    """Class for turtlesim/msg/Color."""

    r: int
    g: int
    b: int
    __msgtype__: ClassVar[str] = 'turtlesim/msg/Color'


@dataclass
class turtlesim__msg__Pose:
    """Class for turtlesim/msg/Pose."""

    x: float
    y: float
    theta: float
    linear_velocity: float
    angular_velocity: float
    __msgtype__: ClassVar[str] = 'turtlesim/msg/Pose'


@dataclass
class turtlesim__srv__Kill_Request:
    """Class for turtlesim/srv/Kill_Request."""

    name: str
    __msgtype__: ClassVar[str] = 'turtlesim/srv/Kill_Request'


@dataclass
class turtlesim__srv__Kill_Response:
    """Class for turtlesim/srv/Kill_Response."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'turtlesim/srv/Kill_Response'


@dataclass
class turtlesim__srv__SetPen_Request:
    """Class for turtlesim/srv/SetPen_Request."""

    r: int
    g: int
    b: int
    width: int
    off: int
    __msgtype__: ClassVar[str] = 'turtlesim/srv/SetPen_Request'


@dataclass
class turtlesim__srv__SetPen_Response:
    """Class for turtlesim/srv/SetPen_Response."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'turtlesim/srv/SetPen_Response'


@dataclass
class turtlesim__srv__Spawn_Request:
    """Class for turtlesim/srv/Spawn_Request."""

    x: float
    y: float
    theta: float
    name: str
    __msgtype__: ClassVar[str] = 'turtlesim/srv/Spawn_Request'


@dataclass
class turtlesim__srv__Spawn_Response:
    """Class for turtlesim/srv/Spawn_Response."""

    name: str
    __msgtype__: ClassVar[str] = 'turtlesim/srv/Spawn_Response'


@dataclass
class turtlesim__srv__TeleportAbsolute_Request:
    """Class for turtlesim/srv/TeleportAbsolute_Request."""

    x: float
    y: float
    theta: float
    __msgtype__: ClassVar[str] = 'turtlesim/srv/TeleportAbsolute_Request'


@dataclass
class turtlesim__srv__TeleportAbsolute_Response:
    """Class for turtlesim/srv/TeleportAbsolute_Response."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'turtlesim/srv/TeleportAbsolute_Response'


@dataclass
class turtlesim__srv__TeleportRelative_Request:
    """Class for turtlesim/srv/TeleportRelative_Request."""

    linear: float
    angular: float
    __msgtype__: ClassVar[str] = 'turtlesim/srv/TeleportRelative_Request'


@dataclass
class turtlesim__srv__TeleportRelative_Response:
    """Class for turtlesim/srv/TeleportRelative_Response."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'turtlesim/srv/TeleportRelative_Response'


@dataclass
class turtlesim__srv__msg__Kill_Request:
    """Class for turtlesim/srv/msg/Kill_Request."""

    name: str
    __msgtype__: ClassVar[str] = 'turtlesim/srv/msg/Kill_Request'


@dataclass
class turtlesim__srv__msg__Kill_Response:
    """Class for turtlesim/srv/msg/Kill_Response."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'turtlesim/srv/msg/Kill_Response'


@dataclass
class turtlesim__srv__msg__SetPen_Request:
    """Class for turtlesim/srv/msg/SetPen_Request."""

    r: int
    g: int
    b: int
    width: int
    off: int
    __msgtype__: ClassVar[str] = 'turtlesim/srv/msg/SetPen_Request'


@dataclass
class turtlesim__srv__msg__SetPen_Response:
    """Class for turtlesim/srv/msg/SetPen_Response."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'turtlesim/srv/msg/SetPen_Response'


@dataclass
class turtlesim__srv__msg__Spawn_Request:
    """Class for turtlesim/srv/msg/Spawn_Request."""

    x: float
    y: float
    theta: float
    name: str
    __msgtype__: ClassVar[str] = 'turtlesim/srv/msg/Spawn_Request'


@dataclass
class turtlesim__srv__msg__Spawn_Response:
    """Class for turtlesim/srv/msg/Spawn_Response."""

    name: str
    __msgtype__: ClassVar[str] = 'turtlesim/srv/msg/Spawn_Response'


@dataclass
class turtlesim__srv__msg__TeleportAbsolute_Request:
    """Class for turtlesim/srv/msg/TeleportAbsolute_Request."""

    x: float
    y: float
    theta: float
    __msgtype__: ClassVar[str] = 'turtlesim/srv/msg/TeleportAbsolute_Request'


@dataclass
class turtlesim__srv__msg__TeleportAbsolute_Response:
    """Class for turtlesim/srv/msg/TeleportAbsolute_Response."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'turtlesim/srv/msg/TeleportAbsolute_Response'


@dataclass
class turtlesim__srv__msg__TeleportRelative_Request:
    """Class for turtlesim/srv/msg/TeleportRelative_Request."""

    linear: float
    angular: float
    __msgtype__: ClassVar[str] = 'turtlesim/srv/msg/TeleportRelative_Request'


@dataclass
class turtlesim__srv__msg__TeleportRelative_Response:
    """Class for turtlesim/srv/msg/TeleportRelative_Response."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'turtlesim/srv/msg/TeleportRelative_Response'


@dataclass
class visualization_msgs__srv__GetInteractiveMarkers_Request:
    """Class for visualization_msgs/srv/GetInteractiveMarkers_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'visualization_msgs/srv/GetInteractiveMarkers_Request'


@dataclass
class visualization_msgs__srv__GetInteractiveMarkers_Response:
    """Class for visualization_msgs/srv/GetInteractiveMarkers_Response."""

    sequence_number: int
    markers: list[visualization_msgs__msg__InteractiveMarker]
    __msgtype__: ClassVar[str] = 'visualization_msgs/srv/GetInteractiveMarkers_Response'


@dataclass
class visualization_msgs__srv__msg__GetInteractiveMarkers_Request:
    """Class for visualization_msgs/srv/msg/GetInteractiveMarkers_Request."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = 'visualization_msgs/srv/msg/GetInteractiveMarkers_Request'


@dataclass
class visualization_msgs__srv__msg__GetInteractiveMarkers_Response:
    """Class for visualization_msgs/srv/msg/GetInteractiveMarkers_Response."""

    sequence_number: int
    markers: list[visualization_msgs__srv__msg__InteractiveMarker]
    __msgtype__: ClassVar[str] = 'visualization_msgs/srv/msg/GetInteractiveMarkers_Response'


FIELDDEFS: Typesdict = {
    'action_msgs/msg/GoalInfo': base.FIELDDEFS['action_msgs/msg/GoalInfo'],
    'action_msgs/msg/GoalStatus': base.FIELDDEFS['action_msgs/msg/GoalStatus'],
    'action_msgs/msg/GoalStatusArray': base.FIELDDEFS['action_msgs/msg/GoalStatusArray'],
    'actionlib_msgs/msg/GoalID': base.FIELDDEFS['actionlib_msgs/msg/GoalID'],
    'actionlib_msgs/msg/GoalStatus': base.FIELDDEFS['actionlib_msgs/msg/GoalStatus'],
    'actionlib_msgs/msg/GoalStatusArray': base.FIELDDEFS['actionlib_msgs/msg/GoalStatusArray'],
    'builtin_interfaces/msg/Duration': base.FIELDDEFS['builtin_interfaces/msg/Duration'],
    'builtin_interfaces/msg/Time': base.FIELDDEFS['builtin_interfaces/msg/Time'],
    'diagnostic_msgs/msg/DiagnosticArray': base.FIELDDEFS['diagnostic_msgs/msg/DiagnosticArray'],
    'diagnostic_msgs/msg/DiagnosticStatus': base.FIELDDEFS['diagnostic_msgs/msg/DiagnosticStatus'],
    'diagnostic_msgs/msg/KeyValue': base.FIELDDEFS['diagnostic_msgs/msg/KeyValue'],
    'geometry_msgs/msg/Accel': base.FIELDDEFS['geometry_msgs/msg/Accel'],
    'geometry_msgs/msg/AccelStamped': base.FIELDDEFS['geometry_msgs/msg/AccelStamped'],
    'geometry_msgs/msg/AccelWithCovariance': base.FIELDDEFS['geometry_msgs/msg/AccelWithCovariance'],
    'geometry_msgs/msg/AccelWithCovarianceStamped': base.FIELDDEFS['geometry_msgs/msg/AccelWithCovarianceStamped'],
    'geometry_msgs/msg/Inertia': base.FIELDDEFS['geometry_msgs/msg/Inertia'],
    'geometry_msgs/msg/InertiaStamped': base.FIELDDEFS['geometry_msgs/msg/InertiaStamped'],
    'geometry_msgs/msg/Point': base.FIELDDEFS['geometry_msgs/msg/Point'],
    'geometry_msgs/msg/Point32': base.FIELDDEFS['geometry_msgs/msg/Point32'],
    'geometry_msgs/msg/PointStamped': base.FIELDDEFS['geometry_msgs/msg/PointStamped'],
    'geometry_msgs/msg/Polygon': base.FIELDDEFS['geometry_msgs/msg/Polygon'],
    'geometry_msgs/msg/PolygonStamped': base.FIELDDEFS['geometry_msgs/msg/PolygonStamped'],
    'geometry_msgs/msg/Pose': base.FIELDDEFS['geometry_msgs/msg/Pose'],
    'geometry_msgs/msg/Pose2D': base.FIELDDEFS['geometry_msgs/msg/Pose2D'],
    'geometry_msgs/msg/PoseArray': base.FIELDDEFS['geometry_msgs/msg/PoseArray'],
    'geometry_msgs/msg/PoseStamped': base.FIELDDEFS['geometry_msgs/msg/PoseStamped'],
    'geometry_msgs/msg/PoseWithCovariance': base.FIELDDEFS['geometry_msgs/msg/PoseWithCovariance'],
    'geometry_msgs/msg/PoseWithCovarianceStamped': base.FIELDDEFS['geometry_msgs/msg/PoseWithCovarianceStamped'],
    'geometry_msgs/msg/Quaternion': base.FIELDDEFS['geometry_msgs/msg/Quaternion'],
    'geometry_msgs/msg/QuaternionStamped': base.FIELDDEFS['geometry_msgs/msg/QuaternionStamped'],
    'geometry_msgs/msg/Transform': base.FIELDDEFS['geometry_msgs/msg/Transform'],
    'geometry_msgs/msg/TransformStamped': base.FIELDDEFS['geometry_msgs/msg/TransformStamped'],
    'geometry_msgs/msg/Twist': base.FIELDDEFS['geometry_msgs/msg/Twist'],
    'geometry_msgs/msg/TwistStamped': base.FIELDDEFS['geometry_msgs/msg/TwistStamped'],
    'geometry_msgs/msg/TwistWithCovariance': base.FIELDDEFS['geometry_msgs/msg/TwistWithCovariance'],
    'geometry_msgs/msg/TwistWithCovarianceStamped': base.FIELDDEFS['geometry_msgs/msg/TwistWithCovarianceStamped'],
    'geometry_msgs/msg/Vector3': base.FIELDDEFS['geometry_msgs/msg/Vector3'],
    'geometry_msgs/msg/Vector3Stamped': base.FIELDDEFS['geometry_msgs/msg/Vector3Stamped'],
    'geometry_msgs/msg/Wrench': base.FIELDDEFS['geometry_msgs/msg/Wrench'],
    'geometry_msgs/msg/WrenchStamped': base.FIELDDEFS['geometry_msgs/msg/WrenchStamped'],
    'lifecycle_msgs/msg/State': base.FIELDDEFS['lifecycle_msgs/msg/State'],
    'lifecycle_msgs/msg/Transition': base.FIELDDEFS['lifecycle_msgs/msg/Transition'],
    'lifecycle_msgs/msg/TransitionDescription': base.FIELDDEFS['lifecycle_msgs/msg/TransitionDescription'],
    'lifecycle_msgs/msg/TransitionEvent': base.FIELDDEFS['lifecycle_msgs/msg/TransitionEvent'],
    'nav_msgs/msg/GridCells': base.FIELDDEFS['nav_msgs/msg/GridCells'],
    'nav_msgs/msg/MapMetaData': base.FIELDDEFS['nav_msgs/msg/MapMetaData'],
    'nav_msgs/msg/OccupancyGrid': base.FIELDDEFS['nav_msgs/msg/OccupancyGrid'],
    'nav_msgs/msg/Odometry': base.FIELDDEFS['nav_msgs/msg/Odometry'],
    'nav_msgs/msg/Path': base.FIELDDEFS['nav_msgs/msg/Path'],
    'rcl_interfaces/msg/FloatingPointRange': base.FIELDDEFS['rcl_interfaces/msg/FloatingPointRange'],
    'rcl_interfaces/msg/IntegerRange': base.FIELDDEFS['rcl_interfaces/msg/IntegerRange'],
    'rcl_interfaces/msg/ListParametersResult': base.FIELDDEFS['rcl_interfaces/msg/ListParametersResult'],
    'rcl_interfaces/msg/Log': base.FIELDDEFS['rcl_interfaces/msg/Log'],
    'rcl_interfaces/msg/Parameter': base.FIELDDEFS['rcl_interfaces/msg/Parameter'],
    'rcl_interfaces/msg/ParameterDescriptor': base.FIELDDEFS['rcl_interfaces/msg/ParameterDescriptor'],
    'rcl_interfaces/msg/ParameterEvent': base.FIELDDEFS['rcl_interfaces/msg/ParameterEvent'],
    'rcl_interfaces/msg/ParameterEventDescriptors': base.FIELDDEFS['rcl_interfaces/msg/ParameterEventDescriptors'],
    'rcl_interfaces/msg/ParameterType': base.FIELDDEFS['rcl_interfaces/msg/ParameterType'],
    'rcl_interfaces/msg/SetParametersResult': base.FIELDDEFS['rcl_interfaces/msg/SetParametersResult'],
    'rmw_dds_common/msg/Gid': base.FIELDDEFS['rmw_dds_common/msg/Gid'],
    'rmw_dds_common/msg/NodeEntitiesInfo': base.FIELDDEFS['rmw_dds_common/msg/NodeEntitiesInfo'],
    'rmw_dds_common/msg/ParticipantEntitiesInfo': base.FIELDDEFS['rmw_dds_common/msg/ParticipantEntitiesInfo'],
    'rosbag2_interfaces/msg/ReadSplitEvent': base.FIELDDEFS['rosbag2_interfaces/msg/ReadSplitEvent'],
    'rosbag2_interfaces/msg/WriteSplitEvent': base.FIELDDEFS['rosbag2_interfaces/msg/WriteSplitEvent'],
    'rosgraph_msgs/msg/Clock': base.FIELDDEFS['rosgraph_msgs/msg/Clock'],
    'sensor_msgs/msg/BatteryState': base.FIELDDEFS['sensor_msgs/msg/BatteryState'],
    'sensor_msgs/msg/CameraInfo': base.FIELDDEFS['sensor_msgs/msg/CameraInfo'],
    'sensor_msgs/msg/ChannelFloat32': base.FIELDDEFS['sensor_msgs/msg/ChannelFloat32'],
    'sensor_msgs/msg/CompressedImage': base.FIELDDEFS['sensor_msgs/msg/CompressedImage'],
    'sensor_msgs/msg/FluidPressure': base.FIELDDEFS['sensor_msgs/msg/FluidPressure'],
    'sensor_msgs/msg/Illuminance': base.FIELDDEFS['sensor_msgs/msg/Illuminance'],
    'sensor_msgs/msg/Image': base.FIELDDEFS['sensor_msgs/msg/Image'],
    'sensor_msgs/msg/Imu': base.FIELDDEFS['sensor_msgs/msg/Imu'],
    'sensor_msgs/msg/JointState': base.FIELDDEFS['sensor_msgs/msg/JointState'],
    'sensor_msgs/msg/Joy': base.FIELDDEFS['sensor_msgs/msg/Joy'],
    'sensor_msgs/msg/JoyFeedback': base.FIELDDEFS['sensor_msgs/msg/JoyFeedback'],
    'sensor_msgs/msg/JoyFeedbackArray': base.FIELDDEFS['sensor_msgs/msg/JoyFeedbackArray'],
    'sensor_msgs/msg/LaserEcho': base.FIELDDEFS['sensor_msgs/msg/LaserEcho'],
    'sensor_msgs/msg/LaserScan': base.FIELDDEFS['sensor_msgs/msg/LaserScan'],
    'sensor_msgs/msg/MagneticField': base.FIELDDEFS['sensor_msgs/msg/MagneticField'],
    'sensor_msgs/msg/MultiDOFJointState': base.FIELDDEFS['sensor_msgs/msg/MultiDOFJointState'],
    'sensor_msgs/msg/MultiEchoLaserScan': base.FIELDDEFS['sensor_msgs/msg/MultiEchoLaserScan'],
    'sensor_msgs/msg/NavSatFix': base.FIELDDEFS['sensor_msgs/msg/NavSatFix'],
    'sensor_msgs/msg/NavSatStatus': base.FIELDDEFS['sensor_msgs/msg/NavSatStatus'],
    'sensor_msgs/msg/PointCloud': base.FIELDDEFS['sensor_msgs/msg/PointCloud'],
    'sensor_msgs/msg/PointCloud2': base.FIELDDEFS['sensor_msgs/msg/PointCloud2'],
    'sensor_msgs/msg/PointField': base.FIELDDEFS['sensor_msgs/msg/PointField'],
    'sensor_msgs/msg/Range': base.FIELDDEFS['sensor_msgs/msg/Range'],
    'sensor_msgs/msg/RegionOfInterest': base.FIELDDEFS['sensor_msgs/msg/RegionOfInterest'],
    'sensor_msgs/msg/RelativeHumidity': base.FIELDDEFS['sensor_msgs/msg/RelativeHumidity'],
    'sensor_msgs/msg/Temperature': base.FIELDDEFS['sensor_msgs/msg/Temperature'],
    'sensor_msgs/msg/TimeReference': base.FIELDDEFS['sensor_msgs/msg/TimeReference'],
    'shape_msgs/msg/Mesh': base.FIELDDEFS['shape_msgs/msg/Mesh'],
    'shape_msgs/msg/MeshTriangle': base.FIELDDEFS['shape_msgs/msg/MeshTriangle'],
    'shape_msgs/msg/Plane': base.FIELDDEFS['shape_msgs/msg/Plane'],
    'shape_msgs/msg/SolidPrimitive': base.FIELDDEFS['shape_msgs/msg/SolidPrimitive'],
    'statistics_msgs/msg/MetricsMessage': base.FIELDDEFS['statistics_msgs/msg/MetricsMessage'],
    'statistics_msgs/msg/StatisticDataPoint': base.FIELDDEFS['statistics_msgs/msg/StatisticDataPoint'],
    'statistics_msgs/msg/StatisticDataType': base.FIELDDEFS['statistics_msgs/msg/StatisticDataType'],
    'std_msgs/msg/Bool': base.FIELDDEFS['std_msgs/msg/Bool'],
    'std_msgs/msg/Byte': base.FIELDDEFS['std_msgs/msg/Byte'],
    'std_msgs/msg/ColorRGBA': base.FIELDDEFS['std_msgs/msg/ColorRGBA'],
    'std_msgs/msg/Empty': base.FIELDDEFS['std_msgs/msg/Empty'],
    'std_msgs/msg/Float32': base.FIELDDEFS['std_msgs/msg/Float32'],
    'std_msgs/msg/Float32MultiArray': base.FIELDDEFS['std_msgs/msg/Float32MultiArray'],
    'std_msgs/msg/Float64': base.FIELDDEFS['std_msgs/msg/Float64'],
    'std_msgs/msg/Float64MultiArray': base.FIELDDEFS['std_msgs/msg/Float64MultiArray'],
    'std_msgs/msg/Header': base.FIELDDEFS['std_msgs/msg/Header'],
    'std_msgs/msg/Int16': base.FIELDDEFS['std_msgs/msg/Int16'],
    'std_msgs/msg/Int16MultiArray': base.FIELDDEFS['std_msgs/msg/Int16MultiArray'],
    'std_msgs/msg/Int32': base.FIELDDEFS['std_msgs/msg/Int32'],
    'std_msgs/msg/Int32MultiArray': base.FIELDDEFS['std_msgs/msg/Int32MultiArray'],
    'std_msgs/msg/Int64': base.FIELDDEFS['std_msgs/msg/Int64'],
    'std_msgs/msg/Int64MultiArray': base.FIELDDEFS['std_msgs/msg/Int64MultiArray'],
    'std_msgs/msg/Int8': base.FIELDDEFS['std_msgs/msg/Int8'],
    'std_msgs/msg/Int8MultiArray': base.FIELDDEFS['std_msgs/msg/Int8MultiArray'],
    'std_msgs/msg/MultiArrayDimension': base.FIELDDEFS['std_msgs/msg/MultiArrayDimension'],
    'std_msgs/msg/MultiArrayLayout': base.FIELDDEFS['std_msgs/msg/MultiArrayLayout'],
    'std_msgs/msg/String': base.FIELDDEFS['std_msgs/msg/String'],
    'std_msgs/msg/UInt16': base.FIELDDEFS['std_msgs/msg/UInt16'],
    'std_msgs/msg/UInt16MultiArray': base.FIELDDEFS['std_msgs/msg/UInt16MultiArray'],
    'std_msgs/msg/UInt32': base.FIELDDEFS['std_msgs/msg/UInt32'],
    'std_msgs/msg/UInt32MultiArray': base.FIELDDEFS['std_msgs/msg/UInt32MultiArray'],
    'std_msgs/msg/UInt64': base.FIELDDEFS['std_msgs/msg/UInt64'],
    'std_msgs/msg/UInt64MultiArray': base.FIELDDEFS['std_msgs/msg/UInt64MultiArray'],
    'std_msgs/msg/UInt8': base.FIELDDEFS['std_msgs/msg/UInt8'],
    'std_msgs/msg/UInt8MultiArray': base.FIELDDEFS['std_msgs/msg/UInt8MultiArray'],
    'stereo_msgs/msg/DisparityImage': base.FIELDDEFS['stereo_msgs/msg/DisparityImage'],
    'tf2_msgs/msg/TF2Error': base.FIELDDEFS['tf2_msgs/msg/TF2Error'],
    'tf2_msgs/msg/TFMessage': base.FIELDDEFS['tf2_msgs/msg/TFMessage'],
    'trajectory_msgs/msg/JointTrajectory': base.FIELDDEFS['trajectory_msgs/msg/JointTrajectory'],
    'trajectory_msgs/msg/JointTrajectoryPoint': base.FIELDDEFS['trajectory_msgs/msg/JointTrajectoryPoint'],
    'trajectory_msgs/msg/MultiDOFJointTrajectory': base.FIELDDEFS['trajectory_msgs/msg/MultiDOFJointTrajectory'],
    'trajectory_msgs/msg/MultiDOFJointTrajectoryPoint': base.FIELDDEFS['trajectory_msgs/msg/MultiDOFJointTrajectoryPoint'],
    'unique_identifier_msgs/msg/UUID': base.FIELDDEFS['unique_identifier_msgs/msg/UUID'],
    'visualization_msgs/msg/ImageMarker': base.FIELDDEFS['visualization_msgs/msg/ImageMarker'],
    'visualization_msgs/msg/InteractiveMarker': base.FIELDDEFS['visualization_msgs/msg/InteractiveMarker'],
    'visualization_msgs/msg/InteractiveMarkerControl': base.FIELDDEFS['visualization_msgs/msg/InteractiveMarkerControl'],
    'visualization_msgs/msg/InteractiveMarkerFeedback': base.FIELDDEFS['visualization_msgs/msg/InteractiveMarkerFeedback'],
    'visualization_msgs/msg/InteractiveMarkerInit': base.FIELDDEFS['visualization_msgs/msg/InteractiveMarkerInit'],
    'visualization_msgs/msg/InteractiveMarkerPose': base.FIELDDEFS['visualization_msgs/msg/InteractiveMarkerPose'],
    'visualization_msgs/msg/InteractiveMarkerUpdate': base.FIELDDEFS['visualization_msgs/msg/InteractiveMarkerUpdate'],
    'visualization_msgs/msg/Marker': base.FIELDDEFS['visualization_msgs/msg/Marker'],
    'visualization_msgs/msg/MarkerArray': base.FIELDDEFS['visualization_msgs/msg/MarkerArray'],
    'visualization_msgs/msg/MenuEntry': base.FIELDDEFS['visualization_msgs/msg/MenuEntry'],
    'visualization_msgs/msg/MeshFile': base.FIELDDEFS['visualization_msgs/msg/MeshFile'],
    'visualization_msgs/msg/UVCoordinate': base.FIELDDEFS['visualization_msgs/msg/UVCoordinate'],
    'ackermann_msgs/msg/AckermannDrive': (
        [],
        [
            ('steering_angle', (T.BASE, ('float32', 0))),
            ('steering_angle_velocity', (T.BASE, ('float32', 0))),
            ('speed', (T.BASE, ('float32', 0))),
            ('acceleration', (T.BASE, ('float32', 0))),
            ('jerk', (T.BASE, ('float32', 0))),
        ],
    ),
    'ackermann_msgs/msg/AckermannDriveStamped': (
        [],
        [
            ('header', (T.NAME, 'std_msgs/msg/Header')),
            ('drive', (T.NAME, 'ackermann_msgs/msg/AckermannDrive')),
        ],
    ),
    'action_msgs/srv/msg/CancelGoal_Request': (
        [],
        [
            ('goal_info', (T.NAME, 'action_msgs/srv/msg/GoalInfo')),
        ],
    ),
    'action_msgs/srv/msg/CancelGoal_Response': (
        [
            ('ERROR_NONE', 'int8', 0),
            ('ERROR_REJECTED', 'int8', 1),
            ('ERROR_UNKNOWN_GOAL_ID', 'int8', 2),
            ('ERROR_GOAL_TERMINATED', 'int8', 3),
        ],
        [
            ('return_code', (T.BASE, ('int8', 0))),
            ('goals_canceling', (T.SEQUENCE, ((T.NAME, 'action_msgs/srv/msg/GoalInfo'), 0))),
        ],
    ),
    'action_tutorials_interfaces/action/Fibonacci_Feedback': (
        [],
        [
            ('partial_sequence', (T.SEQUENCE, ((T.BASE, ('int32', 0)), 0))),
        ],
    ),
    'action_tutorials_interfaces/action/Fibonacci_Goal': (
        [],
        [
            ('order', (T.BASE, ('int32', 0))),
        ],
    ),
    'action_tutorials_interfaces/action/Fibonacci_Result': (
        [],
        [
            ('sequence', (T.SEQUENCE, ((T.BASE, ('int32', 0)), 0))),
        ],
    ),
    'bond/msg/Constants': (
        [
            ('DEAD_PUBLISH_PERIOD', 'float32', 0.05),
            ('DEFAULT_CONNECT_TIMEOUT', 'float32', 10.0),
            ('DEFAULT_HEARTBEAT_TIMEOUT', 'float32', 4.0),
            ('DEFAULT_DISCONNECT_TIMEOUT', 'float32', 2.0),
            ('DEFAULT_HEARTBEAT_PERIOD', 'float32', 1.0),
            ('DISABLE_HEARTBEAT_TIMEOUT_PARAM', 'string', '/bond_disable_heartbeat_timeout'),
        ],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'bond/msg/Status': (
        [],
        [
            ('header', (T.NAME, 'std_msgs/msg/Header')),
            ('id', (T.BASE, ('string', 0))),
            ('instance_id', (T.BASE, ('string', 0))),
            ('active', (T.BASE, ('bool', 0))),
            ('heartbeat_timeout', (T.BASE, ('float32', 0))),
            ('heartbeat_period', (T.BASE, ('float32', 0))),
        ],
    ),
    'composition_interfaces/srv/ListNodes_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'composition_interfaces/srv/ListNodes_Response': (
        [],
        [
            ('full_node_names', (T.SEQUENCE, ((T.BASE, ('string', 0)), 0))),
            ('unique_ids', (T.SEQUENCE, ((T.BASE, ('uint64', 0)), 0))),
        ],
    ),
    'composition_interfaces/srv/LoadNode_Request': (
        [],
        [
            ('package_name', (T.BASE, ('string', 0))),
            ('plugin_name', (T.BASE, ('string', 0))),
            ('node_name', (T.BASE, ('string', 0))),
            ('node_namespace', (T.BASE, ('string', 0))),
            ('log_level', (T.BASE, ('uint8', 0))),
            ('remap_rules', (T.SEQUENCE, ((T.BASE, ('string', 0)), 0))),
            ('parameters', (T.SEQUENCE, ((T.NAME, 'rcl_interfaces/msg/Parameter'), 0))),
            ('extra_arguments', (T.SEQUENCE, ((T.NAME, 'rcl_interfaces/msg/Parameter'), 0))),
        ],
    ),
    'composition_interfaces/srv/LoadNode_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
            ('error_message', (T.BASE, ('string', 0))),
            ('full_node_name', (T.BASE, ('string', 0))),
            ('unique_id', (T.BASE, ('uint64', 0))),
        ],
    ),
    'composition_interfaces/srv/UnloadNode_Request': (
        [],
        [
            ('unique_id', (T.BASE, ('uint64', 0))),
        ],
    ),
    'composition_interfaces/srv/UnloadNode_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
            ('error_message', (T.BASE, ('string', 0))),
        ],
    ),
    'composition_interfaces/srv/msg/ListNodes_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'composition_interfaces/srv/msg/ListNodes_Response': (
        [],
        [
            ('full_node_names', (T.SEQUENCE, ((T.BASE, ('string', 0)), 0))),
            ('unique_ids', (T.SEQUENCE, ((T.BASE, ('uint64', 0)), 0))),
        ],
    ),
    'composition_interfaces/srv/msg/LoadNode_Request': (
        [],
        [
            ('package_name', (T.BASE, ('string', 0))),
            ('plugin_name', (T.BASE, ('string', 0))),
            ('node_name', (T.BASE, ('string', 0))),
            ('node_namespace', (T.BASE, ('string', 0))),
            ('log_level', (T.BASE, ('uint8', 0))),
            ('remap_rules', (T.SEQUENCE, ((T.BASE, ('string', 0)), 0))),
            ('parameters', (T.SEQUENCE, ((T.NAME, 'rcl_interfaces/msg/Parameter'), 0))),
            ('extra_arguments', (T.SEQUENCE, ((T.NAME, 'rcl_interfaces/msg/Parameter'), 0))),
        ],
    ),
    'composition_interfaces/srv/msg/LoadNode_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
            ('error_message', (T.BASE, ('string', 0))),
            ('full_node_name', (T.BASE, ('string', 0))),
            ('unique_id', (T.BASE, ('uint64', 0))),
        ],
    ),
    'composition_interfaces/srv/msg/UnloadNode_Request': (
        [],
        [
            ('unique_id', (T.BASE, ('uint64', 0))),
        ],
    ),
    'composition_interfaces/srv/msg/UnloadNode_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
            ('error_message', (T.BASE, ('string', 0))),
        ],
    ),
    'diagnostic_msgs/srv/AddDiagnostics_Request': (
        [],
        [
            ('load_namespace', (T.BASE, ('string', 0))),
        ],
    ),
    'diagnostic_msgs/srv/AddDiagnostics_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
            ('message', (T.BASE, ('string', 0))),
        ],
    ),
    'diagnostic_msgs/srv/SelfTest_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'diagnostic_msgs/srv/SelfTest_Response': (
        [],
        [
            ('id', (T.BASE, ('string', 0))),
            ('passed', (T.NAME, 'octet')),
            ('status', (T.SEQUENCE, ((T.NAME, 'diagnostic_msgs/msg/DiagnosticStatus'), 0))),
        ],
    ),
    'diagnostic_msgs/srv/msg/AddDiagnostics_Request': (
        [],
        [
            ('load_namespace', (T.BASE, ('string', 0))),
        ],
    ),
    'diagnostic_msgs/srv/msg/AddDiagnostics_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
            ('message', (T.BASE, ('string', 0))),
        ],
    ),
    'diagnostic_msgs/srv/msg/SelfTest_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'diagnostic_msgs/srv/msg/SelfTest_Response': (
        [],
        [
            ('id', (T.BASE, ('string', 0))),
            ('passed', (T.BASE, ('byte', 0))),
            ('status', (T.SEQUENCE, ((T.NAME, 'diagnostic_msgs/srv/msg/DiagnosticStatus'), 0))),
        ],
    ),
    'dwb_msgs/msg/CriticScore': (
        [],
        [
            ('name', (T.BASE, ('string', 0))),
            ('raw_score', (T.BASE, ('float32', 0))),
            ('scale', (T.BASE, ('float32', 0))),
        ],
    ),
    'dwb_msgs/msg/LocalPlanEvaluation': (
        [],
        [
            ('header', (T.NAME, 'std_msgs/msg/Header')),
            ('twists', (T.SEQUENCE, ((T.NAME, 'dwb_msgs/msg/TrajectoryScore'), 0))),
            ('best_index', (T.BASE, ('uint16', 0))),
            ('worst_index', (T.BASE, ('uint16', 0))),
        ],
    ),
    'dwb_msgs/msg/Trajectory2D': (
        [],
        [
            ('velocity', (T.NAME, 'nav_2d_msgs/msg/Twist2D')),
            ('time_offsets', (T.SEQUENCE, ((T.NAME, 'builtin_interfaces/msg/Duration'), 0))),
            ('poses', (T.SEQUENCE, ((T.NAME, 'geometry_msgs/msg/Pose2D'), 0))),
        ],
    ),
    'dwb_msgs/msg/TrajectoryScore': (
        [],
        [
            ('traj', (T.NAME, 'dwb_msgs/msg/Trajectory2D')),
            ('scores', (T.SEQUENCE, ((T.NAME, 'dwb_msgs/msg/CriticScore'), 0))),
            ('total', (T.BASE, ('float32', 0))),
        ],
    ),
    'dwb_msgs/srv/DebugLocalPlan_Request': (
        [],
        [
            ('pose', (T.NAME, 'nav_2d_msgs/msg/Pose2DStamped')),
            ('velocity', (T.NAME, 'nav_2d_msgs/msg/Twist2D')),
            ('global_plan', (T.NAME, 'nav_2d_msgs/msg/Path2D')),
        ],
    ),
    'dwb_msgs/srv/DebugLocalPlan_Response': (
        [],
        [
            ('results', (T.NAME, 'dwb_msgs/msg/LocalPlanEvaluation')),
        ],
    ),
    'dwb_msgs/srv/GenerateTrajectory_Request': (
        [],
        [
            ('start_pose', (T.NAME, 'geometry_msgs/msg/Pose2D')),
            ('start_vel', (T.NAME, 'nav_2d_msgs/msg/Twist2D')),
            ('cmd_vel', (T.NAME, 'nav_2d_msgs/msg/Twist2D')),
        ],
    ),
    'dwb_msgs/srv/GenerateTrajectory_Response': (
        [],
        [
            ('traj', (T.NAME, 'dwb_msgs/msg/Trajectory2D')),
        ],
    ),
    'dwb_msgs/srv/GenerateTwists_Request': (
        [],
        [
            ('current_vel', (T.NAME, 'nav_2d_msgs/msg/Twist2D')),
        ],
    ),
    'dwb_msgs/srv/GenerateTwists_Response': (
        [],
        [
            ('twists', (T.SEQUENCE, ((T.NAME, 'nav_2d_msgs/msg/Twist2D'), 0))),
        ],
    ),
    'dwb_msgs/srv/GetCriticScore_Request': (
        [],
        [
            ('pose', (T.NAME, 'nav_2d_msgs/msg/Pose2DStamped')),
            ('velocity', (T.NAME, 'nav_2d_msgs/msg/Twist2D')),
            ('global_plan', (T.NAME, 'nav_2d_msgs/msg/Path2D')),
            ('traj', (T.NAME, 'dwb_msgs/msg/Trajectory2D')),
            ('critic_name', (T.BASE, ('string', 0))),
        ],
    ),
    'dwb_msgs/srv/GetCriticScore_Response': (
        [],
        [
            ('score', (T.NAME, 'dwb_msgs/msg/CriticScore')),
        ],
    ),
    'dwb_msgs/srv/ScoreTrajectory_Request': (
        [],
        [
            ('pose', (T.NAME, 'nav_2d_msgs/msg/Pose2DStamped')),
            ('velocity', (T.NAME, 'nav_2d_msgs/msg/Twist2D')),
            ('global_plan', (T.NAME, 'nav_2d_msgs/msg/Path2D')),
            ('traj', (T.NAME, 'dwb_msgs/msg/Trajectory2D')),
        ],
    ),
    'dwb_msgs/srv/ScoreTrajectory_Response': (
        [],
        [
            ('score', (T.NAME, 'dwb_msgs/msg/TrajectoryScore')),
        ],
    ),
    'dwb_msgs/srv/msg/DebugLocalPlan_Request': (
        [],
        [
            ('pose', (T.NAME, 'nav_2d_msgs/msg/Pose2DStamped')),
            ('velocity', (T.NAME, 'nav_2d_msgs/msg/Twist2D')),
            ('global_plan', (T.NAME, 'nav_2d_msgs/msg/Path2D')),
        ],
    ),
    'dwb_msgs/srv/msg/DebugLocalPlan_Response': (
        [],
        [
            ('results', (T.NAME, 'dwb_msgs/srv/msg/LocalPlanEvaluation')),
        ],
    ),
    'dwb_msgs/srv/msg/GenerateTrajectory_Request': (
        [],
        [
            ('start_pose', (T.NAME, 'geometry_msgs/msg/Pose2D')),
            ('start_vel', (T.NAME, 'nav_2d_msgs/msg/Twist2D')),
            ('cmd_vel', (T.NAME, 'nav_2d_msgs/msg/Twist2D')),
        ],
    ),
    'dwb_msgs/srv/msg/GenerateTrajectory_Response': (
        [],
        [
            ('traj', (T.NAME, 'dwb_msgs/srv/msg/Trajectory2D')),
        ],
    ),
    'dwb_msgs/srv/msg/GenerateTwists_Request': (
        [],
        [
            ('current_vel', (T.NAME, 'nav_2d_msgs/msg/Twist2D')),
        ],
    ),
    'dwb_msgs/srv/msg/GenerateTwists_Response': (
        [],
        [
            ('twists', (T.SEQUENCE, ((T.NAME, 'nav_2d_msgs/msg/Twist2D'), 0))),
        ],
    ),
    'dwb_msgs/srv/msg/GetCriticScore_Request': (
        [],
        [
            ('pose', (T.NAME, 'nav_2d_msgs/msg/Pose2DStamped')),
            ('velocity', (T.NAME, 'nav_2d_msgs/msg/Twist2D')),
            ('global_plan', (T.NAME, 'nav_2d_msgs/msg/Path2D')),
            ('traj', (T.NAME, 'dwb_msgs/srv/msg/Trajectory2D')),
            ('critic_name', (T.BASE, ('string', 0))),
        ],
    ),
    'dwb_msgs/srv/msg/GetCriticScore_Response': (
        [],
        [
            ('score', (T.NAME, 'dwb_msgs/srv/msg/CriticScore')),
        ],
    ),
    'dwb_msgs/srv/msg/ScoreTrajectory_Request': (
        [],
        [
            ('pose', (T.NAME, 'nav_2d_msgs/msg/Pose2DStamped')),
            ('velocity', (T.NAME, 'nav_2d_msgs/msg/Twist2D')),
            ('global_plan', (T.NAME, 'nav_2d_msgs/msg/Path2D')),
            ('traj', (T.NAME, 'dwb_msgs/srv/msg/Trajectory2D')),
        ],
    ),
    'dwb_msgs/srv/msg/ScoreTrajectory_Response': (
        [],
        [
            ('score', (T.NAME, 'dwb_msgs/srv/msg/TrajectoryScore')),
        ],
    ),
    'example_interfaces/action/Fibonacci_Feedback': (
        [],
        [
            ('sequence', (T.SEQUENCE, ((T.BASE, ('int32', 0)), 0))),
        ],
    ),
    'example_interfaces/action/Fibonacci_Goal': (
        [],
        [
            ('order', (T.BASE, ('int32', 0))),
        ],
    ),
    'example_interfaces/action/Fibonacci_Result': (
        [],
        [
            ('sequence', (T.SEQUENCE, ((T.BASE, ('int32', 0)), 0))),
        ],
    ),
    'example_interfaces/msg/Bool': (
        [],
        [
            ('data', (T.BASE, ('bool', 0))),
        ],
    ),
    'example_interfaces/msg/Byte': (
        [],
        [
            ('data', (T.BASE, ('byte', 0))),
        ],
    ),
    'example_interfaces/msg/ByteMultiArray': (
        [],
        [
            ('layout', (T.NAME, 'example_interfaces/msg/MultiArrayLayout')),
            ('data', (T.SEQUENCE, ((T.NAME, 'octet'), 0))),
        ],
    ),
    'example_interfaces/msg/Char': (
        [],
        [
            ('data', (T.BASE, ('uint8', 0))),
        ],
    ),
    'example_interfaces/msg/Empty': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'example_interfaces/msg/Float32': (
        [],
        [
            ('data', (T.BASE, ('float32', 0))),
        ],
    ),
    'example_interfaces/msg/Float32MultiArray': (
        [],
        [
            ('layout', (T.NAME, 'example_interfaces/msg/MultiArrayLayout')),
            ('data', (T.SEQUENCE, ((T.BASE, ('float32', 0)), 0))),
        ],
    ),
    'example_interfaces/msg/Float64': (
        [],
        [
            ('data', (T.BASE, ('float64', 0))),
        ],
    ),
    'example_interfaces/msg/Float64MultiArray': (
        [],
        [
            ('layout', (T.NAME, 'example_interfaces/msg/MultiArrayLayout')),
            ('data', (T.SEQUENCE, ((T.BASE, ('float64', 0)), 0))),
        ],
    ),
    'example_interfaces/msg/Int16': (
        [],
        [
            ('data', (T.BASE, ('int16', 0))),
        ],
    ),
    'example_interfaces/msg/Int16MultiArray': (
        [],
        [
            ('layout', (T.NAME, 'example_interfaces/msg/MultiArrayLayout')),
            ('data', (T.SEQUENCE, ((T.BASE, ('int16', 0)), 0))),
        ],
    ),
    'example_interfaces/msg/Int32': (
        [],
        [
            ('data', (T.BASE, ('int32', 0))),
        ],
    ),
    'example_interfaces/msg/Int32MultiArray': (
        [],
        [
            ('layout', (T.NAME, 'example_interfaces/msg/MultiArrayLayout')),
            ('data', (T.SEQUENCE, ((T.BASE, ('int32', 0)), 0))),
        ],
    ),
    'example_interfaces/msg/Int64': (
        [],
        [
            ('data', (T.BASE, ('int64', 0))),
        ],
    ),
    'example_interfaces/msg/Int64MultiArray': (
        [],
        [
            ('layout', (T.NAME, 'example_interfaces/msg/MultiArrayLayout')),
            ('data', (T.SEQUENCE, ((T.BASE, ('int64', 0)), 0))),
        ],
    ),
    'example_interfaces/msg/Int8': (
        [],
        [
            ('data', (T.BASE, ('int8', 0))),
        ],
    ),
    'example_interfaces/msg/Int8MultiArray': (
        [],
        [
            ('layout', (T.NAME, 'example_interfaces/msg/MultiArrayLayout')),
            ('data', (T.SEQUENCE, ((T.BASE, ('int8', 0)), 0))),
        ],
    ),
    'example_interfaces/msg/MultiArrayDimension': (
        [],
        [
            ('label', (T.BASE, ('string', 0))),
            ('size', (T.BASE, ('uint32', 0))),
            ('stride', (T.BASE, ('uint32', 0))),
        ],
    ),
    'example_interfaces/msg/MultiArrayLayout': (
        [],
        [
            ('dim', (T.SEQUENCE, ((T.NAME, 'example_interfaces/msg/MultiArrayDimension'), 0))),
            ('data_offset', (T.BASE, ('uint32', 0))),
        ],
    ),
    'example_interfaces/msg/String': (
        [],
        [
            ('data', (T.BASE, ('string', 0))),
        ],
    ),
    'example_interfaces/msg/UInt16': (
        [],
        [
            ('data', (T.BASE, ('uint16', 0))),
        ],
    ),
    'example_interfaces/msg/UInt16MultiArray': (
        [],
        [
            ('layout', (T.NAME, 'example_interfaces/msg/MultiArrayLayout')),
            ('data', (T.SEQUENCE, ((T.BASE, ('uint16', 0)), 0))),
        ],
    ),
    'example_interfaces/msg/UInt32': (
        [],
        [
            ('data', (T.BASE, ('uint32', 0))),
        ],
    ),
    'example_interfaces/msg/UInt32MultiArray': (
        [],
        [
            ('layout', (T.NAME, 'example_interfaces/msg/MultiArrayLayout')),
            ('data', (T.SEQUENCE, ((T.BASE, ('uint32', 0)), 0))),
        ],
    ),
    'example_interfaces/msg/UInt64': (
        [],
        [
            ('data', (T.BASE, ('uint64', 0))),
        ],
    ),
    'example_interfaces/msg/UInt64MultiArray': (
        [],
        [
            ('layout', (T.NAME, 'example_interfaces/msg/MultiArrayLayout')),
            ('data', (T.SEQUENCE, ((T.BASE, ('uint64', 0)), 0))),
        ],
    ),
    'example_interfaces/msg/UInt8': (
        [],
        [
            ('data', (T.BASE, ('uint8', 0))),
        ],
    ),
    'example_interfaces/msg/UInt8MultiArray': (
        [],
        [
            ('layout', (T.NAME, 'example_interfaces/msg/MultiArrayLayout')),
            ('data', (T.SEQUENCE, ((T.BASE, ('uint8', 0)), 0))),
        ],
    ),
    'example_interfaces/msg/WString': (
        [],
        [
            ('data', (T.NAME, 'example_interfaces/msg/wstring')),
        ],
    ),
    'example_interfaces/srv/AddTwoInts_Request': (
        [],
        [
            ('a', (T.BASE, ('int64', 0))),
            ('b', (T.BASE, ('int64', 0))),
        ],
    ),
    'example_interfaces/srv/AddTwoInts_Response': (
        [],
        [
            ('sum', (T.BASE, ('int64', 0))),
        ],
    ),
    'example_interfaces/srv/SetBool_Request': (
        [],
        [
            ('data', (T.BASE, ('bool', 0))),
        ],
    ),
    'example_interfaces/srv/SetBool_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
            ('message', (T.BASE, ('string', 0))),
        ],
    ),
    'example_interfaces/srv/Trigger_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'example_interfaces/srv/Trigger_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
            ('message', (T.BASE, ('string', 0))),
        ],
    ),
    'example_interfaces/srv/msg/AddTwoInts_Request': (
        [],
        [
            ('a', (T.BASE, ('int64', 0))),
            ('b', (T.BASE, ('int64', 0))),
        ],
    ),
    'example_interfaces/srv/msg/AddTwoInts_Response': (
        [],
        [
            ('sum', (T.BASE, ('int64', 0))),
        ],
    ),
    'example_interfaces/srv/msg/SetBool_Request': (
        [],
        [
            ('data', (T.BASE, ('bool', 0))),
        ],
    ),
    'example_interfaces/srv/msg/SetBool_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
            ('message', (T.BASE, ('string', 0))),
        ],
    ),
    'example_interfaces/srv/msg/Trigger_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'example_interfaces/srv/msg/Trigger_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
            ('message', (T.BASE, ('string', 0))),
        ],
    ),
    'geometry_msgs/msg/VelocityStamped': (
        [],
        [
            ('header', (T.NAME, 'std_msgs/msg/Header')),
            ('body_frame_id', (T.BASE, ('string', 0))),
            ('reference_frame_id', (T.BASE, ('string', 0))),
            ('velocity', (T.NAME, 'geometry_msgs/msg/Twist')),
        ],
    ),
    'lifecycle_msgs/srv/ChangeState_Request': (
        [],
        [
            ('transition', (T.NAME, 'lifecycle_msgs/msg/Transition')),
        ],
    ),
    'lifecycle_msgs/srv/ChangeState_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
        ],
    ),
    'lifecycle_msgs/srv/GetAvailableStates_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'lifecycle_msgs/srv/GetAvailableStates_Response': (
        [],
        [
            ('available_states', (T.SEQUENCE, ((T.NAME, 'lifecycle_msgs/msg/State'), 0))),
        ],
    ),
    'lifecycle_msgs/srv/GetAvailableTransitions_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'lifecycle_msgs/srv/GetAvailableTransitions_Response': (
        [],
        [
            ('available_transitions', (T.SEQUENCE, ((T.NAME, 'lifecycle_msgs/msg/TransitionDescription'), 0))),
        ],
    ),
    'lifecycle_msgs/srv/GetState_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'lifecycle_msgs/srv/GetState_Response': (
        [],
        [
            ('current_state', (T.NAME, 'lifecycle_msgs/msg/State')),
        ],
    ),
    'lifecycle_msgs/srv/msg/ChangeState_Request': (
        [],
        [
            ('transition', (T.NAME, 'lifecycle_msgs/srv/msg/Transition')),
        ],
    ),
    'lifecycle_msgs/srv/msg/ChangeState_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
        ],
    ),
    'lifecycle_msgs/srv/msg/GetAvailableStates_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'lifecycle_msgs/srv/msg/GetAvailableStates_Response': (
        [],
        [
            ('available_states', (T.SEQUENCE, ((T.NAME, 'lifecycle_msgs/srv/msg/State'), 0))),
        ],
    ),
    'lifecycle_msgs/srv/msg/GetAvailableTransitions_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'lifecycle_msgs/srv/msg/GetAvailableTransitions_Response': (
        [],
        [
            ('available_transitions', (T.SEQUENCE, ((T.NAME, 'lifecycle_msgs/srv/msg/TransitionDescription'), 0))),
        ],
    ),
    'lifecycle_msgs/srv/msg/GetState_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'lifecycle_msgs/srv/msg/GetState_Response': (
        [],
        [
            ('current_state', (T.NAME, 'lifecycle_msgs/srv/msg/State')),
        ],
    ),
    'logging_demo/srv/ConfigLogger_Request': (
        [],
        [
            ('logger_name', (T.BASE, ('string', 0))),
            ('level', (T.BASE, ('string', 0))),
        ],
    ),
    'logging_demo/srv/ConfigLogger_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
        ],
    ),
    'logging_demo/srv/msg/ConfigLogger_Request': (
        [],
        [
            ('logger_name', (T.BASE, ('string', 0))),
            ('level', (T.BASE, ('string', 0))),
        ],
    ),
    'logging_demo/srv/msg/ConfigLogger_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
        ],
    ),
    'map_msgs/msg/OccupancyGridUpdate': (
        [],
        [
            ('header', (T.NAME, 'std_msgs/msg/Header')),
            ('x', (T.BASE, ('int32', 0))),
            ('y', (T.BASE, ('int32', 0))),
            ('width', (T.BASE, ('uint32', 0))),
            ('height', (T.BASE, ('uint32', 0))),
            ('data', (T.SEQUENCE, ((T.BASE, ('int8', 0)), 0))),
        ],
    ),
    'map_msgs/msg/PointCloud2Update': (
        [
            ('ADD', 'uint32', 0),
            ('DELETE', 'uint32', 1),
        ],
        [
            ('header', (T.NAME, 'std_msgs/msg/Header')),
            ('type', (T.BASE, ('uint32', 0))),
            ('points', (T.NAME, 'sensor_msgs/msg/PointCloud2')),
        ],
    ),
    'map_msgs/msg/ProjectedMap': (
        [],
        [
            ('map', (T.NAME, 'nav_msgs/msg/OccupancyGrid')),
            ('min_z', (T.BASE, ('float64', 0))),
            ('max_z', (T.BASE, ('float64', 0))),
        ],
    ),
    'map_msgs/msg/ProjectedMapInfo': (
        [],
        [
            ('frame_id', (T.BASE, ('string', 0))),
            ('x', (T.BASE, ('float64', 0))),
            ('y', (T.BASE, ('float64', 0))),
            ('width', (T.BASE, ('float64', 0))),
            ('height', (T.BASE, ('float64', 0))),
            ('min_z', (T.BASE, ('float64', 0))),
            ('max_z', (T.BASE, ('float64', 0))),
        ],
    ),
    'map_msgs/srv/GetMapROI_Request': (
        [],
        [
            ('x', (T.BASE, ('float64', 0))),
            ('y', (T.BASE, ('float64', 0))),
            ('l_x', (T.BASE, ('float64', 0))),
            ('l_y', (T.BASE, ('float64', 0))),
        ],
    ),
    'map_msgs/srv/GetMapROI_Response': (
        [],
        [
            ('sub_map', (T.NAME, 'nav_msgs/msg/OccupancyGrid')),
        ],
    ),
    'map_msgs/srv/GetPointMapROI_Request': (
        [],
        [
            ('x', (T.BASE, ('float64', 0))),
            ('y', (T.BASE, ('float64', 0))),
            ('z', (T.BASE, ('float64', 0))),
            ('r', (T.BASE, ('float64', 0))),
            ('l_x', (T.BASE, ('float64', 0))),
            ('l_y', (T.BASE, ('float64', 0))),
            ('l_z', (T.BASE, ('float64', 0))),
        ],
    ),
    'map_msgs/srv/GetPointMapROI_Response': (
        [],
        [
            ('sub_map', (T.NAME, 'sensor_msgs/msg/PointCloud2')),
        ],
    ),
    'map_msgs/srv/GetPointMap_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'map_msgs/srv/GetPointMap_Response': (
        [],
        [
            ('map', (T.NAME, 'sensor_msgs/msg/PointCloud2')),
        ],
    ),
    'map_msgs/srv/ProjectedMapsInfo_Request': (
        [],
        [
            ('projected_maps_info', (T.SEQUENCE, ((T.NAME, 'map_msgs/msg/ProjectedMapInfo'), 0))),
        ],
    ),
    'map_msgs/srv/ProjectedMapsInfo_Response': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'map_msgs/srv/SaveMap_Request': (
        [],
        [
            ('filename', (T.NAME, 'std_msgs/msg/String')),
        ],
    ),
    'map_msgs/srv/SaveMap_Response': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'map_msgs/srv/SetMapProjections_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'map_msgs/srv/SetMapProjections_Response': (
        [],
        [
            ('projected_maps_info', (T.SEQUENCE, ((T.NAME, 'map_msgs/msg/ProjectedMapInfo'), 0))),
        ],
    ),
    'map_msgs/srv/msg/GetMapROI_Request': (
        [],
        [
            ('x', (T.BASE, ('float64', 0))),
            ('y', (T.BASE, ('float64', 0))),
            ('l_x', (T.BASE, ('float64', 0))),
            ('l_y', (T.BASE, ('float64', 0))),
        ],
    ),
    'map_msgs/srv/msg/GetMapROI_Response': (
        [],
        [
            ('sub_map', (T.NAME, 'nav_msgs/msg/OccupancyGrid')),
        ],
    ),
    'map_msgs/srv/msg/GetPointMapROI_Request': (
        [],
        [
            ('x', (T.BASE, ('float64', 0))),
            ('y', (T.BASE, ('float64', 0))),
            ('z', (T.BASE, ('float64', 0))),
            ('r', (T.BASE, ('float64', 0))),
            ('l_x', (T.BASE, ('float64', 0))),
            ('l_y', (T.BASE, ('float64', 0))),
            ('l_z', (T.BASE, ('float64', 0))),
        ],
    ),
    'map_msgs/srv/msg/GetPointMapROI_Response': (
        [],
        [
            ('sub_map', (T.NAME, 'sensor_msgs/msg/PointCloud2')),
        ],
    ),
    'map_msgs/srv/msg/GetPointMap_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'map_msgs/srv/msg/GetPointMap_Response': (
        [],
        [
            ('map', (T.NAME, 'sensor_msgs/msg/PointCloud2')),
        ],
    ),
    'map_msgs/srv/msg/ProjectedMapsInfo_Request': (
        [],
        [
            ('projected_maps_info', (T.SEQUENCE, ((T.NAME, 'map_msgs/msg/ProjectedMapInfo'), 0))),
        ],
    ),
    'map_msgs/srv/msg/ProjectedMapsInfo_Response': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'map_msgs/srv/msg/SaveMap_Request': (
        [],
        [
            ('filename', (T.NAME, 'std_msgs/msg/String')),
        ],
    ),
    'map_msgs/srv/msg/SaveMap_Response': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'map_msgs/srv/msg/SetMapProjections_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'map_msgs/srv/msg/SetMapProjections_Response': (
        [],
        [
            ('projected_maps_info', (T.SEQUENCE, ((T.NAME, 'map_msgs/msg/ProjectedMapInfo'), 0))),
        ],
    ),
    'nav2_msgs/action/AssistedTeleop_Feedback': (
        [],
        [
            ('current_teleop_duration', (T.NAME, 'builtin_interfaces/msg/Duration')),
        ],
    ),
    'nav2_msgs/action/AssistedTeleop_Goal': (
        [],
        [
            ('time_allowance', (T.NAME, 'builtin_interfaces/msg/Duration')),
        ],
    ),
    'nav2_msgs/action/AssistedTeleop_Result': (
        [],
        [
            ('total_elapsed_time', (T.NAME, 'builtin_interfaces/msg/Duration')),
        ],
    ),
    'nav2_msgs/action/BackUp_Feedback': (
        [],
        [
            ('distance_traveled', (T.BASE, ('float32', 0))),
        ],
    ),
    'nav2_msgs/action/BackUp_Goal': (
        [],
        [
            ('target', (T.NAME, 'geometry_msgs/msg/Point')),
            ('speed', (T.BASE, ('float32', 0))),
            ('time_allowance', (T.NAME, 'builtin_interfaces/msg/Duration')),
        ],
    ),
    'nav2_msgs/action/BackUp_Result': (
        [],
        [
            ('total_elapsed_time', (T.NAME, 'builtin_interfaces/msg/Duration')),
        ],
    ),
    'nav2_msgs/action/ComputePathThroughPoses_Feedback': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'nav2_msgs/action/ComputePathThroughPoses_Goal': (
        [],
        [
            ('goals', (T.SEQUENCE, ((T.NAME, 'geometry_msgs/msg/PoseStamped'), 0))),
            ('start', (T.NAME, 'geometry_msgs/msg/PoseStamped')),
            ('planner_id', (T.BASE, ('string', 0))),
            ('use_start', (T.BASE, ('bool', 0))),
        ],
    ),
    'nav2_msgs/action/ComputePathThroughPoses_Result': (
        [],
        [
            ('path', (T.NAME, 'nav_msgs/msg/Path')),
            ('planning_time', (T.NAME, 'builtin_interfaces/msg/Duration')),
        ],
    ),
    'nav2_msgs/action/ComputePathToPose_Feedback': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'nav2_msgs/action/ComputePathToPose_Goal': (
        [],
        [
            ('goal', (T.NAME, 'geometry_msgs/msg/PoseStamped')),
            ('start', (T.NAME, 'geometry_msgs/msg/PoseStamped')),
            ('planner_id', (T.BASE, ('string', 0))),
            ('use_start', (T.BASE, ('bool', 0))),
        ],
    ),
    'nav2_msgs/action/ComputePathToPose_Result': (
        [],
        [
            ('path', (T.NAME, 'nav_msgs/msg/Path')),
            ('planning_time', (T.NAME, 'builtin_interfaces/msg/Duration')),
        ],
    ),
    'nav2_msgs/action/DriveOnHeading_Feedback': (
        [],
        [
            ('distance_traveled', (T.BASE, ('float32', 0))),
        ],
    ),
    'nav2_msgs/action/DriveOnHeading_Goal': (
        [],
        [
            ('target', (T.NAME, 'geometry_msgs/msg/Point')),
            ('speed', (T.BASE, ('float32', 0))),
            ('time_allowance', (T.NAME, 'builtin_interfaces/msg/Duration')),
        ],
    ),
    'nav2_msgs/action/DriveOnHeading_Result': (
        [],
        [
            ('total_elapsed_time', (T.NAME, 'builtin_interfaces/msg/Duration')),
        ],
    ),
    'nav2_msgs/action/DummyBehavior_Feedback': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'nav2_msgs/action/DummyBehavior_Goal': (
        [],
        [
            ('command', (T.NAME, 'std_msgs/msg/String')),
        ],
    ),
    'nav2_msgs/action/DummyBehavior_Result': (
        [],
        [
            ('total_elapsed_time', (T.NAME, 'builtin_interfaces/msg/Duration')),
        ],
    ),
    'nav2_msgs/action/FollowPath_Feedback': (
        [],
        [
            ('distance_to_goal', (T.BASE, ('float32', 0))),
            ('speed', (T.BASE, ('float32', 0))),
        ],
    ),
    'nav2_msgs/action/FollowPath_Goal': (
        [],
        [
            ('path', (T.NAME, 'nav_msgs/msg/Path')),
            ('controller_id', (T.BASE, ('string', 0))),
            ('goal_checker_id', (T.BASE, ('string', 0))),
        ],
    ),
    'nav2_msgs/action/FollowPath_Result': (
        [],
        [
            ('result', (T.NAME, 'std_msgs/msg/Empty')),
        ],
    ),
    'nav2_msgs/action/FollowWaypoints_Feedback': (
        [],
        [
            ('current_waypoint', (T.BASE, ('uint32', 0))),
        ],
    ),
    'nav2_msgs/action/FollowWaypoints_Goal': (
        [],
        [
            ('poses', (T.SEQUENCE, ((T.NAME, 'geometry_msgs/msg/PoseStamped'), 0))),
        ],
    ),
    'nav2_msgs/action/FollowWaypoints_Result': (
        [],
        [
            ('missed_waypoints', (T.SEQUENCE, ((T.BASE, ('int32', 0)), 0))),
        ],
    ),
    'nav2_msgs/action/NavigateThroughPoses_Feedback': (
        [],
        [
            ('current_pose', (T.NAME, 'geometry_msgs/msg/PoseStamped')),
            ('navigation_time', (T.NAME, 'builtin_interfaces/msg/Duration')),
            ('estimated_time_remaining', (T.NAME, 'builtin_interfaces/msg/Duration')),
            ('number_of_recoveries', (T.BASE, ('int16', 0))),
            ('distance_remaining', (T.BASE, ('float32', 0))),
            ('number_of_poses_remaining', (T.BASE, ('int16', 0))),
        ],
    ),
    'nav2_msgs/action/NavigateThroughPoses_Goal': (
        [],
        [
            ('poses', (T.SEQUENCE, ((T.NAME, 'geometry_msgs/msg/PoseStamped'), 0))),
            ('behavior_tree', (T.BASE, ('string', 0))),
        ],
    ),
    'nav2_msgs/action/NavigateThroughPoses_Result': (
        [],
        [
            ('result', (T.NAME, 'std_msgs/msg/Empty')),
        ],
    ),
    'nav2_msgs/action/NavigateToPose_Feedback': (
        [],
        [
            ('current_pose', (T.NAME, 'geometry_msgs/msg/PoseStamped')),
            ('navigation_time', (T.NAME, 'builtin_interfaces/msg/Duration')),
            ('estimated_time_remaining', (T.NAME, 'builtin_interfaces/msg/Duration')),
            ('number_of_recoveries', (T.BASE, ('int16', 0))),
            ('distance_remaining', (T.BASE, ('float32', 0))),
        ],
    ),
    'nav2_msgs/action/NavigateToPose_Goal': (
        [],
        [
            ('pose', (T.NAME, 'geometry_msgs/msg/PoseStamped')),
            ('behavior_tree', (T.BASE, ('string', 0))),
        ],
    ),
    'nav2_msgs/action/NavigateToPose_Result': (
        [],
        [
            ('result', (T.NAME, 'std_msgs/msg/Empty')),
        ],
    ),
    'nav2_msgs/action/SmoothPath_Feedback': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'nav2_msgs/action/SmoothPath_Goal': (
        [],
        [
            ('path', (T.NAME, 'nav_msgs/msg/Path')),
            ('smoother_id', (T.BASE, ('string', 0))),
            ('max_smoothing_duration', (T.NAME, 'builtin_interfaces/msg/Duration')),
            ('check_for_collisions', (T.BASE, ('bool', 0))),
        ],
    ),
    'nav2_msgs/action/SmoothPath_Result': (
        [],
        [
            ('path', (T.NAME, 'nav_msgs/msg/Path')),
            ('smoothing_duration', (T.NAME, 'builtin_interfaces/msg/Duration')),
            ('was_completed', (T.BASE, ('bool', 0))),
        ],
    ),
    'nav2_msgs/action/Spin_Feedback': (
        [],
        [
            ('angular_distance_traveled', (T.BASE, ('float32', 0))),
        ],
    ),
    'nav2_msgs/action/Spin_Goal': (
        [],
        [
            ('target_yaw', (T.BASE, ('float32', 0))),
            ('time_allowance', (T.NAME, 'builtin_interfaces/msg/Duration')),
        ],
    ),
    'nav2_msgs/action/Spin_Result': (
        [],
        [
            ('total_elapsed_time', (T.NAME, 'builtin_interfaces/msg/Duration')),
        ],
    ),
    'nav2_msgs/action/Wait_Feedback': (
        [],
        [
            ('time_left', (T.NAME, 'builtin_interfaces/msg/Duration')),
        ],
    ),
    'nav2_msgs/action/Wait_Goal': (
        [],
        [
            ('time', (T.NAME, 'builtin_interfaces/msg/Duration')),
        ],
    ),
    'nav2_msgs/action/Wait_Result': (
        [],
        [
            ('total_elapsed_time', (T.NAME, 'builtin_interfaces/msg/Duration')),
        ],
    ),
    'nav2_msgs/msg/BehaviorTreeLog': (
        [],
        [
            ('timestamp', (T.NAME, 'builtin_interfaces/msg/Time')),
            ('event_log', (T.SEQUENCE, ((T.NAME, 'nav2_msgs/msg/BehaviorTreeStatusChange'), 0))),
        ],
    ),
    'nav2_msgs/msg/BehaviorTreeStatusChange': (
        [],
        [
            ('timestamp', (T.NAME, 'builtin_interfaces/msg/Time')),
            ('node_name', (T.BASE, ('string', 0))),
            ('previous_status', (T.BASE, ('string', 0))),
            ('current_status', (T.BASE, ('string', 0))),
        ],
    ),
    'nav2_msgs/msg/CollisionMonitorState': (
        [
            ('DO_NOTHING', 'uint8', 0),
            ('STOP', 'uint8', 1),
            ('SLOWDOWN', 'uint8', 2),
            ('APPROACH', 'uint8', 3),
        ],
        [
            ('action_type', (T.BASE, ('uint8', 0))),
            ('polygon_name', (T.BASE, ('string', 0))),
        ],
    ),
    'nav2_msgs/msg/Costmap': (
        [],
        [
            ('header', (T.NAME, 'std_msgs/msg/Header')),
            ('metadata', (T.NAME, 'nav2_msgs/msg/CostmapMetaData')),
            ('data', (T.SEQUENCE, ((T.BASE, ('uint8', 0)), 0))),
        ],
    ),
    'nav2_msgs/msg/CostmapFilterInfo': (
        [],
        [
            ('header', (T.NAME, 'std_msgs/msg/Header')),
            ('type', (T.BASE, ('uint8', 0))),
            ('filter_mask_topic', (T.BASE, ('string', 0))),
            ('base', (T.BASE, ('float32', 0))),
            ('multiplier', (T.BASE, ('float32', 0))),
        ],
    ),
    'nav2_msgs/msg/CostmapMetaData': (
        [],
        [
            ('map_load_time', (T.NAME, 'builtin_interfaces/msg/Time')),
            ('update_time', (T.NAME, 'builtin_interfaces/msg/Time')),
            ('layer', (T.BASE, ('string', 0))),
            ('resolution', (T.BASE, ('float32', 0))),
            ('size_x', (T.BASE, ('uint32', 0))),
            ('size_y', (T.BASE, ('uint32', 0))),
            ('origin', (T.NAME, 'geometry_msgs/msg/Pose')),
        ],
    ),
    'nav2_msgs/msg/Particle': (
        [],
        [
            ('pose', (T.NAME, 'geometry_msgs/msg/Pose')),
            ('weight', (T.BASE, ('float64', 0))),
        ],
    ),
    'nav2_msgs/msg/ParticleCloud': (
        [],
        [
            ('header', (T.NAME, 'std_msgs/msg/Header')),
            ('particles', (T.SEQUENCE, ((T.NAME, 'nav2_msgs/msg/Particle'), 0))),
        ],
    ),
    'nav2_msgs/msg/SpeedLimit': (
        [],
        [
            ('header', (T.NAME, 'std_msgs/msg/Header')),
            ('percentage', (T.BASE, ('bool', 0))),
            ('speed_limit', (T.BASE, ('float64', 0))),
        ],
    ),
    'nav2_msgs/msg/VoxelGrid': (
        [],
        [
            ('header', (T.NAME, 'std_msgs/msg/Header')),
            ('data', (T.SEQUENCE, ((T.BASE, ('uint32', 0)), 0))),
            ('origin', (T.NAME, 'geometry_msgs/msg/Point32')),
            ('resolutions', (T.NAME, 'geometry_msgs/msg/Vector3')),
            ('size_x', (T.BASE, ('uint32', 0))),
            ('size_y', (T.BASE, ('uint32', 0))),
            ('size_z', (T.BASE, ('uint32', 0))),
        ],
    ),
    'nav2_msgs/srv/ClearCostmapAroundRobot_Request': (
        [],
        [
            ('reset_distance', (T.BASE, ('float32', 0))),
        ],
    ),
    'nav2_msgs/srv/ClearCostmapAroundRobot_Response': (
        [],
        [
            ('response', (T.NAME, 'std_msgs/msg/Empty')),
        ],
    ),
    'nav2_msgs/srv/ClearCostmapExceptRegion_Request': (
        [],
        [
            ('reset_distance', (T.BASE, ('float32', 0))),
        ],
    ),
    'nav2_msgs/srv/ClearCostmapExceptRegion_Response': (
        [],
        [
            ('response', (T.NAME, 'std_msgs/msg/Empty')),
        ],
    ),
    'nav2_msgs/srv/ClearEntireCostmap_Request': (
        [],
        [
            ('request', (T.NAME, 'std_msgs/msg/Empty')),
        ],
    ),
    'nav2_msgs/srv/ClearEntireCostmap_Response': (
        [],
        [
            ('response', (T.NAME, 'std_msgs/msg/Empty')),
        ],
    ),
    'nav2_msgs/srv/GetCostmap_Request': (
        [],
        [
            ('specs', (T.NAME, 'nav2_msgs/msg/CostmapMetaData')),
        ],
    ),
    'nav2_msgs/srv/GetCostmap_Response': (
        [],
        [
            ('map', (T.NAME, 'nav2_msgs/msg/Costmap')),
        ],
    ),
    'nav2_msgs/srv/IsPathValid_Request': (
        [],
        [
            ('path', (T.NAME, 'nav_msgs/msg/Path')),
        ],
    ),
    'nav2_msgs/srv/IsPathValid_Response': (
        [],
        [
            ('is_valid', (T.BASE, ('bool', 0))),
            ('invalid_pose_indices', (T.SEQUENCE, ((T.BASE, ('int32', 0)), 0))),
        ],
    ),
    'nav2_msgs/srv/LoadMap_Request': (
        [],
        [
            ('map_url', (T.BASE, ('string', 0))),
        ],
    ),
    'nav2_msgs/srv/LoadMap_Response': (
        [
            ('RESULT_SUCCESS', 'uint8', 0),
            ('RESULT_MAP_DOES_NOT_EXIST', 'uint8', 1),
            ('RESULT_INVALID_MAP_DATA', 'uint8', 2),
            ('RESULT_INVALID_MAP_METADATA', 'uint8', 3),
            ('RESULT_UNDEFINED_FAILURE', 'uint8', 255),
        ],
        [
            ('map', (T.NAME, 'nav_msgs/msg/OccupancyGrid')),
            ('result', (T.BASE, ('uint8', 0))),
        ],
    ),
    'nav2_msgs/srv/ManageLifecycleNodes_Request': (
        [
            ('STARTUP', 'uint8', 0),
            ('PAUSE', 'uint8', 1),
            ('RESUME', 'uint8', 2),
            ('RESET', 'uint8', 3),
            ('SHUTDOWN', 'uint8', 4),
        ],
        [
            ('command', (T.BASE, ('uint8', 0))),
        ],
    ),
    'nav2_msgs/srv/ManageLifecycleNodes_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
        ],
    ),
    'nav2_msgs/srv/SaveMap_Request': (
        [],
        [
            ('map_topic', (T.BASE, ('string', 0))),
            ('map_url', (T.BASE, ('string', 0))),
            ('image_format', (T.BASE, ('string', 0))),
            ('map_mode', (T.BASE, ('string', 0))),
            ('free_thresh', (T.BASE, ('float32', 0))),
            ('occupied_thresh', (T.BASE, ('float32', 0))),
        ],
    ),
    'nav2_msgs/srv/SaveMap_Response': (
        [],
        [
            ('result', (T.BASE, ('bool', 0))),
        ],
    ),
    'nav2_msgs/srv/SetInitialPose_Request': (
        [],
        [
            ('pose', (T.NAME, 'geometry_msgs/msg/PoseWithCovarianceStamped')),
        ],
    ),
    'nav2_msgs/srv/SetInitialPose_Response': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'nav2_msgs/srv/msg/ClearCostmapAroundRobot_Request': (
        [],
        [
            ('reset_distance', (T.BASE, ('float32', 0))),
        ],
    ),
    'nav2_msgs/srv/msg/ClearCostmapAroundRobot_Response': (
        [],
        [
            ('response', (T.NAME, 'std_msgs/msg/Empty')),
        ],
    ),
    'nav2_msgs/srv/msg/ClearCostmapExceptRegion_Request': (
        [],
        [
            ('reset_distance', (T.BASE, ('float32', 0))),
        ],
    ),
    'nav2_msgs/srv/msg/ClearCostmapExceptRegion_Response': (
        [],
        [
            ('response', (T.NAME, 'std_msgs/msg/Empty')),
        ],
    ),
    'nav2_msgs/srv/msg/ClearEntireCostmap_Request': (
        [],
        [
            ('request', (T.NAME, 'std_msgs/msg/Empty')),
        ],
    ),
    'nav2_msgs/srv/msg/ClearEntireCostmap_Response': (
        [],
        [
            ('response', (T.NAME, 'std_msgs/msg/Empty')),
        ],
    ),
    'nav2_msgs/srv/msg/GetCostmap_Request': (
        [],
        [
            ('specs', (T.NAME, 'nav2_msgs/msg/CostmapMetaData')),
        ],
    ),
    'nav2_msgs/srv/msg/GetCostmap_Response': (
        [],
        [
            ('map', (T.NAME, 'nav2_msgs/msg/Costmap')),
        ],
    ),
    'nav2_msgs/srv/msg/IsPathValid_Request': (
        [],
        [
            ('path', (T.NAME, 'nav_msgs/msg/Path')),
        ],
    ),
    'nav2_msgs/srv/msg/IsPathValid_Response': (
        [],
        [
            ('is_valid', (T.BASE, ('bool', 0))),
            ('invalid_pose_indices', (T.SEQUENCE, ((T.BASE, ('int32', 0)), 0))),
        ],
    ),
    'nav2_msgs/srv/msg/LoadMap_Request': (
        [],
        [
            ('map_url', (T.BASE, ('string', 0))),
        ],
    ),
    'nav2_msgs/srv/msg/LoadMap_Response': (
        [
            ('RESULT_SUCCESS', 'uint8', 0),
            ('RESULT_MAP_DOES_NOT_EXIST', 'uint8', 1),
            ('RESULT_INVALID_MAP_DATA', 'uint8', 2),
            ('RESULT_INVALID_MAP_METADATA', 'uint8', 3),
            ('RESULT_UNDEFINED_FAILURE', 'uint8', 255),
        ],
        [
            ('map', (T.NAME, 'nav_msgs/msg/OccupancyGrid')),
            ('result', (T.BASE, ('uint8', 0))),
        ],
    ),
    'nav2_msgs/srv/msg/ManageLifecycleNodes_Request': (
        [
            ('STARTUP', 'uint8', 0),
            ('PAUSE', 'uint8', 1),
            ('RESUME', 'uint8', 2),
            ('RESET', 'uint8', 3),
            ('SHUTDOWN', 'uint8', 4),
        ],
        [
            ('command', (T.BASE, ('uint8', 0))),
        ],
    ),
    'nav2_msgs/srv/msg/ManageLifecycleNodes_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
        ],
    ),
    'nav2_msgs/srv/msg/SaveMap_Request': (
        [],
        [
            ('map_topic', (T.BASE, ('string', 0))),
            ('map_url', (T.BASE, ('string', 0))),
            ('image_format', (T.BASE, ('string', 0))),
            ('map_mode', (T.BASE, ('string', 0))),
            ('free_thresh', (T.BASE, ('float32', 0))),
            ('occupied_thresh', (T.BASE, ('float32', 0))),
        ],
    ),
    'nav2_msgs/srv/msg/SaveMap_Response': (
        [],
        [
            ('result', (T.BASE, ('bool', 0))),
        ],
    ),
    'nav2_msgs/srv/msg/SetInitialPose_Request': (
        [],
        [
            ('pose', (T.NAME, 'geometry_msgs/msg/PoseWithCovarianceStamped')),
        ],
    ),
    'nav2_msgs/srv/msg/SetInitialPose_Response': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'nav_2d_msgs/msg/Path2D': (
        [],
        [
            ('header', (T.NAME, 'std_msgs/msg/Header')),
            ('poses', (T.SEQUENCE, ((T.NAME, 'geometry_msgs/msg/Pose2D'), 0))),
        ],
    ),
    'nav_2d_msgs/msg/Pose2D32': (
        [],
        [
            ('x', (T.BASE, ('float32', 0))),
            ('y', (T.BASE, ('float32', 0))),
            ('theta', (T.BASE, ('float32', 0))),
        ],
    ),
    'nav_2d_msgs/msg/Pose2DStamped': (
        [],
        [
            ('header', (T.NAME, 'std_msgs/msg/Header')),
            ('pose', (T.NAME, 'geometry_msgs/msg/Pose2D')),
        ],
    ),
    'nav_2d_msgs/msg/Twist2D': (
        [],
        [
            ('x', (T.BASE, ('float64', 0))),
            ('y', (T.BASE, ('float64', 0))),
            ('theta', (T.BASE, ('float64', 0))),
        ],
    ),
    'nav_2d_msgs/msg/Twist2D32': (
        [],
        [
            ('x', (T.BASE, ('float32', 0))),
            ('y', (T.BASE, ('float32', 0))),
            ('theta', (T.BASE, ('float32', 0))),
        ],
    ),
    'nav_2d_msgs/msg/Twist2DStamped': (
        [],
        [
            ('header', (T.NAME, 'std_msgs/msg/Header')),
            ('velocity', (T.NAME, 'nav_2d_msgs/msg/Twist2D')),
        ],
    ),
    'nav_msgs/msg/Goals': (
        [],
        [
            ('header', (T.NAME, 'std_msgs/msg/Header')),
            ('goals', (T.SEQUENCE, ((T.NAME, 'geometry_msgs/msg/PoseStamped'), 0))),
        ],
    ),
    'nav_msgs/srv/GetMap_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'nav_msgs/srv/GetMap_Response': (
        [],
        [
            ('map', (T.NAME, 'nav_msgs/msg/OccupancyGrid')),
        ],
    ),
    'nav_msgs/srv/GetPlan_Request': (
        [],
        [
            ('start', (T.NAME, 'geometry_msgs/msg/PoseStamped')),
            ('goal', (T.NAME, 'geometry_msgs/msg/PoseStamped')),
            ('tolerance', (T.BASE, ('float32', 0))),
        ],
    ),
    'nav_msgs/srv/GetPlan_Response': (
        [],
        [
            ('plan', (T.NAME, 'nav_msgs/msg/Path')),
        ],
    ),
    'nav_msgs/srv/LoadMap_Request': (
        [],
        [
            ('map_url', (T.BASE, ('string', 0))),
        ],
    ),
    'nav_msgs/srv/LoadMap_Response': (
        [
            ('RESULT_SUCCESS', 'uint8', 0),
            ('RESULT_MAP_DOES_NOT_EXIST', 'uint8', 1),
            ('RESULT_INVALID_MAP_DATA', 'uint8', 2),
            ('RESULT_INVALID_MAP_METADATA', 'uint8', 3),
            ('RESULT_UNDEFINED_FAILURE', 'uint8', 255),
        ],
        [
            ('map', (T.NAME, 'nav_msgs/msg/OccupancyGrid')),
            ('result', (T.BASE, ('uint8', 0))),
        ],
    ),
    'nav_msgs/srv/SetMap_Request': (
        [],
        [
            ('map', (T.NAME, 'nav_msgs/msg/OccupancyGrid')),
            ('initial_pose', (T.NAME, 'geometry_msgs/msg/PoseWithCovarianceStamped')),
        ],
    ),
    'nav_msgs/srv/SetMap_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
        ],
    ),
    'nav_msgs/srv/msg/GetMap_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'nav_msgs/srv/msg/GetMap_Response': (
        [],
        [
            ('map', (T.NAME, 'nav_msgs/srv/msg/OccupancyGrid')),
        ],
    ),
    'nav_msgs/srv/msg/GetPlan_Request': (
        [],
        [
            ('start', (T.NAME, 'geometry_msgs/msg/PoseStamped')),
            ('goal', (T.NAME, 'geometry_msgs/msg/PoseStamped')),
            ('tolerance', (T.BASE, ('float32', 0))),
        ],
    ),
    'nav_msgs/srv/msg/GetPlan_Response': (
        [],
        [
            ('plan', (T.NAME, 'nav_msgs/srv/msg/Path')),
        ],
    ),
    'nav_msgs/srv/msg/LoadMap_Request': (
        [],
        [
            ('map_url', (T.BASE, ('string', 0))),
        ],
    ),
    'nav_msgs/srv/msg/LoadMap_Response': (
        [
            ('RESULT_SUCCESS', 'uint8', 0),
            ('RESULT_MAP_DOES_NOT_EXIST', 'uint8', 1),
            ('RESULT_INVALID_MAP_DATA', 'uint8', 2),
            ('RESULT_INVALID_MAP_METADATA', 'uint8', 3),
            ('RESULT_UNDEFINED_FAILURE', 'uint8', 255),
        ],
        [
            ('map', (T.NAME, 'nav_msgs/msg/OccupancyGrid')),
            ('result', (T.BASE, ('uint8', 0))),
        ],
    ),
    'nav_msgs/srv/msg/SetMap_Request': (
        [],
        [
            ('map', (T.NAME, 'nav_msgs/msg/OccupancyGrid')),
            ('initial_pose', (T.NAME, 'geometry_msgs/msg/PoseWithCovarianceStamped')),
        ],
    ),
    'nav_msgs/srv/msg/SetMap_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
        ],
    ),
    'pcl_msgs/msg/ModelCoefficients': (
        [],
        [
            ('header', (T.NAME, 'std_msgs/msg/Header')),
            ('values', (T.SEQUENCE, ((T.BASE, ('float32', 0)), 0))),
        ],
    ),
    'pcl_msgs/msg/PointIndices': (
        [],
        [
            ('header', (T.NAME, 'std_msgs/msg/Header')),
            ('indices', (T.SEQUENCE, ((T.BASE, ('int32', 0)), 0))),
        ],
    ),
    'pcl_msgs/msg/PolygonMesh': (
        [],
        [
            ('header', (T.NAME, 'std_msgs/msg/Header')),
            ('cloud', (T.NAME, 'sensor_msgs/msg/PointCloud2')),
            ('polygons', (T.SEQUENCE, ((T.NAME, 'pcl_msgs/msg/Vertices'), 0))),
        ],
    ),
    'pcl_msgs/msg/Vertices': (
        [],
        [
            ('vertices', (T.SEQUENCE, ((T.BASE, ('uint32', 0)), 0))),
        ],
    ),
    'pcl_msgs/srv/UpdateFilename_Request': (
        [],
        [
            ('filename', (T.BASE, ('string', 0))),
        ],
    ),
    'pcl_msgs/srv/UpdateFilename_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
        ],
    ),
    'pcl_msgs/srv/msg/UpdateFilename_Request': (
        [],
        [
            ('filename', (T.BASE, ('string', 0))),
        ],
    ),
    'pcl_msgs/srv/msg/UpdateFilename_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
        ],
    ),
    'pendulum_msgs/msg/JointCommand': (
        [],
        [
            ('position', (T.BASE, ('float64', 0))),
        ],
    ),
    'pendulum_msgs/msg/JointState': (
        [],
        [
            ('position', (T.BASE, ('float64', 0))),
            ('velocity', (T.BASE, ('float64', 0))),
            ('effort', (T.BASE, ('float64', 0))),
        ],
    ),
    'pendulum_msgs/msg/RttestResults': (
        [],
        [
            ('stamp', (T.NAME, 'builtin_interfaces/msg/Time')),
            ('command', (T.NAME, 'pendulum_msgs/msg/JointCommand')),
            ('state', (T.NAME, 'pendulum_msgs/msg/JointState')),
            ('cur_latency', (T.BASE, ('uint64', 0))),
            ('mean_latency', (T.BASE, ('float64', 0))),
            ('min_latency', (T.BASE, ('uint64', 0))),
            ('max_latency', (T.BASE, ('uint64', 0))),
            ('minor_pagefaults', (T.BASE, ('uint64', 0))),
            ('major_pagefaults', (T.BASE, ('uint64', 0))),
        ],
    ),
    'rcl_interfaces/msg/ParameterValue': (
        [],
        [
            ('type', (T.BASE, ('uint8', 0))),
            ('bool_value', (T.BASE, ('bool', 0))),
            ('integer_value', (T.BASE, ('int64', 0))),
            ('double_value', (T.BASE, ('float64', 0))),
            ('string_value', (T.BASE, ('string', 0))),
            ('byte_array_value', (T.SEQUENCE, ((T.NAME, 'octet'), 0))),
            ('bool_array_value', (T.SEQUENCE, ((T.BASE, ('bool', 0)), 0))),
            ('integer_array_value', (T.SEQUENCE, ((T.BASE, ('int64', 0)), 0))),
            ('double_array_value', (T.SEQUENCE, ((T.BASE, ('float64', 0)), 0))),
            ('string_array_value', (T.SEQUENCE, ((T.BASE, ('string', 0)), 0))),
        ],
    ),
    'rcl_interfaces/srv/DescribeParameters_Request': (
        [],
        [
            ('names', (T.SEQUENCE, ((T.BASE, ('string', 0)), 0))),
        ],
    ),
    'rcl_interfaces/srv/DescribeParameters_Response': (
        [],
        [
            ('descriptors', (T.SEQUENCE, ((T.NAME, 'rcl_interfaces/msg/ParameterDescriptor'), 0))),
        ],
    ),
    'rcl_interfaces/srv/GetParameterTypes_Request': (
        [],
        [
            ('names', (T.SEQUENCE, ((T.BASE, ('string', 0)), 0))),
        ],
    ),
    'rcl_interfaces/srv/GetParameterTypes_Response': (
        [],
        [
            ('types', (T.SEQUENCE, ((T.BASE, ('uint8', 0)), 0))),
        ],
    ),
    'rcl_interfaces/srv/GetParameters_Request': (
        [],
        [
            ('names', (T.SEQUENCE, ((T.BASE, ('string', 0)), 0))),
        ],
    ),
    'rcl_interfaces/srv/GetParameters_Response': (
        [],
        [
            ('values', (T.SEQUENCE, ((T.NAME, 'rcl_interfaces/msg/ParameterValue'), 0))),
        ],
    ),
    'rcl_interfaces/srv/ListParameters_Request': (
        [
            ('DEPTH_RECURSIVE', 'uint64', 0),
        ],
        [
            ('prefixes', (T.SEQUENCE, ((T.BASE, ('string', 0)), 0))),
            ('depth', (T.BASE, ('uint64', 0))),
        ],
    ),
    'rcl_interfaces/srv/ListParameters_Response': (
        [],
        [
            ('result', (T.NAME, 'rcl_interfaces/msg/ListParametersResult')),
        ],
    ),
    'rcl_interfaces/srv/SetParametersAtomically_Request': (
        [],
        [
            ('parameters', (T.SEQUENCE, ((T.NAME, 'rcl_interfaces/msg/Parameter'), 0))),
        ],
    ),
    'rcl_interfaces/srv/SetParametersAtomically_Response': (
        [],
        [
            ('result', (T.NAME, 'rcl_interfaces/msg/SetParametersResult')),
        ],
    ),
    'rcl_interfaces/srv/SetParameters_Request': (
        [],
        [
            ('parameters', (T.SEQUENCE, ((T.NAME, 'rcl_interfaces/msg/Parameter'), 0))),
        ],
    ),
    'rcl_interfaces/srv/SetParameters_Response': (
        [],
        [
            ('results', (T.SEQUENCE, ((T.NAME, 'rcl_interfaces/msg/SetParametersResult'), 0))),
        ],
    ),
    'rcl_interfaces/srv/msg/DescribeParameters_Request': (
        [],
        [
            ('names', (T.SEQUENCE, ((T.BASE, ('string', 0)), 0))),
        ],
    ),
    'rcl_interfaces/srv/msg/DescribeParameters_Response': (
        [],
        [
            ('descriptors', (T.SEQUENCE, ((T.NAME, 'rcl_interfaces/srv/msg/ParameterDescriptor'), 0))),
        ],
    ),
    'rcl_interfaces/srv/msg/GetParameterTypes_Request': (
        [],
        [
            ('names', (T.SEQUENCE, ((T.BASE, ('string', 0)), 0))),
        ],
    ),
    'rcl_interfaces/srv/msg/GetParameterTypes_Response': (
        [],
        [
            ('types', (T.SEQUENCE, ((T.BASE, ('uint8', 0)), 0))),
        ],
    ),
    'rcl_interfaces/srv/msg/GetParameters_Request': (
        [],
        [
            ('names', (T.SEQUENCE, ((T.BASE, ('string', 0)), 0))),
        ],
    ),
    'rcl_interfaces/srv/msg/GetParameters_Response': (
        [],
        [
            ('values', (T.SEQUENCE, ((T.NAME, 'rcl_interfaces/srv/msg/ParameterValue'), 0))),
        ],
    ),
    'rcl_interfaces/srv/msg/ListParameters_Request': (
        [
            ('DEPTH_RECURSIVE', 'uint64', 0),
        ],
        [
            ('prefixes', (T.SEQUENCE, ((T.BASE, ('string', 0)), 0))),
            ('depth', (T.BASE, ('uint64', 0))),
        ],
    ),
    'rcl_interfaces/srv/msg/ListParameters_Response': (
        [],
        [
            ('result', (T.NAME, 'rcl_interfaces/srv/msg/ListParametersResult')),
        ],
    ),
    'rcl_interfaces/srv/msg/SetParametersAtomically_Request': (
        [],
        [
            ('parameters', (T.SEQUENCE, ((T.NAME, 'rcl_interfaces/srv/msg/Parameter'), 0))),
        ],
    ),
    'rcl_interfaces/srv/msg/SetParametersAtomically_Response': (
        [],
        [
            ('result', (T.NAME, 'rcl_interfaces/srv/msg/SetParametersResult')),
        ],
    ),
    'rcl_interfaces/srv/msg/SetParameters_Request': (
        [],
        [
            ('parameters', (T.SEQUENCE, ((T.NAME, 'rcl_interfaces/srv/msg/Parameter'), 0))),
        ],
    ),
    'rcl_interfaces/srv/msg/SetParameters_Response': (
        [],
        [
            ('results', (T.SEQUENCE, ((T.NAME, 'rcl_interfaces/srv/msg/SetParametersResult'), 0))),
        ],
    ),
    'rosbag2_interfaces/srv/Burst_Request': (
        [],
        [
            ('num_messages', (T.BASE, ('uint64', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/Burst_Response': (
        [],
        [
            ('actually_burst', (T.BASE, ('uint64', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/GetRate_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/GetRate_Response': (
        [],
        [
            ('rate', (T.BASE, ('float64', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/IsPaused_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/IsPaused_Response': (
        [],
        [
            ('paused', (T.BASE, ('bool', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/Pause_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/Pause_Response': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/PlayNext_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/PlayNext_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/Resume_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/Resume_Response': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/Seek_Request': (
        [],
        [
            ('time', (T.NAME, 'builtin_interfaces/msg/Time')),
        ],
    ),
    'rosbag2_interfaces/srv/Seek_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/SetRate_Request': (
        [],
        [
            ('rate', (T.BASE, ('float64', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/SetRate_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/Snapshot_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/Snapshot_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/TogglePaused_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/TogglePaused_Response': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/msg/Burst_Request': (
        [],
        [
            ('num_messages', (T.BASE, ('uint64', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/msg/Burst_Response': (
        [],
        [
            ('actually_burst', (T.BASE, ('uint64', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/msg/GetRate_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/msg/GetRate_Response': (
        [],
        [
            ('rate', (T.BASE, ('float64', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/msg/IsPaused_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/msg/IsPaused_Response': (
        [],
        [
            ('paused', (T.BASE, ('bool', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/msg/Pause_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/msg/Pause_Response': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/msg/PlayNext_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/msg/PlayNext_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/msg/Resume_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/msg/Resume_Response': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/msg/Seek_Request': (
        [],
        [
            ('time', (T.NAME, 'builtin_interfaces/msg/Time')),
        ],
    ),
    'rosbag2_interfaces/srv/msg/Seek_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/msg/SetRate_Request': (
        [],
        [
            ('rate', (T.BASE, ('float64', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/msg/SetRate_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/msg/Snapshot_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/msg/Snapshot_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/msg/TogglePaused_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'rosbag2_interfaces/srv/msg/TogglePaused_Response': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'sensor_msgs/srv/SetCameraInfo_Request': (
        [],
        [
            ('camera_info', (T.NAME, 'sensor_msgs/msg/CameraInfo')),
        ],
    ),
    'sensor_msgs/srv/SetCameraInfo_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
            ('status_message', (T.BASE, ('string', 0))),
        ],
    ),
    'sensor_msgs/srv/msg/SetCameraInfo_Request': (
        [],
        [
            ('camera_info', (T.NAME, 'sensor_msgs/msg/CameraInfo')),
        ],
    ),
    'sensor_msgs/srv/msg/SetCameraInfo_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
            ('status_message', (T.BASE, ('string', 0))),
        ],
    ),
    'std_msgs/msg/ByteMultiArray': (
        [],
        [
            ('layout', (T.NAME, 'std_msgs/msg/MultiArrayLayout')),
            ('data', (T.SEQUENCE, ((T.NAME, 'octet'), 0))),
        ],
    ),
    'std_msgs/msg/Char': (
        [],
        [
            ('data', (T.BASE, ('uint8', 0))),
        ],
    ),
    'std_srvs/srv/Empty_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'std_srvs/srv/Empty_Response': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'std_srvs/srv/SetBool_Request': (
        [],
        [
            ('data', (T.BASE, ('bool', 0))),
        ],
    ),
    'std_srvs/srv/SetBool_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
            ('message', (T.BASE, ('string', 0))),
        ],
    ),
    'std_srvs/srv/Trigger_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'std_srvs/srv/Trigger_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
            ('message', (T.BASE, ('string', 0))),
        ],
    ),
    'std_srvs/srv/msg/Empty_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'std_srvs/srv/msg/Empty_Response': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'std_srvs/srv/msg/SetBool_Request': (
        [],
        [
            ('data', (T.BASE, ('bool', 0))),
        ],
    ),
    'std_srvs/srv/msg/SetBool_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
            ('message', (T.BASE, ('string', 0))),
        ],
    ),
    'std_srvs/srv/msg/Trigger_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'std_srvs/srv/msg/Trigger_Response': (
        [],
        [
            ('success', (T.BASE, ('bool', 0))),
            ('message', (T.BASE, ('string', 0))),
        ],
    ),
    'tf2_msgs/action/LookupTransform_Feedback': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'tf2_msgs/action/LookupTransform_Goal': (
        [],
        [
            ('target_frame', (T.BASE, ('string', 0))),
            ('source_frame', (T.BASE, ('string', 0))),
            ('source_time', (T.NAME, 'builtin_interfaces/msg/Time')),
            ('timeout', (T.NAME, 'builtin_interfaces/msg/Duration')),
            ('target_time', (T.NAME, 'builtin_interfaces/msg/Time')),
            ('fixed_frame', (T.BASE, ('string', 0))),
            ('advanced', (T.BASE, ('bool', 0))),
        ],
    ),
    'tf2_msgs/action/LookupTransform_Result': (
        [],
        [
            ('transform', (T.NAME, 'geometry_msgs/msg/TransformStamped')),
            ('error', (T.NAME, 'tf2_msgs/msg/TF2Error')),
        ],
    ),
    'tf2_msgs/srv/FrameGraph_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'tf2_msgs/srv/FrameGraph_Response': (
        [],
        [
            ('frame_yaml', (T.BASE, ('string', 0))),
        ],
    ),
    'tf2_msgs/srv/msg/FrameGraph_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'tf2_msgs/srv/msg/FrameGraph_Response': (
        [],
        [
            ('frame_yaml', (T.BASE, ('string', 0))),
        ],
    ),
    'turtlesim/action/RotateAbsolute_Feedback': (
        [],
        [
            ('remaining', (T.BASE, ('float32', 0))),
        ],
    ),
    'turtlesim/action/RotateAbsolute_Goal': (
        [],
        [
            ('theta', (T.BASE, ('float32', 0))),
        ],
    ),
    'turtlesim/action/RotateAbsolute_Result': (
        [],
        [
            ('delta', (T.BASE, ('float32', 0))),
        ],
    ),
    'turtlesim/msg/Color': (
        [],
        [
            ('r', (T.BASE, ('uint8', 0))),
            ('g', (T.BASE, ('uint8', 0))),
            ('b', (T.BASE, ('uint8', 0))),
        ],
    ),
    'turtlesim/msg/Pose': (
        [],
        [
            ('x', (T.BASE, ('float32', 0))),
            ('y', (T.BASE, ('float32', 0))),
            ('theta', (T.BASE, ('float32', 0))),
            ('linear_velocity', (T.BASE, ('float32', 0))),
            ('angular_velocity', (T.BASE, ('float32', 0))),
        ],
    ),
    'turtlesim/srv/Kill_Request': (
        [],
        [
            ('name', (T.BASE, ('string', 0))),
        ],
    ),
    'turtlesim/srv/Kill_Response': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'turtlesim/srv/SetPen_Request': (
        [],
        [
            ('r', (T.BASE, ('uint8', 0))),
            ('g', (T.BASE, ('uint8', 0))),
            ('b', (T.BASE, ('uint8', 0))),
            ('width', (T.BASE, ('uint8', 0))),
            ('off', (T.BASE, ('uint8', 0))),
        ],
    ),
    'turtlesim/srv/SetPen_Response': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'turtlesim/srv/Spawn_Request': (
        [],
        [
            ('x', (T.BASE, ('float32', 0))),
            ('y', (T.BASE, ('float32', 0))),
            ('theta', (T.BASE, ('float32', 0))),
            ('name', (T.BASE, ('string', 0))),
        ],
    ),
    'turtlesim/srv/Spawn_Response': (
        [],
        [
            ('name', (T.BASE, ('string', 0))),
        ],
    ),
    'turtlesim/srv/TeleportAbsolute_Request': (
        [],
        [
            ('x', (T.BASE, ('float32', 0))),
            ('y', (T.BASE, ('float32', 0))),
            ('theta', (T.BASE, ('float32', 0))),
        ],
    ),
    'turtlesim/srv/TeleportAbsolute_Response': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'turtlesim/srv/TeleportRelative_Request': (
        [],
        [
            ('linear', (T.BASE, ('float32', 0))),
            ('angular', (T.BASE, ('float32', 0))),
        ],
    ),
    'turtlesim/srv/TeleportRelative_Response': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'turtlesim/srv/msg/Kill_Request': (
        [],
        [
            ('name', (T.BASE, ('string', 0))),
        ],
    ),
    'turtlesim/srv/msg/Kill_Response': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'turtlesim/srv/msg/SetPen_Request': (
        [],
        [
            ('r', (T.BASE, ('uint8', 0))),
            ('g', (T.BASE, ('uint8', 0))),
            ('b', (T.BASE, ('uint8', 0))),
            ('width', (T.BASE, ('uint8', 0))),
            ('off', (T.BASE, ('uint8', 0))),
        ],
    ),
    'turtlesim/srv/msg/SetPen_Response': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'turtlesim/srv/msg/Spawn_Request': (
        [],
        [
            ('x', (T.BASE, ('float32', 0))),
            ('y', (T.BASE, ('float32', 0))),
            ('theta', (T.BASE, ('float32', 0))),
            ('name', (T.BASE, ('string', 0))),
        ],
    ),
    'turtlesim/srv/msg/Spawn_Response': (
        [],
        [
            ('name', (T.BASE, ('string', 0))),
        ],
    ),
    'turtlesim/srv/msg/TeleportAbsolute_Request': (
        [],
        [
            ('x', (T.BASE, ('float32', 0))),
            ('y', (T.BASE, ('float32', 0))),
            ('theta', (T.BASE, ('float32', 0))),
        ],
    ),
    'turtlesim/srv/msg/TeleportAbsolute_Response': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'turtlesim/srv/msg/TeleportRelative_Request': (
        [],
        [
            ('linear', (T.BASE, ('float32', 0))),
            ('angular', (T.BASE, ('float32', 0))),
        ],
    ),
    'turtlesim/srv/msg/TeleportRelative_Response': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'visualization_msgs/srv/GetInteractiveMarkers_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'visualization_msgs/srv/GetInteractiveMarkers_Response': (
        [],
        [
            ('sequence_number', (T.BASE, ('uint64', 0))),
            ('markers', (T.SEQUENCE, ((T.NAME, 'visualization_msgs/msg/InteractiveMarker'), 0))),
        ],
    ),
    'visualization_msgs/srv/msg/GetInteractiveMarkers_Request': (
        [],
        [
            ('structure_needs_at_least_one_member', (T.BASE, ('uint8', 0))),
        ],
    ),
    'visualization_msgs/srv/msg/GetInteractiveMarkers_Response': (
        [],
        [
            ('sequence_number', (T.BASE, ('uint64', 0))),
            ('markers', (T.SEQUENCE, ((T.NAME, 'visualization_msgs/srv/msg/InteractiveMarker'), 0))),
        ],
    ),
}

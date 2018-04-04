/*
© Siemens AG, 2017-2018
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public static class MessageTypes
    {
        public static readonly Dictionary<string, Type> Dictionary = new Dictionary<string, Type>
        {
            { "actionlib_msgs/GoalID", typeof(ActionGoalID) },
            { "actionlib_msgs/GoalStatus", typeof(ActionGoalStatus) },
            { "actionlib_msgs/GoalStatusArray", typeof(ActionGoalStatusArray) },
            { "control_msgs/GripperCommand", typeof(ControlGripperCommand) },
            { "control_msgs/GripperCommandActionGoal", typeof(ControlGripperCommandActionGoal) },
            { "control_msgs/GripperCommandGoal", typeof(ControlGripperCommandGoal) },
            { "control_msgs/FollowJointTrajectoryActionGoal", typeof(ControlFollowJointTrajectoryActionGoal) },
            { "control_msgs/FollowJointTrajectoryGoal", typeof(ControlFollowJointTrajectoryGoal) },
            { "control_msgs/JointTolerance", typeof(ControlJointTolerance) },
            { "geometry_msgs/Accel", typeof(GeometryAccel) },
            { "geometry_msgs/GeometryTransform", typeof(GeometryTransform) },
            { "geometry_msgs/Point", typeof(GeometryPoint) },
            { "geometry_msgs/Pose", typeof(GeometryPose) },
            { "geometry_msgs/PoseStamped", typeof(GeometryPoseStamped) },
            { "geometry_msgs/PoseWithCovariance",typeof(GeometryPoseWithCovariance) },
            { "geometry_msgs/Quaternion",typeof(GeometryQuaternion) },
            { "geometry_msgs/TransformStamped", typeof(GeometryTransformStamped) },
            { "geometry_msgs/Twist", typeof(GeometryTwist) },
            { "geometry_msgs/TwistWithCovariance", typeof(GeometryTwistWithCovariance) },
            { "geometry_msgs/Vector3", typeof(GeometryVector3) },
            { "move_base_msgs/MoveBaseActionFeedback", typeof(MoveBaseActionFeedback) },
            { "move_base_msgs/MoveBaseActionGoal", typeof (MoveBaseActionGoal) },
            { "move_base_msgs/MoveBaseActionResult", typeof(MoveBaseActionResult) },
            { "move_base_msgs/MoveBaseFeedback", typeof(MoveBaseFeedback) },
            { "move_base_msgs/MoveBaseGoal", typeof (MoveBaseGoal) },
            { "move_base_msgs/MoveBaseResult", typeof(MoveBaseResult) },
            { "moveit_msgs/ExecuteTrajectoryActionGoal", typeof(MoveItExecuteTrajectoryActionGoal) },
            { "moveit_msgs/ExecutableTrajectoryGoal", typeof(MoveItExecutableTrajectoryGoal) },
            { "moveit_msgs/RobotTrajectory", typeof(MoveItRobotTrajectory) },
            { "nav_msgs/MapMetaData", typeof(NavigationMapMetaData) },
            { "nav_msgs/OccupancyGrid", typeof(NavigationOccupancyGrid)},
            { "nav_msgs/Odometry", typeof(NavigationOdometry) },
            { "nav_msgs/Path", typeof(NavigationPath) },
            { "sensor_msgs/CompressedImage", typeof(SensorCompressedImage) },
            { "sensor_msgs/Image", typeof(SensorImage) },
            { "sensor_msgs/JointState", typeof(SensorJointStates) },
            { "sensor_msgs/Joy", typeof(SensorJoy) },
            { "sensor_msgs/LaserScan", typeof(SensorLaserScan) },
            { "sensor_msgs/PointCloud2",typeof(SensorPointCloud2) },
            { "sensor_msgs/PointField", typeof(SensorPointField) },
            { "std_msgs/Bool", typeof(StandardBool) },
            { "std_msgs/Duration", typeof(StandardDuration) },
            { "std_msgs/Header", typeof(StandardHeader) },
            { "std_msgs/String", typeof(StandardString) },
            { "std_msgs/Time", typeof(StandardTime) },
            { "tf2_msgs/TFMessage", typeof(TFMessage) },
            { "trajectory_msgs/JointTrajectory", typeof(JointTrajectory) },
            { "trajectory_msgs/JointTrajectoryPoint", typeof(JointTrajectoryPoint) },
            { "trajectory_msgs/MultiDOFJointTrajectory", typeof(MultiDOFJointTrajectory) },
            { "trajectory_msgs/MultiDOFJointTrajectoryPoint", typeof(MultiDOFJointTrajectoryPoint) },
        };
        public static string RosMessageType(Type messageType)
        {
            Debug.Assert(messageType != null);
            if (messageType == null)
            {
                return "Unknown";
            }
            return Dictionary.FirstOrDefault(x => x.Value == messageType).Key;
        }
        public static Type MessageType(string rosMessageType)
        {
            Type messageType;
            Dictionary.TryGetValue(rosMessageType, out messageType);
            return messageType;
        }
    }
}

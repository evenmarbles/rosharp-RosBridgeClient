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

using Newtonsoft.Json;

namespace RosSharp.RosBridgeClient
{
    public class Message
    {
        [JsonIgnore]
        public string RosMessageType
        {
            get { return MessageTypes.RosMessageType(GetType()); }
        }
    }


	#region actionlib_msgs
	public class ActionGoalID : Message
	{
		public StandardTime stamp;
		public string id;
		public ActionGoalID()
		{
			stamp = new StandardTime();
			id = "";
		}
	}

	public class ActionGoalStatus : Message
	{
		/*
        uint PENDING = 0;
        uint ACTIVE = 1;
        uint PREEMPTED = 2;
        uint SUCCEEDED = 3;
        uint ABORTED = 4;
        uint REJECTED = 5;
        uint PREEMPTING = 6;
        uint RECALLING = 7;
        uint RECALLED = 8;
        uint LOST = 9;
        */
		public ActionGoalID goal_id;
		public uint status;
		public string text;
		public ActionGoalStatus()
		{
			goal_id = new ActionGoalID();
			status = 0;
			text = "";
		}
	}

	public class ActionGoalStatusArray : Message
	{
		public StandardHeader header;
		public ActionGoalStatus[] status_list;
		public ActionGoalStatusArray()
		{
			header = new StandardHeader();
			status_list = new ActionGoalStatus[0];
		}
	}
	#endregion

	#region control_msgs
	public class ControlGripperCommand : Message
	{
		public float position;
		public float max_effort;
		public ControlGripperCommand()
		{
			position = 0f;
			max_effort = 0f;
		}
	}

	public class ControlGripperCommandActionGoal : Message
	{
		public StandardHeader header;
		public ActionGoalID goal_id;
		public ControlGripperCommandGoal goal;
		public ControlGripperCommandActionGoal()
		{
			header = new StandardHeader();
			goal_id = new ActionGoalID();
			goal = new ControlGripperCommandGoal();
		}
	}

	public class ControlGripperCommandGoal : Message
	{
		public ControlGripperCommand command;
		public ControlGripperCommandGoal()
		{
			command = new ControlGripperCommand();
		}
	}

	public class ControlFollowJointTrajectoryActionGoal : Message
	{
		public StandardHeader header;
		public ActionGoalID goal_id;
		public ControlFollowJointTrajectoryGoal goal;
		public ControlFollowJointTrajectoryActionGoal()
		{
			header = new StandardHeader();
			goal_id = new ActionGoalID();
			goal = new ControlFollowJointTrajectoryGoal();
		}
	}

	public class ControlFollowJointTrajectoryGoal : Message
	{
		public JointTrajectory trajectory;
		public ControlJointTolerance[] path_tolerance;
		public ControlJointTolerance[] goal_tolerance;
		public StandardDuration duration;
		public ControlFollowJointTrajectoryGoal()
		{
			trajectory = new JointTrajectory();
			path_tolerance = new ControlJointTolerance[0];
			goal_tolerance = new ControlJointTolerance[0];
			duration = new StandardDuration();
		}
	}

	public class ControlJointTolerance : Message
	{
		public string name;
		public float position;
		public float velocity;
		public float acceleration;
		public ControlJointTolerance()
		{
			name = "";
			position = 0f;
			velocity = 0f;
			acceleration = 0f;
		}
	}
	#endregion

	#region geometry_msgs
	public class GeometryAccel : Message
	{
		public GeometryVector3 linear;
		public GeometryVector3 angular;
		public GeometryAccel()
		{
			linear = new GeometryVector3();
			angular = new GeometryVector3();
		}
	}

	public class GeometryTransform : Message
	{
		public GeometryVector3 translation;
		public GeometryQuaternion rotation;
		public GeometryTransform()
		{
			translation = new GeometryVector3();
			rotation = new GeometryQuaternion();
		}
	}

	public class GeometryPoint : Message
	{
		public float x;
		public float y;
		public float z;
		public GeometryPoint()
		{
			x = 0;
			y = 0;
			z = 0;
		}
	}

	public class GeometryPose : Message
	{
		public GeometryPoint position;
		public GeometryQuaternion orientation;
		public GeometryPose()
		{
			position = new GeometryPoint();
			orientation = new GeometryQuaternion();
		}
	}

	public class GeometryPoseStamped : Message
	{
		public StandardHeader header;
		public GeometryPose pose;
		public GeometryPoseStamped()
		{
			header = new StandardHeader();
			pose = new GeometryPose();
		}
	}

	public class GeometryPoseWithCovariance : Message
	{
		public GeometryPose pose;
		public float[] covariance;
		public GeometryPoseWithCovariance()
		{
			pose = new GeometryPose();
			covariance = new float[32];
		}
	}

	public class GeometryQuaternion : Message
	{
		public float x;
		public float y;
		public float z;
		public float w;
		public GeometryQuaternion()
		{
			x = 0;
			y = 0;
			z = 0;
			w = 0;
		}
		public GeometryQuaternion(float _x, float _y, float _z, float _w)
		{
			x = _x;
			y = _y;
			z = _z;
			w = _w;
		}
	}

	public class GeometryTransformStamped : Message
	{
		public StandardHeader header;
		public string child_frame_id;
		public GeometryTransform transform;
		public GeometryTransformStamped()
		{
			header = new StandardHeader();
			child_frame_id = "";
			transform = new GeometryTransform();
		}
	}

    public class GeometryTwist : Message
    {
        public GeometryVector3 linear;
        public GeometryVector3 angular;
        public GeometryTwist()
        {
            linear = new GeometryVector3();
            angular = new GeometryVector3();
        }
    }

	public class GeometryTwistWithCovariance : Message
	{
		public GeometryTwist twist;
		public float[] covariance;
		public GeometryTwistWithCovariance()
		{
			twist = new GeometryTwist();
			covariance = new float[32];
		}
	}

    public class GeometryVector3 : Message
    {
        public float x;
        public float y;
        public float z;
        public GeometryVector3()
        {
            x = 0f;
            y = 0f;
            z = 0f;
        }
		public GeometryVector3(float _x, float _y, float _z)
		{
			x = _x;
			y = _y;
			z = _z;
		}
    }
	#endregion // geometry_msgs

	#region move_base_msgs
	public class MoveBaseActionFeedback : Message
	{
		public StandardHeader header;
		public ActionGoalStatus status;
		public MoveBaseFeedback feedback;
		public MoveBaseActionFeedback()
		{
			header = new StandardHeader();
			status = new ActionGoalStatus();
			feedback = new MoveBaseFeedback();
		}
	}

	public class MoveBaseActionGoal : Message
	{
		public StandardHeader header;
		public ActionGoalID goal_id;
		public MoveBaseGoal goal;
		public MoveBaseActionGoal()
		{
			header = new StandardHeader();
			goal_id = new ActionGoalID();
			goal = new MoveBaseGoal();
		}
	}

	public class MoveBaseActionResult : Message
	{
		public StandardHeader header;
		public ActionGoalStatus status;
		public MoveBaseResult result;
		public MoveBaseActionResult()
		{
			header = new StandardHeader();
			status = new ActionGoalStatus();
			result = new MoveBaseResult();
		}
	}

	public class MoveBaseFeedback : Message
	{
		public GeometryPoseStamped base_position;
		public MoveBaseFeedback()
		{
			base_position = new GeometryPoseStamped();
		}
	}

    public class MoveBaseGoal : Message
    {
        public GeometryPoseStamped target_pose;
        public MoveBaseGoal()
        {
            target_pose = new GeometryPoseStamped();
        }
    }

    public class MoveBaseResult : Message
	{
		public MoveBaseResult()
		{
		}
	}
	#endregion // move_base_msgs

	#region moveit_msgs
	public class MoveItExecuteTrajectoryActionGoal : Message
	{
		public StandardHeader header;
		public ActionGoalID goal_id;
		public MoveItExecutableTrajectoryGoal goal;
		public MoveItExecuteTrajectoryActionGoal()
		{
			header = new StandardHeader();
			goal_id = new ActionGoalID();
			goal = new MoveItExecutableTrajectoryGoal();
		}
	}

	public class MoveItExecutableTrajectoryGoal : Message
	{
		public MoveItRobotTrajectory trajectory;
		public MoveItExecutableTrajectoryGoal()
		{
			trajectory = new MoveItRobotTrajectory();
		}
	}

	public class MoveItRobotTrajectory : Message
	{
		public JointTrajectory joint_trajectory;
		public MultiDOFJointTrajectory multi_dof_joint_trajectory;
		public MoveItRobotTrajectory()
		{
			joint_trajectory = new JointTrajectory ();
			multi_dof_joint_trajectory = new MultiDOFJointTrajectory ();
		}
	}
	#endregion // moveit_msgs

	#region nav_msgs
    public class NavigationMapMetaData : Message
    {
        public StandardTime map_load_time;
        public float resolution;
        public uint width;
        public uint height;
        public GeometryPose origin;
		public NavigationMapMetaData()
        {
            map_load_time = null;
            resolution = 0;
            width = 0;
            height = 0;
            origin = new GeometryPose();
        }
    }

    public class NavigationOccupancyGrid : Message
    {
        public StandardHeader header;
        public NavigationMapMetaData info;
        public sbyte[] data;
        public NavigationOccupancyGrid()
        {
            header = new StandardHeader();
            info = new NavigationMapMetaData();
            data = null;
        }
    }

	public class NavigationOdometry : Message
	{
		public StandardHeader header;
		public string child_frame_id;
		public GeometryPoseWithCovariance pose;
		public GeometryTwistWithCovariance twist;
		public NavigationOdometry()
		{
			header = new StandardHeader();
			child_frame_id = "";
			pose = new GeometryPoseWithCovariance();
			twist = new GeometryTwistWithCovariance();
		}
	}

	public class NavigationPath : Message
	{
		public StandardHeader header;
		public GeometryPoseStamped[] poses;
		public NavigationPath()
		{
			header = new StandardHeader();
			poses = new GeometryPoseStamped[0];
		}
	}
	#endregion // nav_msgs

	#region sensor_msgs
	public class SensorCompressedImage : Message
	{
		public StandardHeader header;
		public string format;
		public byte[] data;
		public SensorCompressedImage()
		{
			header = new StandardHeader();
			format = "";
			data = new byte[0];
		}
	}

	public class SensorImage : Message
	{
		public StandardHeader header;
		public int height;
		public int width;
		public string encoding;
		public bool is_bigendian;
		public int step;
		public byte[] data;
		public SensorImage()
		{
			header = new StandardHeader();
			height = 0;
			width = 0;
			encoding = "undefined";
			is_bigendian = false;
			step = 0;
			data = new byte[0];
		}
	}

	public class SensorJointStates : Message
	{
		public StandardHeader header;
		public string[] name;
		public float[] position;
		public float[] velocity;
		public float[] effort;
		public SensorJointStates()
		{
			header = new StandardHeader();
			name = new string[0];
			position = new float[0];
			velocity = new float[0];
			effort = new float[0];
		}
	}

	public class SensorJoy : Message
	{
		public StandardHeader header;
		public float[] axes;
		public int[] buttons;
		public SensorJoy()
		{
			header = new StandardHeader();
			axes = new float[0];
			buttons = new int[0];
		}
	}

	public class SensorLaserScan : Message
	{
		public StandardHeader header;
		public float angle_min;
		public float angle_max;
		public float angle_increment;
		public float time_increment;
		public float scan_time;
		public float range_min;
		public float range_max;
		public float[] ranges;
		public float[] intensities;
		public SensorLaserScan()
		{
			header = new StandardHeader();
			angle_min = 0;
			angle_max = 0;
			angle_increment = 0;
			time_increment = 0;
			scan_time = 0;
			range_min = 0;
			range_max = 0;
			ranges = new float[0];
			intensities = new float[0];
		}
	}

	public class SensorPointCloud2 : Message
	{
		public StandardHeader header;
		public int height;
		public int width;
		public SensorPointField[] fields;
		public bool is_bigendian;
		public int point_step;
		public int row_step;

		public byte[] data;
		public bool is_dense;
		public SensorPointCloud2()
		{
			header = new StandardHeader();
			height = 0;
			width = 0;
			fields = new SensorPointField[0];
			is_bigendian = false;
			point_step = 0;
			row_step = 0;
			is_dense = false;
			data = new byte[0];
		}
	}

	public class SensorPointField : Message
	{
		public int datatype;
		public string name;
		public int offset;
		public int count;
		public SensorPointField()
		{
			datatype = 0;
			name = "";
			offset = 0;
			count = 0;
		}
	}
	#endregion // sensor_msgs

	#region std_msgs
	public class StandardBool : Message
	{
		public bool data;
		public StandardBool()
		{
			data = new bool();
		}
	}

	public class StandardDuration : Message
	{
		public int secs;
		public int nsec;
		public StandardDuration()
		{
			secs = 0;
			nsec = 0;
		}
	}

	public class StandardHeader : Message
	{
		public int seq;
		public StandardTime stamp;
		public string frame_id;
		public StandardHeader()
		{
			seq = 0;
			stamp = new StandardTime();
			frame_id = "";
		}
	}

	public class StandardString : Message
	{
		public string data;
		public StandardString()
		{
			data = "";
		}
	}

	public class StandardTime : Message
	{
		public int secs;
		public int nsecs;
		public StandardTime()
		{
			secs = 0;
			nsecs = 0;
		}
	}
	#endregion // std_msgs

	#region tf2_msgs
	public class TFMessage : Message
	{
		public GeometryTransformStamped[] transforms;
		public TFMessage()
		{
			transforms = new GeometryTransformStamped[0];
		}
	}
	#endregion // tf2_msgs

	#region trajectory_msgs
	public class JointTrajectory : Message
	{
		public StandardHeader header;
		public string[] joint_names;
		public JointTrajectoryPoint[] points;
		public JointTrajectory()
		{
			header = new StandardHeader();
			joint_names = new string[0];
			points = new JointTrajectoryPoint[0];
		}
	}

	public class JointTrajectoryPoint : Message
	{
		public float[] positions;
		public float[] velocities;
		public float[] accelerations;
		public float[] effort;
		public StandardDuration time_from_start;
		public JointTrajectoryPoint()
		{
			positions = new float[0];
			velocities = new float[0];
			accelerations = new float[0];
			effort = new float[0];
			time_from_start = new StandardDuration();
		}
	}

	public class MultiDOFJointTrajectory : Message
	{
		public StandardHeader header;
		public float[] joint_names;
		public MultiDOFJointTrajectoryPoint[] points;
		public MultiDOFJointTrajectory()
		{
			header = new StandardHeader();
			joint_names = new float[0];
			points = new MultiDOFJointTrajectoryPoint[0];
		}
	}

	public class MultiDOFJointTrajectoryPoint : Message
	{
		public GeometryTransform[] transforms;
		public GeometryTwist[] velocities;
		public GeometryTwist[] accelerations;
		public StandardDuration time_from_start;
		public MultiDOFJointTrajectoryPoint()
		{
			transforms = new GeometryTransform[0];
			velocities = new GeometryTwist[0];
			accelerations = new GeometryTwist[0];
			time_from_start = new StandardDuration();
		}
	}
	#endregion // trajectory_msgs

    public class ParamName : Message
    {
        public string name;
        public ParamName(string _name)
        {
            name = _name;
        }
    }

    public class SetParam : Message
    {
        public string name;
        public string value;
        public SetParam(string _name, string _value)
        {
            name = _name;
            value = _value;
        }
    }

    public class ParamValueString : Message
    {
        public string value;
    }

    public class ParamValueByte : Message
    {
        public byte[] value;
    }
}

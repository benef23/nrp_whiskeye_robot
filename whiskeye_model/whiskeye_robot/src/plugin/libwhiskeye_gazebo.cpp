
#include "libwhiskeye_gazebo.h"
#include "whisker_pose.h"

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/sensors/SensorManager.hh>

#include <ignition/math.hh>
using namespace ignition;

#include "gazebo_versions.h"

#include <stdio.h>
#include <string>
using namespace std;

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <whiskeye_msgs/bridge_u.h>
#include <image_transport/image_transport.h>

#define MAX_SENSOR_COUNT 32
#define NUM_50HZ_SENSORS 25

#define __COUT_W cout << "**** WARNING **** "
#define __ERROR(msg) do { cout << "\n**** ERROR **** " << msg << "\n" << endl; throw(msg); } while(false)
#define __UNUSED(param) do { (void)param; } while(false)

struct XY
{
	float x;
	float y;
};

struct WhiskerXY
{
	struct XY xy[ROBOT_FS_SCALE][ROBOT_ROW_COUNT][ROBOT_COL_COUNT];
};



namespace gazebo
{
	struct JointDesc
	{
		string name;
		physics::JointPtr joint;
	};

	struct WhiskerPose
	{
		double o[3]; // joint centre (origin)
		double y[3]; // 100mm off in +ve y (joint axis direction)
		double z[3]; // 100mm off in +ve z (towards whisker tip)
	};

	WORLD_POSE transform_zero_y(WORLD_POSE T, WORLD_POSE q)
	{
		WORLD_POSE Tq = q * T;
		double f = atan2(__POSE_Y(Tq), __POSE_X(Tq));
		T = T * WORLD_POSE(0, 0, 0, 0, 0, -f);
		return T;
	}

	WORLD_POSE transform_zero_x(WORLD_POSE T, WORLD_POSE q)
	{
		WORLD_POSE Tq = q * T;
		double f = atan2(__POSE_X(Tq), __POSE_Z(Tq));
		T = T * WORLD_POSE(0, 0, 0, 0, -f, 0);
		return T;
	}

	WORLD_POSE transform_zero_x2(WORLD_POSE T, WORLD_POSE q)
	{
		WORLD_POSE Tq = q * T;
		double f = atan2(__POSE_X(Tq), __POSE_Y(Tq));
		T = T * WORLD_POSE(0, 0, 0, 0, 0, f);
		return T;
	}

	WORLD_POSE get_whisker_transform(const WhiskerPose* wpose)
	{
		//	construct objects we can use for numerical search
		WORLD_POSE o(wpose->o[0], wpose->o[1], wpose->o[2], 0, 0, 0);
		WORLD_POSE y(wpose->y[0], wpose->y[1], wpose->y[2], 0, 0, 0);
		WORLD_POSE z(wpose->z[0], wpose->z[1], wpose->z[2], 0, 0, 0);

#define __NORM(x) ((x).pos.GetLength())

		//	initial guess puts o at origin
		WORLD_POSE T = __POSE_GET_INVERSE(o);

		/*
		cout << "----------------" << endl;
		cout << (o * T).pos << endl;
		cout << (y * T).pos << " (" << __NORM(y * T) << ")" << endl;
		cout << (z * T).pos << " (" << __NORM(z * T) << ")" << endl;
		*/

		//	zero y coordinate of z
		T = transform_zero_y(T, z);

		/*
		cout << "----------------" << endl;
		cout << (o * T).pos << endl;
		cout << (y * T).pos << " (" << __NORM(y * T) << ")" << endl;
		cout << (z * T).pos << " (" << __NORM(z * T) << ")" << endl;
		*/

		//	zero x coordinate of z
		T = transform_zero_x(T, z);

		/*
		cout << "----------------" << endl;
		cout << (o * T).pos << endl;
		cout << (y * T).pos << " (" << __NORM(y * T) << ")" << endl;
		cout << (z * T).pos << " (" << __NORM(z * T) << ")" << endl;
		*/

		//	zero x coordinate of y
		T = transform_zero_x2(T, y);

		/*
		cout << "----------------" << endl;
		cout << (o * T).pos << endl;
		cout << (y * T).pos << " (" << __NORM(y * T) << ")" << endl;
		cout << (z * T).pos << " (" << __NORM(z * T) << ")" << endl;
		*/

		//	check
		if (__VECTOR_LENGTH(__POSE_POS(o * T) - VECTOR3(0, 0, 0)) > 0.001)
			__ERROR("bad O");
		if (__VECTOR_LENGTH(__POSE_POS(y * T) - VECTOR3(0, 0.1, 0)) > 0.001)
			__ERROR("bad O");
		if (__VECTOR_LENGTH(__POSE_POS(z * T) - VECTOR3(0, 0, 0.1)) > 0.001)
			__ERROR("bad O");

		//	ok
		return T;
	}

	struct Whiskeye_ModelPlugin : public ModelPlugin
	{
		static void ros_init()
		{
			//	initialize layer if not already
			if (!ros::isInitialized())
			{
				cout << "initializing ROS..." << endl;
				int argc = 0;
				char **argv = NULL;
				ros::init(argc, argv, "whiskeye", ros::init_options::NoSigintHandler);

				//	use check() to ensure that we can contact the ROS master,
				//	otherwise we will hang when we create our first NodeHandle
				int count = 10;
				while (count)
				{
					cout << "attempting to contact ROS master..." << endl;
					if (ros::master::check())
						return;

					usleep(1000000);
					count--;
				}

				//	failed to contact master
				__ERROR("could not contact ROS master");
			}
		}

		Whiskeye_ModelPlugin()
			:
			ModelPlugin()
		{
			cout << "Whiskeye_ModelPlugin() [ctor]" << endl;
			ros_init();

			//	create ROS handle
			output.h_ros = new ros::NodeHandle;
		}

		~Whiskeye_ModelPlugin()
		{
			cout << "~Whiskeye_ModelPlugin() [dtor]" << endl;
			delete output.h_ros;
		}

		physics::LinkPtr GetLink(string name)
		{
			physics::LinkPtr ret = state.model->GetLink(name);
			if (ret)
			{
				//	return it
				return ret;
			}

			//	report existing links
			physics::Link_V links = state.model->GetLinks();
			for (uint32_t i=0; i<links.size(); i++)
			{
				physics::LinkPtr link = links[i];
				cout << "link: " << link->GetName() << endl;
			}

			//	fail
			__ERROR("could not find link \"" + name + "\"");
		}

		void connect(event::ConnectionPtr connection)
		{
			if (state.sensors_connected == MAX_SENSOR_COUNT)
				__ERROR("ran out of space to store sensor connections");
			state.connection[state.sensors_connected++] = connection;
		}

		void SetPID(string joint_name, double scale, double dscale)
		{
			double P = 10.0 * scale;
			double I = 0.0 * scale;
			double D = 10.0 * dscale * scale;
			double imax = 0.0 * scale;
			double imin = -imax;
			double cmdMax = 5.0 * scale;
			double cmdMin = -cmdMax;
			common::PID pid(P, I, D, imax, imin, cmdMax, cmdMin);
			state.controller->SetPositionPID(joint_name, pid);
		}

		void SetVelocityPID(string joint_name, double scale, double dscale)
		{
			double P = 10.0 * scale;
			double I = 0.0 * scale;
			double D = 10.0 * dscale * scale;
			double imax = 0.0 * scale;
			double imin = -imax;
			double cmdMax = 5.0 * scale;
			double cmdMin = -cmdMax;
			common::PID pid(P, I, D); //P, I, D, imax, imin, cmdMax, cmdMin);
			state.controller->SetVelocityPID(joint_name, pid);
		}

		JointDesc GetJoint(string joint_name, double scale, double dscale, bool velocity_control = false)
		{
			//	get joints (from joint controller)
			std::map< std::string, physics::JointPtr > joints =
				state.controller->GetJoints();

			//	search
			joint_name = state.model->GetName() + "::" + joint_name;
			for(std::map< std::string, physics::JointPtr >::iterator it = joints.begin();
						it != joints.end(); ++it)
			{
				if (it->first == joint_name)
				{
					//	configure its PID
					if (velocity_control)
						SetVelocityPID(joint_name, scale, dscale);
					else
						SetPID(joint_name, scale, dscale);

					//	return it
					JointDesc desc;
					desc.name = joint_name;
					desc.joint = it->second;
					return desc;
				}
			}

			//	fail
			for(std::map< std::string, physics::JointPtr >::iterator it = joints.begin();
						it != joints.end(); ++it)
			{
				cout << it->first << endl;
			}
			__ERROR("joint \"" + joint_name + "\" not found (see above for list)");
		}

		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
		{
			cout << "Load()" << endl;

			//	store pointer to model
			state.model = _parent;

			//	report node name
			cout << "model instance name: " << state.model->GetName() << endl;

			//	get links
			state.link_body = GetLink("body");

////	SENSORS

			//	sensor summary
			sensors::Sensor_V ss = sensors::SensorManager::Instance()->GetSensors();
			cout << "SensorManager: " << ss.size() << " sensors are " <<
				(sensors::SensorManager::Instance()->SensorsInitialized()
					? "initialized" : "**** not initialized ****") << endl;
			/*
			for (int i=0; i<ss.size(); i++)
			{
				sensors::SensorPtr s = ss[i];
				cout << "sensor: " << s->Name() << endl;
			}
			*/

#define __GET_SENSOR(id, name, callback) do { \
	sens.id = sensors::SensorManager::Instance()->GetSensor(name); \
	if (!sens.id) __ERROR(("sensor not found \"" + string(name) + "\"")); \
	connect(sens.id->ConnectUpdated(boost::bind(&Whiskeye_ModelPlugin::callback, this, sens.id))); \
} while(false)

			//	get sensors
			__GET_SENSOR(imu_body, "imu_body", OnIMUUpdate);
			__GET_SENSOR(cam[0], "cam0", OnCamUpdate);
			__GET_SENSOR(cam[1], "cam1", OnCamUpdate);

			//	get contact sensors
			for (int r=0; r<ROBOT_ROW_COUNT; r++)
			{
				for (int c=0; c<ROBOT_COL_COUNT; c++)
				{
					stringstream ss;
					ss << "whisker" << (r+1) << "_" << (c+1) << "_contact";
					__GET_SENSOR(contact[r][c], ss.str().c_str(), OnContactUpdate);
				}
			}

////	JOINTS

			//	get joint controller
			state.controller = state.model->GetJointController();
			if (!state.controller)
				__ERROR("failed to get joint controller");

			//	find pose joints
			state.joints_pose[0] = GetJoint("pose_x", ROBOT_POSE_PID_P_0, ROBOT_POSE_PID_D, true);
			state.joints_pose[1] = GetJoint("pose_y", ROBOT_POSE_PID_P_1, ROBOT_POSE_PID_D, true);
			state.joints_pose[2] = GetJoint("pose_theta", ROBOT_POSE_PID_P_2, ROBOT_POSE_PID_D, true);

			//	find neck joints
			state.joints_neck[0] = GetJoint("body_neck", ROBOT_NECK_PID_P_0, ROBOT_NECK_PID_D);
			state.joints_neck[1] = GetJoint("neck_gmbl", ROBOT_NECK_PID_P_1, ROBOT_NECK_PID_D);
			state.joints_neck[2] = GetJoint("gmbl_head", ROBOT_NECK_PID_P_2, ROBOT_NECK_PID_D);

			//	find whisker joints
			for (int row=0; row<ROBOT_ROW_COUNT; row++)
			{
				for (int col=0; col<ROBOT_COL_COUNT; col++)
				{
					stringstream ss;
					ss << "head_whisker" << (row+1) << "_" << (col+1);
					state.joints_whisker[row][col] = GetJoint(ss.str(), ROBOT_WHISKER_PID_P, ROBOT_WHISKER_PID_D);
				}
			}

////	WHISKER MAPPING

			/*
				We need the initial state of the whiskers (their "world pose") because we'll
				use this to map contacts in their run-time FOR back to the initial whisker
				FOR for interpretation. To discover this, we set the whole model as "static"
				in the SDF, to make sure we get this code to run before anything moves, then
				we measure these poses here. Thankfully, they are all [0 0 0 0 0 0] (because
				that is how the SDF model is put together) so after discovering this, we can
				retire this code, safe in that knowledge.

				Initial world pose of all whiskers: [0 0 0 0 0 0]
			*/

			/*
			//	for each whisker
			for (int r=0; r<ROBOT_ROW_COUNT; r++)
			{
				for (int c=0; c<ROBOT_COL_COUNT; c++)
				{
					//	get link
					stringstream ss;
					ss << "whisker" << (r+1) << "_" << (c+1);
					physics::LinkPtr link = GetLink(ss.str());
					if (!link)
						__ERROR("whisker link not found");

					//	get pose
					const WORLD_POSE& pose = link->GAZEBO_WORLD_POSE();
					cout << ss.str() << " : " << pose << endl;
				}
			}
			*/

			/*
				Having mapped contacts into the initial FOR for each whisker, we then need
				to map them into some canonical FOR. We choose an FOR pointing directly up
				in +z, since it's intuitive and also since then +x and +y map directly to
				the x/y outputs we ultimately need to generate. This mapping is implicit in
				the STLs that we start with, since each whisker's pose at start-up is encoded
				in its actual position there. We could in principle measure it here, but it's
				somewhat easier to measure it during model creation and pass it in here using
				a header file. In fact, since the API available here is good at handling poses
				and the external file has access to the rotaxe objects, we'll get the model
				creation process to recover three reference points (joint centre, joint axis,
				whisker tip) and then use the API available here to convert that to a pose
				that will map each whisker's physical FOR onto the canonical whisker FOR.
			*/

			//	read whisker_pose
			if (sizeof(whisker_pose) != (ROBOT_WHISKER_COUNT * 9 * sizeof(double)))
				__ERROR("bad whisker pose array size");
			const WhiskerPose* wpose = (const WhiskerPose*) whisker_pose;
			for (int r=0; r<ROBOT_ROW_COUNT; r++)
			{
				for (int c=0; c<ROBOT_COL_COUNT; c++)
				{
					//	find transform that brings this initial whisker
					//	pose back to the canonical pose
					WORLD_POSE T = get_whisker_transform(wpose++);

					//	store
					state.wpose[r][c] = T;
				}
			}

////	INTERFACE

/*
			//	function not currently used
			//	connect to physics update event (1ms, typically)
			connect(event::Events::ConnectWorldUpdateBegin(
					boost::bind(&Whiskeye_ModelPlugin::OnWorldUpdate, this, _1)
					));
*/

			//	publish
			string topic_root = "/whiskeye";
			image_transport::ImageTransport it(*output.h_ros);
#define __ADVERTISE(field, type, name) \
	output.field.pub = output.h_ros->advertise<type> \
		((topic_root + name).c_str(), ROS_SEND_QUEUE_SIZE);
#define __IT_ADVERTISE(field, name) \
	output.field.pub = it.advertise \
		((topic_root + name).c_str(), ROS_SEND_QUEUE_SIZE);
			__ADVERTISE(bumper, std_msgs::Bool, "/body/bumper");
			__ADVERTISE(imu_body, sensor_msgs::Imu, "/body/imu_body");
			__ADVERTISE(pose, geometry_msgs::Pose2D, "/body/pose");
			__ADVERTISE(bridge_u, whiskeye_msgs::bridge_u, "/head/bridge_u");
			__ADVERTISE(xy, std_msgs::Float32MultiArray, "/head/xy");
			__IT_ADVERTISE(cam[0], "/platform/cam0");
			__IT_ADVERTISE(cam[1], "/platform/cam1");

			//	subscribe
			string topic;

			//	to neck
			topic = topic_root + "/head/neck_cmd";
			cout << "subscribe: " << topic << endl;
			input.neck.sub = output.h_ros->subscribe(
				topic.c_str(),
				ROS_RECV_QUEUE_SIZE,
				&Whiskeye_ModelPlugin::callback_neck,
				this
				);

			//	to theta
			topic = topic_root + "/head/theta_cmd";
			cout << "subscribe: " << topic << endl;
			input.theta.sub = output.h_ros->subscribe(
				topic.c_str(),
				ROS_RECV_QUEUE_SIZE,
				&Whiskeye_ModelPlugin::callback_theta,
				this
				);

			//	to cmd_vel
			topic = topic_root + "/body/cmd_vel";
			cout << "subscribe: " << topic << endl;
			input.cmd_vel.sub = output.h_ros->subscribe(
				topic.c_str(),
				ROS_RECV_QUEUE_SIZE,
				&Whiskeye_ModelPlugin::callback_cmd_vel,
				this
				);
		}

		void callback_neck(const std_msgs::Float32MultiArray::ConstPtr& msg)
		{
			//	validate input
			if (msg->data.size() != ROBOT_NECK_COUNT)
			{
				__COUT_W << "input /neck wrong size and is ignored" << endl;
				return;
			}

			//	store
			for (int i=0; i<ROBOT_NECK_COUNT; i++)
				input.neck.cmd[i] = msg->data[i] - state.neck_offset[i];
		}

		void callback_theta(const std_msgs::Float32MultiArray::ConstPtr& msg)
		{
			//	validate input
			if (msg->data.size() != ROBOT_WHISKER_COUNT)
			{
				__COUT_W << "input /theta wrong size and is ignored" << endl;
				return;
			}

			//	store
			for (int i=0; i<ROBOT_WHISKER_COUNT; i++)
				input.theta.cmd[i] = msg->data[i];
		}

		void callback_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg)
		{
			//	store
			input.cmd_vel.vel[0] = msg->linear.x;
			input.cmd_vel.vel[1] = msg->linear.y;
			input.cmd_vel.vel[2] = msg->angular.z;
		}

/*
		//	function not currently used
		void Reset()
		{
			cout << "Reset()" << endl;
		}

		//	function not currently used
		void OnWorldUpdate(const common::UpdateInfo & _info)
		{
			__UNUSED(_info);
			cout << "OnWorldUpdate()" << endl;
		}
*/
		void LoadImage(shared_ptr<sensors::CameraSensor>& cam, sensor_msgs::Image& msg)
		{
			int32_t w = cam->GAZEBO_CAMERA_SENSOR_IMAGE_WIDTH();
			int32_t h = cam->GAZEBO_CAMERA_SENSOR_IMAGE_HEIGHT();
			msg.encoding = "rgb8";
			msg.width = w;
			msg.height = h;
			msg.step = w * 3;
			msg.data.resize(msg.step * h);
			memcpy(&msg.data[0], cam->GAZEBO_CAMERA_SENSOR_IMAGE_DATA(), msg.data.size());
		}

		void OnCamUpdate(sensors::SensorPtr sensor)
		{
			//	get index
			int index = (sensor == sens.cam[0]) ? 0 : 1;

			//	cast
			shared_ptr<sensors::CameraSensor> cam = dynamic_pointer_cast<sensors::CameraSensor> (sensor);

			//	load and publish
			sensor_msgs::Image& msg = output.cam[index].msg;
			LoadImage(cam, msg);
			output.cam[index].pub.publish(msg);
		}

		void OnIMUUpdate(sensors::SensorPtr sensor)
		{
			//	cast
			shared_ptr<sensors::ImuSensor> imu = dynamic_pointer_cast<sensors::ImuSensor> (sensor);

			//	read IMU
			VECTOR3 acc = imu->GAZEBO_IMU_SENSOR_LINEAR_ACCELERATION();

			//	store output
			output.imu_body.msg.linear_acceleration.x = acc[0];
			output.imu_body.msg.linear_acceleration.y = acc[1];
			output.imu_body.msg.linear_acceleration.z = acc[2];

			//	offer 50Hz
			offer_50Hz();
		}

		void OnContactUpdate(sensors::SensorPtr sensor)
		{
			//	cast
			shared_ptr<sensors::ContactSensor> contact = dynamic_pointer_cast<sensors::ContactSensor> (sensor);

			//	sense
			msgs::Contacts contacts = contact->Contacts();
			if (contacts.contact_size())
			{
				//	identify sensor
				int row = -1, col = -1;
				for (int r=0; r<ROBOT_ROW_COUNT; r++)
				{
					for (int c=0; c<ROBOT_COL_COUNT; c++)
					{
						if (sensor == sens.contact[r][c])
						{
							row = r;
							col = c;
							break;
						}
					}
					if (row != -1)
						break;
				}
				if (row == -1)
				{
					__COUT_W << "sensor not found" << endl;
				}
				else
				{
					//cout << row << ", " << col << ": " << contacts.contact_size() << endl;

					//	get first contact in array
					physics::Contact c;
					c = contacts.contact(0);
//					cout << "--------\nname: " << c.collision2->GetLink()->GetName() << endl;

					//	get associated link to collision2, which is the link of the whisker
					physics::LinkPtr link = c.collision2->GetLink();

					//	get its current pose in WORLD
					const WORLD_POSE& pose = link->GAZEBO_WORLD_POSE();
					//cout << "pose: " << pose << endl;

					//	get transform back to initial pose of this whisker (initial
					//	pose of all whiskers is [0 0 0 0 0 0], see initialisation)
					WORLD_POSE ipose = __POSE_GET_INVERSE(pose);

					//	convolve with transform back to canonical whisker pose
					ipose = ipose * state.wpose[row][col];

					//	collate total effect of all contacts on this whisker
					double x = 0.0;
					double y = 0.0;

					//	for each contact
					for (int i=0; i<contacts.contact_size(); i++)
					{
						//	get contact
						c = contacts.contact(i);

						//	for each contact element
						for (int j=0; j<c.count; j++)
						{
							//	get contact data
							VECTOR3 pos_W = c.positions[j];
							VECTOR3 norm_W = c.normals[j];
							double depth = c.depths[j];

							//cout << c.collision2->GetLink()->GetName() << " : " << pos << ", " << norm << ", " << depth << endl;

							//	transform contact data into canonical frame
                            WORLD_POSE pose_W(pos_W, QUATERNION());
							VECTOR3 pos = __POSE_POS(pose_W * ipose);
							VECTOR3 norm = __POSE_ROT(ipose).RotateVector(norm_W);

							//	report
							//cout << pos << "  @@@  " << norm << endl;

							//	scale normal by depth
							norm_W *= depth;

							//	adjust by whisker length
							double z = __VECTOR3_Z(pos_W) + ROBOT_WHISKER_Z_OFF;
							norm *= 1.0 / z;

							//	scale by gain
							norm *= ROBOT_WHISKER_GAIN;

							//	accumulate
							x += __VECTOR3_X(norm);
							y += __VECTOR3_Y(norm);
						}
					}

					//	get sim time
					//common::Time t = state.model->GetWorld()->GetSimTime();
					//cout << t << endl;

					//	store sensor output
					struct WhiskerXY* w = (struct WhiskerXY*) &output.bridge_u.msg.xy.data[0];
					for (int i=0; i<ROBOT_FS_SCALE; i++)
					{
						w->xy[i][row][col].x = x;
						w->xy[i][row][col].y = y;
					}

					//	store sensor output
					float* xy = (float*) &output.xy.msg.data[2 * (row*4+col)];
					xy[0] = x;
					xy[1] = y;
				}
			}

			//	offer 50Hz
			offer_50Hz();
		}

		void offer_50Hz()
		{
			/*
				This function is called by every 50Hz sensor. When
				all 50Hz sensors have updated, we can run a 50Hz
				control cycle. We then set sensors_updated to 0.
				Therefore, sensors_updated should always be 0 when
				a physics update runs, which we check to confirm
				synchrony.
			*/

			//	report
			//cout << "sensor " << state.sensors_updated << " @ " << get_sim_time() << endl;

			//	one more sensor updated
			state.sensors_updated++;

			//	if that's all of them
			if (state.sensors_updated == NUM_50HZ_SENSORS)
			{
				//	run the control cycle
				On50Hz();

				//	clear the count
				state.sensors_updated = 0;
			}
		}

		void ReadJointAngles()
		{
			//	this function reads in the current (measured) joint angles from
			//	all the robot joints, and stores them in the outgoing message

			//	neck
			int j = 0;
			for (int t=0; t<ROBOT_FS_SCALE; t++)
				for (int i=0; i<ROBOT_NECK_COUNT; i++)
					output.bridge_u.msg.neck.data[j++] = state.joints_neck[i].joint->GAZEBO_GET_JOINT_POS(0) + state.neck_offset[i];

			//	whiskers
			j = 0;
			for (int t=0; t<ROBOT_FS_SCALE; t++)
				for (int r=0; r<ROBOT_ROW_COUNT; r++)
					for (int c=0; c<ROBOT_COL_COUNT; c++)
						output.bridge_u.msg.theta.data[j++] = state.joints_whisker[r][c].joint->GAZEBO_GET_JOINT_POS(0);
		}

		void TestDriveNeck()
		{
			//	test drive
			static double t;

			double x = sin(t * 2.0 * M_PI * 0.2);
			x = 0.6 + x * 0.3;
			double y = sin(t * 2.0 * M_PI * 0.1);
			y = 0.0 + y * 0.78;
			for (int i=0; i<2; i++)
				input.neck.cmd[i] = x;
			input.neck.cmd[2] = y;

			//	test drive
			t += 0.02;
		}

		void TestDriveWhiskers()
		{
			//	test drive
			static double t;

			double z = sin(t * 2.0 * M_PI * 0.5);
			z = 0.0 + z * 0.78;
			for (int i=0; i<24; i++)
				input.theta.cmd[i] = z;

			//	test drive
			t += 0.02;
		}

		void On50Hz()
		{
			//	shutdown?
			if (!output.h_ros->ok())
				__ERROR("ROS was shutdown");

			//	test drive
			//TestDriveNeck();
			//TestDriveWhiskers();

/*
			//	if the simulation is paused, we sometimes continue to get
			//	these calls - I'm not clear why, or how Gazebo is configured
			//	to synchronize across different objects. in any case, we for
			//	now just detect the paused state and ignore the update
			if (state.model->GetWorld()->IsPaused())
			{
				//	simulation is paused, so do not step model or controllers
//				cout << "." << endl;
				return;
			}
*/

////////////////	OUTPUT

//	can do it this way, but now we read it direct from pose control linkage
#if 0
			//	read robot position (God's odometry)
			const WORLD_POSE& pose = state.link_body->GAZEBO_WORLD_POSE();
			math::Quaternion rot = pose.rot;
			output.pose.msg.x = pose.pos.x;
			output.pose.msg.y = pose.pos.y;
			output.pose.msg.theta = rot.GetYaw();
#endif

			//	read robot position (odometry from pose control linkage)
			output.pose.msg.x = state.joints_pose[0].joint->GAZEBO_GET_JOINT_POS(0);
			output.pose.msg.y = state.joints_pose[1].joint->GAZEBO_GET_JOINT_POS(0);
			output.pose.msg.theta = state.joints_pose[2].joint->GAZEBO_GET_JOINT_POS(0);

			//	read joint angles
			ReadJointAngles();

			//	publish
			output.bumper.pub.publish(output.bumper.msg);
			output.imu_body.pub.publish(output.imu_body.msg);
			output.pose.pub.publish(output.pose.msg);
			output.bridge_u.pub.publish(output.bridge_u.msg);
			output.xy.pub.publish(output.xy.msg);

			//	zero x/y so we can fill them again next time
			struct WhiskerXY* w = (struct WhiskerXY*) &output.bridge_u.msg.xy.data[0];
			memset(w, 0, sizeof(WhiskerXY));
			memset(&output.xy.msg.data[0], 0, 2 * ROBOT_WHISKER_COUNT * sizeof(float));

////////////////	INPUT

			//	spin
			ros::spinOnce();

//	NB: I don't like using the string "joint_desc.name" to select these, but
//	the API doesn't seem to define any alternative route...
#define __SETPOS(joint_desc, angle) \
	do { bool ret = state.controller->SetPositionTarget(joint_desc.name, angle); if (!ret) __COUT_W << "joint not found" << endl; } while(false)

			//	neck drive
			for (int i=0; i<ROBOT_NECK_COUNT; i++)
				__SETPOS(state.joints_neck[i], input.neck.cmd[i]);

			//	whisker drive
			for (int i=0; i<ROBOT_WHISKER_COUNT; i++)
				__SETPOS(state.joints_whisker[0][i], input.theta.cmd[i]);

//	NB: I don't like using the string "joint_desc.name" to select these, but
//	the API doesn't seem to define any alternative route...
#define __SETVEL(joint_desc, vel) \
	do { bool ret = state.controller->SetVelocityTarget(joint_desc.name, vel); if (!ret) __COUT_W << "joint not found" << endl; } while(false)

			//	rotate pose drive from robot frame (as delivered) into world frame (as actioned)
			double theta = output.pose.msg.theta;
			double cx = cos(theta);
			double sx = sin(theta);
			double vx = input.cmd_vel.vel[0];
			double vy = input.cmd_vel.vel[1];
			input.cmd_vel.vel[0] = cx * vx - sx * vy;
			input.cmd_vel.vel[1] = sx * vx + cx * vy;

			//	pose drive
			for (int i=0; i<3; i++)
			{
				__SETVEL(state.joints_pose[i], input.cmd_vel.vel[i]);
				input.cmd_vel.vel[i] = 0.0; // in case ROS inputs stop coming, let's not go to the icy wastes
			}
		}

	private:

		//	state
		struct State
		{
			State() :
				model(NULL),
				sensors_connected(0),
				sensors_updated(0)
			{
				neck_offset[0] = -M_PI / 2.0;
				neck_offset[1] = 0.0;
				neck_offset[2] = 0.0;
			}

			//	top-level objects
			physics::ModelPtr model;
			physics::JointControllerPtr controller;

			//	link objects
			physics::LinkPtr link_body;

			//	joint objects
			JointDesc joints_pose[3];
			JointDesc joints_neck[ROBOT_NECK_COUNT];
			JointDesc joints_whisker[ROBOT_ROW_COUNT][ROBOT_COL_COUNT];

			//	sensor connections
			event::ConnectionPtr connection[MAX_SENSOR_COUNT];
			uint32_t sensors_connected;
			uint32_t sensors_updated;

			//	zero on each joint means "as configured in the CAD"; we don't really
			//	have control over this (MJP does it) and it may not agree with the
			//	canonical zero points; we fix this, using an offset here. in principle,
			//	we could do the same with whiskers, but currently MJP == canonical.
			double neck_offset[ROBOT_NECK_COUNT];

			//	transforms from initial whisker poses to canonical pose
			WORLD_POSE wpose[ROBOT_ROW_COUNT][ROBOT_COL_COUNT];
		}
		state;

		//	input
		struct Input
		{
			Input()
			{
			}

			struct Neck
			{
				Neck()
				{
					cmd[0] = cmd[1] = M_PI / 4.0;
					cmd[2] = 0.0;
				}

				ros::Subscriber sub;
				double cmd[ROBOT_NECK_COUNT];
			}
			neck;

			struct Theta
			{
				Theta()
				{
					for (int i=0; i<ROBOT_WHISKER_COUNT; i++)
						cmd[i] = 0.0;
				}

				ros::Subscriber sub;
				double cmd[ROBOT_WHISKER_COUNT];
			}
			theta;

			struct CmdVel
			{
				CmdVel()
				{
					for (int i=0; i<3; i++)
						vel[i] = 0.0;
				}

				ros::Subscriber sub;
				double vel[3];
			}
			cmd_vel;
		}
		input;

		//	output
		struct Output
		{
			Output()
				:
				h_ros(NULL)
			{
			}

			ros::NodeHandle* h_ros;

			struct Bumper
			{
				Bumper()
				{

				}

				ros::Publisher pub;
				std_msgs::Bool msg;
			}
			bumper;

			struct IMU
			{
				IMU()
				{

				}

				ros::Publisher pub;
				sensor_msgs::Imu msg;
			}
			imu_body;

			struct Pose
			{
				Pose()
				{

				}

				ros::Publisher pub;
				geometry_msgs::Pose2D msg;
			}
			pose;

			struct Cam
			{
				Cam()
				{

				}

				image_transport::Publisher pub;
				sensor_msgs::Image msg;
			}
			cam[2];

			struct BridgeU
			{
				BridgeU()
				{
					msg.neck.data.resize(ROBOT_NECK_COUNT * ROBOT_FS_SCALE);
					msg.theta.data.resize(ROBOT_WHISKER_COUNT * ROBOT_FS_SCALE);
					msg.xy.data.resize(2 * ROBOT_WHISKER_COUNT * ROBOT_FS_SCALE);
					msg.physical.data = false;
				}

				ros::Publisher pub;
				whiskeye_msgs::bridge_u msg;
			}
			bridge_u;

			struct XY
			{
				XY()
				{
					msg.data.resize(2 * ROBOT_WHISKER_COUNT);
				}

				ros::Publisher pub;
				std_msgs::Float32MultiArray msg;
			}
			xy;
		}
		output;

		//	sensors
		struct Sensors
		{
			Sensors() :
				imu_body(NULL)
			{
			}

			sensors::SensorPtr imu_body;
			sensors::SensorPtr cam[2];
			sensors::SensorPtr contact[ROBOT_ROW_COUNT][ROBOT_COL_COUNT];
		}
		sens;

	};

	GZ_REGISTER_MODEL_PLUGIN(Whiskeye_ModelPlugin)
}




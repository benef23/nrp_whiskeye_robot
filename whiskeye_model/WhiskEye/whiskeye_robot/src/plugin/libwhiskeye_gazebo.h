
//	ROS niceties
#define ROS_SEND_QUEUE_SIZE 1
#define ROS_RECV_QUEUE_SIZE 1

//	robot/interface geometry
#define ROBOT_FS_SCALE 10
#define ROBOT_NECK_COUNT 3
#define ROBOT_ROW_COUNT 6
#define ROBOT_COL_COUNT 4
#define ROBOT_WHISKER_COUNT (ROBOT_ROW_COUNT * ROBOT_COL_COUNT)

////	VELOCITY CONTROL

//	P parameter for control of three pose joints (x, y, theta)
#define ROBOT_POSE_PID_P_0 250
#define ROBOT_POSE_PID_P_1 250
#define ROBOT_POSE_PID_P_2 100

//	D parameter for control of all three pose joints (fractional)
#define ROBOT_POSE_PID_D 0.0

////	POSITION CONTROL

//	P parameter for control of three neck joints (proximal to distal)
#define ROBOT_NECK_PID_P_0 25
#define ROBOT_NECK_PID_P_1 15
#define ROBOT_NECK_PID_P_2 10

//	D parameter for control of all three neck joints (fractional)
#define ROBOT_NECK_PID_D 0.1

//	P/D parameters for control of whiskers
#define ROBOT_WHISKER_PID_P 0.2
#define ROBOT_WHISKER_PID_D 0.02

////	WHISKER SENSORS

//	sensor parameters
#define ROBOT_WHISKER_Z_OFF 0.1
#define ROBOT_WHISKER_GAIN 0.05




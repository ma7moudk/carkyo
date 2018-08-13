

/*
 * =====================================================================================
 *
 *       Filename:  mpu9150_node.cpp
 *
 *    Description: ROS package that launches a node and publishes the Invensense MPU-9150 to a Topic  
 *
 *        Version:  1.0
 *        Created:  27/07/13 15:06:50
 *       Revision:  none
 *
 *         Author:  Víctor Mayoral Vilches <v.mayoralv@gmail.com>
 *
 * =====================================================================================
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
//add ma7moudk
#include "sensor_msgs/Imu.h"

#include <sstream>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <getopt.h>
#include <errno.h>
#include <tf/transform_datatypes.h>
// Needed when mixing C and C++ code/libraries
#ifdef __cplusplus
    extern "C" {
#endif
        #include "mpu9150.h"
        #include "linux_glue.h"
        #include "local_defaults.h"

#ifdef __cplusplus
    }
#endif


int set_cal(int mag, char *cal_file);
void read_loop(unsigned int sample_rate);
void print_fused_euler_angles(mpudata_t *mpu);
void print_fused_quaternion(mpudata_t *mpu);
void print_calibrated_accel(mpudata_t *mpu);
void print_calibrated_mag(mpudata_t *mpu);
void register_sig_handler();
void sigint_handler(int sig);

int done;
double quatX,quatY,quatZ,quatW,convRoll,convPitch,convYaw;
void usage(char *argv_0)
{
    printf("\nUsage: %s [options]\n", argv_0);
    printf("  -b <i2c-bus>          The I2C bus number where the IMU is. The default is 1 to use /dev/i2c-1.\n");
    printf("  -s <sample-rate>      The IMU sample rate in Hz. Range 2-50, default 10.\n");
    printf("  -y <yaw-mix-factor>   Effect of mag yaw on fused yaw data.\n");
    printf("                           0 = gyro only\n");
    printf("                           1 = mag only\n");
    printf("                           > 1 scaled mag adjustment of gyro data\n");
    printf("                           The default is 4.\n");
    printf("  -a <accelcal file>    Path to accelerometer calibration file. Default is ./accelcal.txt\n");
    printf("  -m <magcal file>      Path to mag calibration file. Default is ./magcal.txt\n");
    printf("  -v                    Verbose messages\n");
    printf("  -h                    Show this help\n");

    printf("\nExample: %s -b3 -s20 -y10\n\n", argv_0);

    exit(1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpu9150_node_pi");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("imu_euler_pi", 1000);
  ros::Publisher imu_pub     = n.advertise<sensor_msgs::Imu>("imu_data_pi", 1000);
  ros::Publisher imu_enu_pub = n.advertise<sensor_msgs::Imu>("imu_enu_pi", 2);

  ros::Rate loop_rate(10);

  /* Init the sensor the values are hardcoded at the local_defaults.h file */
    int opt, len;
//////////////ma7moudk add these three lines ////////////
//	int DEFAULT_I2C_BUS;
//	ros::NodeHandle private_node_handle_("~");
//	private_node_handle_.param("DEFAULT_I2C_BUS" , DEFAULT_I2C_BUS , int(2));

	int i2c_bus = DEFAULT_I2C_BUS;
	int sample_rate = DEFAULT_SAMPLE_RATE_HZ;
	int yaw_mix_factor = DEFAULT_YAW_MIX_FACTOR;
	int verbose = 0;
	char *mag_cal_file = NULL;
	char *accel_cal_file = NULL;
	unsigned long loop_delay;
	mpudata_t mpu;

    // receive the parameters and process them
    while ((opt = getopt(argc, argv, "b:s:y:a:m:vh")) != -1) {
        switch (opt) {
        case 'b':
            i2c_bus = strtoul(optarg, NULL, 0);

            if (errno == EINVAL)
                usage(argv[0]);

            if (i2c_bus < MIN_I2C_BUS || i2c_bus > MAX_I2C_BUS)
                usage(argv[0]);

            break;

        case 's':
            sample_rate = strtoul(optarg, NULL, 0);

            if (errno == EINVAL){
                printf("EINVAL\n");
                usage(argv[0]);
            }

            if (sample_rate < MIN_SAMPLE_RATE || sample_rate > MAX_SAMPLE_RATE){
                printf("sample rate problem\n");
                usage(argv[0]);
            }

            break;

        case 'y':
            yaw_mix_factor = strtoul(optarg, NULL, 0);

            if (errno == EINVAL)
                usage(argv[0]);

            if (yaw_mix_factor < 0 || yaw_mix_factor > 100)
                usage(argv[0]);

            break;

        case 'a':
            len = 1 + strlen(optarg);

            accel_cal_file = (char *)malloc(len);

            if (!accel_cal_file) {
                perror("malloc");
                exit(1);
            }

            strcpy(accel_cal_file, optarg);
            break;

        case 'm':
            len = 1 + strlen(optarg);

            mag_cal_file = (char *)malloc(len);

            if (!mag_cal_file) {
                perror("malloc");
                exit(1);
            }

            strcpy(mag_cal_file, optarg);
            break;

        case 'v':
            verbose = 1;
            break;

        case 'h':
        default:
            usage(argv[0]);
            break;
        }
    }

    // Initialize the MPU-9150
	register_sig_handler();
	mpu9150_set_debug(verbose);
	if (mpu9150_init(i2c_bus, sample_rate, yaw_mix_factor))
		exit(1);
	set_cal(0, accel_cal_file);
	set_cal(1, mag_cal_file);
	if (accel_cal_file)
		free(accel_cal_file);
	if (mag_cal_file)
		free(mag_cal_file);
	memset(&mpu, 0, sizeof(mpudata_t));
	if (sample_rate == 0)
		return -1;

    // ROS loop config
	loop_delay = (1000 / sample_rate) - 2;
	printf("\nEntering MPU read loop (ctrl-c to exit)\n\n");
	linux_delay_ms(loop_delay);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;
    //add ma7moudk
    sensor_msgs::Imu imu_msg;
    sensor_msgs::Imu imu_enu;

    std::stringstream ss;

	if (mpu9150_read(&mpu) == 0) {
//		print_fused_euler_angles(&mpu);
	    //ss << "\rX: %0.0f Y: %0.0f Z: %0.0f        ",
	    ss << "\rX: " << mpu.tempEuler[VEC3_X] * RAD_TO_DEGREE <<
          " Y: " << mpu.tempEuler[VEC3_Y] * RAD_TO_DEGREE <<
			" Z: " << mpu.tempEuler[VEC3_Z] * RAD_TO_DEGREE ;

    //   printf("and");
	// print_fused_quaternions(&mpu);
//	print_calibrated_accel(&mpu);
     //   printf("and");
  //      print_calibrated_mag(&mpu);
    //    printf("and");
       msg.data = ss.str();
//add ma7moudk
        imu_msg.header.frame_id= "base_link";
        imu_msg.header.stamp= ros::Time::now();
        imu_msg.orientation.w=mpu.fusedQuat[QUAT_W];
        imu_msg.orientation.x=mpu.fusedQuat[QUAT_X];
        imu_msg.orientation.y=mpu.fusedQuat[QUAT_Y];
        imu_msg.orientation.z=mpu.fusedQuat[QUAT_Z];
        imu_msg.angular_velocity.x=mpu.rawGyro[VEC3_X];
        imu_msg.angular_velocity.y=mpu.rawGyro[VEC3_Y];
        imu_msg.angular_velocity.z=mpu.rawGyro[VEC3_Z];

        imu_enu.header.frame_id= "imu_link";
	imu_enu.header.stamp= ros::Time::now();
////////////////////////////////////ma7moudk//////////////////////////////////////////
//try to convert in ROS .. no need for it .. now conversion happens in mpu9150.c
//	quatW=mpu.fusedQuat[QUAT_W];
//	quatX=mpu.fusedQuat[QUAT_Y];
//	quatY=mpu.fusedQuat[QUAT_X];
//	quatZ=-mpu.fusedQuat[QUAT_Z];
//	tf::Quaternion q(quatX,quatY,quatZ,quatW);
//	tf::Matrix3x3 m(q);
//	m.getRPY(convRoll,convPitch,convYaw);
//	ROS_INFO("enu yaw before add 90: %9.4f\n" ,convYaw); 
//	convYaw+=(M_PI/2.0);
//	if (convYaw>M_PI){ 
//		convYaw-=(2*M_PI);
//}

//	ROS_INFO("enu yaw after add 90: %9.4f\n" ,convYaw); 

//	tf::Quaternion qq = tf::createQuaternionFromRPY (convRoll, convPitch, convYaw);
////////////////////////////////////////////////////////////////////////////////////////
        imu_enu.orientation.w=mpu.fusedQuat[QUAT_W];
        imu_enu.orientation.x=mpu.fusedQuat[QUAT_X];
        imu_enu.orientation.y=mpu.fusedQuat[QUAT_Y];
        imu_enu.orientation.z=mpu.fusedQuat[QUAT_Z];


	imu_enu.angular_velocity.x=mpu.rawGyro[VEC3_X]*3.142857/(16.34*180.0);
	imu_enu.angular_velocity.y=mpu.rawGyro[VEC3_Y]*3.142857/(16.34*180.0);
	imu_enu.angular_velocity.z=mpu.rawGyro[VEC3_Z]*3.142857/(16.34*180.0);

//	imu_msg.angular_velocity.z=mpu.calibratedAccel[VEC3_Z]*9.80665/16384.0	;

	imu_msg.linear_acceleration.x=mpu.rawAccel[VEC3_X]*9.80665/16384.0;
	imu_enu.linear_acceleration.x=mpu.rawAccel[VEC3_X]*9.80665/16384.0;
	imu_msg.linear_acceleration.y=mpu.rawAccel[VEC3_Y]*9.80665/16384.0;
	imu_enu.linear_acceleration.y=mpu.rawAccel[VEC3_Y]*9.80665/16384.0;
	imu_msg.linear_acceleration.z=mpu.rawAccel[VEC3_Z]*9.80665/16384.0;
	imu_enu.linear_acceleration.z=mpu.rawAccel[VEC3_Z]*9.80665/16384.0;

       ROS_INFO("%s", msg.data.c_str());
	}
//	linux_delay_ms(loop_delay);
    chatter_pub.publish(msg);
//add ma7moudk
    imu_pub.publish(imu_msg);
    imu_enu_pub.publish(imu_enu);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}

void print_fused_euler_angles(mpudata_t *mpu)
{
	printf("\reuler X: %0.0f Y: %0.0f Z: %0.0f        ",
			mpu->fusedEuler[VEC3_X] * RAD_TO_DEGREE, 
			mpu->fusedEuler[VEC3_Y] * RAD_TO_DEGREE, 
			mpu->fusedEuler[VEC3_Z] * RAD_TO_DEGREE);

	fflush(stdout);
}

void print_fused_quaternions(mpudata_t *mpu)
{
	printf("\rW: %0.2f X: %0.2f Y: %0.2f Z: %0.2f        ",
			mpu->fusedQuat[QUAT_W],
			mpu->fusedQuat[QUAT_X],
			mpu->fusedQuat[QUAT_Y],
			mpu->fusedQuat[QUAT_Z]);

	fflush(stdout);
}

void print_calibrated_accel(mpudata_t *mpu)
{
	printf("\racc X: %05d Y: %05d Z: %05d        ",
			mpu->calibratedAccel[VEC3_X], 
			mpu->calibratedAccel[VEC3_Y], 
			mpu->calibratedAccel[VEC3_Z]);

	fflush(stdout);
}

void print_calibrated_mag(mpudata_t *mpu)
{
	printf("\r mag X: %03d Y: %03d Z: %03d        ",
			mpu->calibratedMag[VEC3_X], 
			mpu->calibratedMag[VEC3_Y], 
			mpu->calibratedMag[VEC3_Z]);

	fflush(stdout);
}

int set_cal(int mag, char *cal_file)
{
	int i;
	FILE *f;
	char buff[32];
	long val[6];
	caldata_t cal;

	if (cal_file) {
		f = fopen(cal_file, "r");

		if (!f) {
			perror("open(<cal-file>)");
			return -1;
		}
	}
	else {
		if (mag) {
			f = fopen("catkin_workspace/src/bb_mpu9150/src/linux-mpu9150/magcal.txt", "r");

			if (!f) {
				printf("Default magcal.txt not found\n");
				return 0;
			}
		}
		else {
			f = fopen("catkin_workspace/src/bb_mpu9150/src/linux-mpu9150/accelcal.txt", "r");

			if (!f) {
				printf("Default accelcal.txt not found\n");
				return 0;
			}
		}
	}

	memset(buff, 0, sizeof(buff));

	for (i = 0; i < 6; i++) {
		if (!fgets(buff, 20, f)) {
			printf("Not enough lines in calibration file\n");
			break;
		}

		val[i] = atoi(buff);

		if (val[i] == 0) {
			printf("Invalid cal value: %s\n", buff);
			break;
		}
	}

	fclose(f);

	if (i != 6) 
		return -1;

	cal.offset[0] = (short)((val[0] + val[1]) / 2);
	cal.offset[1] = (short)((val[2] + val[3]) / 2);
	cal.offset[2] = (short)((val[4] + val[5]) / 2);

	cal.range[0] = (short)(val[1] - cal.offset[0]);
	cal.range[1] = (short)(val[3] - cal.offset[1]);
	cal.range[2] = (short)(val[5] - cal.offset[2]);

	if (mag) 
		mpu9150_set_mag_cal(&cal);
	else 
		mpu9150_set_accel_cal(&cal);

	return 0;
}

void register_sig_handler()
{
	struct sigaction sia;

	bzero(&sia, sizeof sia);
	sia.sa_handler = sigint_handler;

	if (sigaction(SIGINT, &sia, NULL) < 0) {
		perror("sigaction(SIGINT)");
		exit(1);
	} 
}

void sigint_handler(int sig)
{
	done = 1;
}
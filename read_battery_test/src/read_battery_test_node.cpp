#include "ros/ros.h"
#include <std_msgs/String.h>
#include <read_battery_test/battery_data.h>
// C library headers
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <sstream>
#include <stdint.h>
//Linux headers
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <bitset>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>

using namespace std;

/* Function definition */
void my_handler(int s);
// void msg_callback(const read_battery_test::battery_data::ConstPtr& msg); // Call back function for test

/* Global variables */
int serial_port = open("/dev/ttyUSB0", O_RDWR);

int main(int argc, char **argv)
{
	//////////////////////////////////////////////////  
	// To handle CTRL + C Interrupt
	//////////////////////////////////////////////////
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
	//////////////////////////////////////////////////

	/* シリアルポートのオープン */
	//int serial_port = open("/dev/ttyUSB0", O_RDWR);
	if(serial_port < 0) {
		cout << "ERROR" << endl;
		cout << "cannot open the port : " << serial_port << endl;
		return 1;
	}
	//cout << "SERIAL PORT : OPEN" << endl;

	/* シリアルポートの通信設定 ( ありものをコピー+ちょっと変えたらとりあえず動いた ) */
	struct termios tty;
	memset(&tty, 0, sizeof tty);
	if(tcgetattr(serial_port, &tty) != 0) {
		cout << "ERROR" << endl;
		return 1;
	}
	tty.c_cflag |= PARENB;  // Set parity bit, enabling parity
	tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
	tty.c_cflag |= CS7; // 8 bits per byte (most common)
	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    /* ここのパラメータが小さいとreadしたときに、全バイトが読み出せなかった。
       もう少し調整できるかも */
	tty.c_cc[VTIME] = 100;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
	tty.c_cc[VMIN] = 34;
    
	cfsetispeed(&tty, B9600);
	cfsetospeed(&tty, B9600);
	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
		printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
		cout << "ERROR" << endl;
		return 1;
	}
	//cout << "SERIAL PORT : CONFIGURATED" << endl;
	
	uint8_t write_frame[32];
	write_frame[0] = 0xDD; // Start Bit
	write_frame[1] = 0xA5; // Status Bit ( write )
	write_frame[2] = 0x03; // Command Code (03=basic information and status )
	write_frame[3] = 0x00; // Length
	write_frame[4] = 0xFF; // Calibration ( check sum )
	write_frame[5] = 0xFD; // Calibration ( check sum )
	write_frame[6] = 0x77; // Stop Bit

	// Setting for publish & subscribing
	ros::init(argc, argv, "read_battery_test_node");
	ros::NodeHandle pub, sub;
	ros::Publisher test_pub = pub.advertise<read_battery_test::battery_data>("battery_data", 100);
	//ros::Subscriber test_sub = sub.subscribe("battery_data", 100, msg_callback); // Subscriber for test
	ros::Rate loop_rate(1);

	read_battery_test::battery_data msg;

	// Configuration for timeout
	fd_set fdz;
	struct timeval tv = {10}; // timeout : 10 sec
	FD_ZERO(&fdz); // initialize 'fdz'
	FD_SET(serial_port, &fdz); // add descriptor 'serial_port' to 'fdz'

	//cout << "LOOP START" << endl;

	while(ros::ok())
	{
		// Serial communication
		int w_st = write(serial_port, &(write_frame[0]), 7);
		//fprintf(stderr, "DONE : Sent %2d data\n", w_st);

		uint8_t read_frame[128] = {0x00};
		//fprintf(stderr, "DONE : Initialize 'read_frame'\n");
		int n = select(serial_port+1, &fdz, 0, 0, &tv);
		if (n < 1){ // If either error about the descriptor or timeout of read() occurs, connection will be closed.
			fprintf(stderr, "Timeout : read()\n");
			close(serial_port);
			return 1;
		}
		int r_st = read(serial_port, &(read_frame), 128);
		if (r_st > 0) fprintf(stderr, "DONE : Received %2d data\n", r_st);
		else{
			fprintf(stderr, "FAILED : Received data\n");
			continue;
		}

		// Data processing for publish
		msg.stamp = ros::Time::now();
		// int32 data_length
		msg.data_length = (int) (((uint16_t) read_frame[2] << 8) + (uint16_t) read_frame[3]);
		// float64 total_voltage
		msg.total_voltage = (double) (((uint16_t) read_frame[4] << 8) + (uint16_t) read_frame[5]) / 100.0;
		// float64 current
		msg.current = (double) (((uint16_t) read_frame[6] << 8) + (uint16_t) read_frame[7]) / 100.0;
		// float64 residual_capacity
		msg.residual_capacity = (double) (((uint16_t) read_frame[8] << 8) + (uint16_t) read_frame[9]) / 100.0;
		// float64 nominal_capacity
		msg.nominal_capacity = (double) (((uint16_t) read_frame[10] << 8) + (uint16_t) read_frame[11]) / 100.0;
		// int32 cycle_life
		msg.cycle_life = (int) (((uint16_t) read_frame[12] << 8) + (uint16_t) read_frame[13]);
		// int32 product_date_year
		// int32 product_date_month
		// int32 product_date_day
		uint16_t product_date = ((uint16_t) read_frame[14] << 8) + (uint16_t) read_frame[15];
		msg.product_date_year = (int) (product_date >> 9) + 2000;
		msg.product_date_month = (int) ((product_date & 0x01e0) >> 5);
		msg.product_date_day = (int) (product_date & 0x001f);
		// uint16 balance_status
		msg.balance_status = ((uint16_t) read_frame[16] << 8) + (uint16_t) read_frame[17];
		// uint16 balance_status_high
		msg.balance_status_high = ((uint16_t) read_frame[18] << 8) + (uint16_t) read_frame[19];
		// uint16 protection_status
		msg.protection_status = ((uint16_t) read_frame[20] << 8) + (uint16_t) read_frame[21];
		// float64 version
		msg.version = (double) (read_frame[22] >> 4) + (double) (read_frame[22] & 0x0f) * 0.1;
		// int32 rsoc
		msg.rsoc = (int) read_frame[23];
		// uint8 control_status
		msg.control_status = read_frame[24];
		// int32 num_cell
		msg.num_cell = (int) read_frame[25];
		// int32 num_ntc
		msg.num_ntc = (int) read_frame[26];
		// float64 temp1
		msg.temp1 = (double) (((uint16_t) read_frame[27] << 8) + (uint16_t) read_frame[28] - 0x0aab) / 10.0;
		// float64 temp2
		msg.temp2 = (double) (((uint16_t) read_frame[29] << 8) + (uint16_t) read_frame[30] - 0x0aab) / 10.0;
		// uint16 check_sum
		msg.check_sum = ((uint16_t) read_frame[31] << 8) + (uint16_t) read_frame[32];

		test_pub.publish(msg);
		//ros::spinOnce();
		loop_rate.sleep();
		sigaction(SIGINT, &sigIntHandler, NULL);
	}

	return 0;
}

void my_handler(int s){
           close(serial_port);
           printf("\n\nCaught signal %d\n Port closed.\n",s);
           exit(1); 
}

/* // Call back function for test
void msg_callback(const read_battery_test::battery_data::ConstPtr& msg){
	ROS_INFO(" ");
	ROS_INFO("***********************");
	ROS_INFO("Data length         : %d", msg->data_length);
	ROS_INFO("Total voltage       : %.2lf V", msg->total_voltage);
	ROS_INFO("Current             : %.2lf A", msg->current);
	ROS_INFO("Residual capacity   : %.2lf Ah", msg->residual_capacity);
	ROS_INFO("Nominal capacity    : %.2lf Ah", msg->nominal_capacity);
	ROS_INFO("Cycle life          : %d", msg->cycle_life);
	ROS_INFO("Product date        : %d/%d/%d", msg->product_date_year, msg->product_date_month, msg->product_date_day);
	ROS_INFO("Balance status      : 0x%04u", msg->balance_status);
	ROS_INFO("Balance status high : 0x%04u", msg->balance_status_high);
	ROS_INFO("Protection status   : %s", (msg->protection_status == 0U)? "OK" : "Abnormal");
	if (msg->protection_status & 0x0001 == 0x0001) ROS_INFO("  > Cell Block Over-Vol");
	if (msg->protection_status & 0x0002 == 0x0002) ROS_INFO("  > Cell Block Under-Vol");
	if (msg->protection_status & 0x0004 == 0x0004) ROS_INFO("  > Battery Over-Vol");
	if (msg->protection_status & 0x0008 == 0x0008) ROS_INFO("  > Battery Under-Vol");
	if (msg->protection_status & 0x0010 == 0x0010) ROS_INFO("  > Charging Over-temp");
	if (msg->protection_status & 0x0020 == 0x0020) ROS_INFO("  > Charging Low-temp");
	if (msg->protection_status & 0x0040 == 0x0040) ROS_INFO("  > Discharging Over-temp");
	if (msg->protection_status & 0x0080 == 0x0080) ROS_INFO("  > Discharging Low-temp");
	if (msg->protection_status & 0x0100 == 0x0100) ROS_INFO("  > Charging Over-current");
	if (msg->protection_status & 0x0200 == 0x0200) ROS_INFO("  > Discharging Over-current");
	if (msg->protection_status & 0x0400 == 0x0400) ROS_INFO("  > Short Circuit");
	if (msg->protection_status & 0x0800 == 0x0800) ROS_INFO("  > Fore-end IC Erro");
	if (msg->protection_status & 0x1000 == 0x1000) ROS_INFO("  > MOS Software Lock-in");
	ROS_INFO("Version             : %.1lf", msg->version);
	ROS_INFO("RSOC                : %d %%", msg->rsoc);
	ROS_INFO("Control status      : ");
	ROS_INFO("  > Charging MOS    : %s", (msg->control_status & 0x01 == 0x01)? "ON" : "OFF");
	ROS_INFO("  > Discharging MOS : %s", (msg->control_status & 0x02 == 0x02)? "ON" : "OFF");
	ROS_INFO("Num of Cells        : %d", msg->num_cell);
	ROS_INFO("Num of NTC          : %d", msg->num_ntc);
	if (msg->num_ntc > 0){
		ROS_INFO("NTC1                : %.1lf degree", msg->temp1);
		if (msg->num_ntc > 1){
			ROS_INFO("NTC2                : %.1lf degree", msg->temp2);
		}
	}
	ROS_INFO("Check sum            : 0x%04u", msg->check_sum);
	ROS_INFO(" ");
}
*/
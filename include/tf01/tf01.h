#ifndef _TF01_H_ 
#define _TF01_H_ 

#include "ros/ros.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#define BAUDRATE 	115200
#define MAXSIZE 	50
#define BUF_SIZE 	9

class TF01
{
public:
	TF01(ros::NodeHandle n_private);
	~TF01();

	float distance;
	float amplitude; 
	bool data_valid;
	int frequency;

	void read_data();

private:
	//serial
	int ret;
	int serial_fd;
	unsigned char read_buf[BUF_SIZE];
	unsigned char ring_buf[MAXSIZE];
	unsigned char data_buf[BUF_SIZE];
	int read_addr;
	int write_addr;

	unsigned int crc_data;
	unsigned int crc_recieved;

	bool read_valid;
	bool crc_valid;

	//parameter
	std::string device;

	int set_serial(int fd, int nSpeed, int nBits, char nEvent, int nStop) ;
	void serial_init();
	void get_param(ros::NodeHandle n_private);
	int next_data_handle(int addr);
	int next_data_handle(int addr,int count);
	void write_data(char data);
	unsigned int crc32gen(unsigned int data[], unsigned int size);
	
};


#endif 
#include "tf01/tf01.h"

TF01::TF01(ros::NodeHandle n_private)
{
	//serial
	read_addr = 0;
	write_addr = 0;

	read_valid = false;
	crc_valid = false;

	get_param(n_private);
	serial_init();

}

TF01::~TF01()
{
	close(serial_fd);
}

int TF01::set_serial(int fd,int nSpeed, int nBits, char nEvent, int nStop)  
{  
	struct termios newtio,oldtio;  
	if( tcgetattr( fd,&oldtio)  !=  0) 
	{   
		ROS_INFO("SetupSerial 1");  
		return -1;  
	}  
	bzero( &newtio, sizeof( newtio ) );  
	newtio.c_cflag  |=  CLOCAL | CREAD;  
	newtio.c_cflag &= ~CSIZE;  

	switch( nBits )  
	{  
		case 7:  
			newtio.c_cflag |= CS7;  
			break;  
		case 8:  
			newtio.c_cflag |= CS8;  
			break;  
	}

	switch( nEvent )  
	{  
		case 'O':  
			newtio.c_cflag |= PARENB;  
			newtio.c_cflag |= PARODD;  
			newtio.c_iflag |= (INPCK | ISTRIP);  
			break;  
		case 'E':   
			newtio.c_iflag |= (INPCK | ISTRIP);  
			newtio.c_cflag |= PARENB;  
			newtio.c_cflag &= ~PARODD;  
			break;
		case 'N':    																									
			newtio.c_cflag &= ~PARENB;  
			break;  
	}  

	switch( nSpeed )  
	{  
		case 2400:  
			cfsetispeed(&newtio, B2400);  
			cfsetospeed(&newtio, B2400);  
			break;  
		case 4800:  
			cfsetispeed(&newtio, B4800);  
			cfsetospeed(&newtio, B4800);  
			break;
		case 9600:  
			cfsetispeed(&newtio, B9600);  
			cfsetospeed(&newtio, B9600);  
			break;  
		case 19200:  
			cfsetispeed(&newtio, B19200);  
			cfsetospeed(&newtio, B19200);  
			break; 
		case 57600:  
			cfsetispeed(&newtio, B57600);  
			cfsetospeed(&newtio, B57600);  
			break; 
		case 115200:  
			cfsetispeed(&newtio, B115200);  
			cfsetospeed(&newtio, B115200);  
			break;  
		case 460800:  
			cfsetispeed(&newtio, B460800);  
			cfsetospeed(&newtio, B460800);  
			break; 
		case 921600:  
			cfsetispeed(&newtio, B921600);  
			cfsetospeed(&newtio, B921600);  
			break;   
		default:  
			cfsetispeed(&newtio, B9600);  
			cfsetospeed(&newtio, B9600);  
			break;  
	}  

	if( nStop == 1 )  
	{
		newtio.c_cflag &=  ~CSTOPB;  
	}
	else if ( nStop == 2 )
	{  
		newtio.c_cflag |=  CSTOPB;  
	} 

	newtio.c_cc[VTIME]  = 10;
	newtio.c_cc[VMIN] = 9;
	tcflush(fd,TCIFLUSH);  
	if((tcsetattr(fd,TCSANOW,&newtio))!=0)  
	{  
		ROS_INFO("com set error!");  
		return -1;  
	}  
	return 0;  
} 

void TF01::serial_init()
{
	//serial_fd = open(DEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);
	serial_fd = open(device.c_str(), O_RDWR | O_NOCTTY);
	if (serial_fd == -1)
	{
		ROS_INFO("Open Error!");  
		exit(1);    
	}  
	ret = set_serial(serial_fd, BAUDRATE, 8, 'N', 1);
	if (ret == -1)  
	{
		ROS_INFO("Set Serial Error!");  
		exit(1);  
	}
}

int TF01::next_data_handle(int addr) {
	return (addr + 1) == MAXSIZE ? 0 : (addr + 1);
}

int TF01::next_data_handle(int addr , int count)     
{     
	int a;
	a = addr;
	for(int i = 0; i < count ; i++)
	{ 
			a = ( (a + 1)  == MAXSIZE ?  0 : ( a + 1 ) ) ;   
	}
	return a;  
}

void TF01::write_data(char data)
{
	*(ring_buf+write_addr) = data;
	write_addr = next_data_handle(write_addr);
}

void TF01::read_data()
{
	memset(read_buf, 0, BUF_SIZE);
	ret = read(serial_fd, read_buf, BUF_SIZE);
	if( ret <= 0 )
	{
		ROS_INFO("read err: %d\n", ret);

	}else
	{
		for(int i = 0 ; i < BUF_SIZE ; i++)
		{
			write_data(read_buf[i]);
		}

		for(int i = 0 ; i < MAXSIZE ; i++)
		{
			if((ring_buf[read_addr] == 0x59) && (ring_buf[next_data_handle(read_addr)] == 0x59))  
			{
				for(int j = 0 ; j < 9 ; j++)
				{
					data_buf[j] = ring_buf[read_addr] ;
					read_addr = next_data_handle(read_addr) ;
				}
				read_valid = true;
				break;
			}else
			{
				read_addr = next_data_handle(read_addr) ;
			}
		}

		if(read_valid)
		{
			distance = ((short)data_buf[3]<<8 | (short)data_buf[2]) ;
			amplitude = ((short)data_buf[5]<<8 | (short)data_buf[4]) ;
		}

		//crc_data = crc32gen((unsigned int*)data_buf, 5);
		//crc_recieved = ((unsigned int)data_buf[23]<<24) | ((unsigned int)data_buf[22]<<16) | ((unsigned int)data_buf[21]<<8) | ((unsigned int)data_buf[20]) ;

		//if(crc_data == crc_recieved)
		// {
		// 	crc_valid = true;
		// }

		//if(read_valid && crc_valid)
		if(read_valid)
		{
			data_valid = true;
			read_valid = false;
		}
	}
}

unsigned int TF01::crc32gen(unsigned int data[], unsigned int size)
{
	unsigned int temp, crc = 0xFFFFFFFF;
	for(int i = 0; i < size; i++)
	{
		temp = data[i];
		for(int j = 0; j < 32; j++)
		{
			if((crc ^ temp) & 0x80000000)
			{
				crc = 0x04C11DB7 ^ (crc << 1);
			}else
			{
				crc <<= 1;
			}

			temp <<= 1;
		}
	}
	return crc;
}

void TF01::get_param(ros::NodeHandle n_private)
{
	if (n_private.getParam("device", device))
	{
		ROS_INFO("Device set to: %s", device.c_str());
	}
	else
	{
		device = "/dev/ttyUSB0";  
		ROS_INFO("Using the default USB device: /dev/ttyUSB0");
	}

}

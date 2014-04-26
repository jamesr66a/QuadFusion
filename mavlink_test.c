#include <mavlink/common/mavlink.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <signal.h>
#include <stdlib.h>

int fd;

void sig_handler(int signal){
    close(fd);
    exit(0);
}

int
set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                //error_message ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // ignore break signal
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                //error_message ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

int main()
{
	char buf[100];
    mavlink_message_t msg;
    mavlink_status_t status;
	
    signal(SIGINT, sig_handler);    

	fd = open("/dev/ttyO0", O_RDWR | O_SYNC | O_NOCTTY);
    set_interface_attribs(fd, B115200, 0);
/*	while (1)
	{
		do {
			read(fd, &diviner, 1);
			//printf("%i", (int)diviner);
		} while (diviner != 0xFE);
		printf("Packet start\n");
		read(fd, &diviner, 1);
		printf("payload length %i\n", diviner);
		read(fd, &diviner, 1);
		printf("packet sequence %i\n", diviner);
		read(fd, &diviner, 1);
		printf("system id %i\n", diviner);
		read(fd, &diviner, 1);
		printf("component id %i\n", diviner);
		read(fd, &diviner, 1);
		printf("message id %i\n", diviner);
		printf("--------------\n");
	}*/
	
    int x = 0;
    int y = 0;    
    int16_t flow_x = 0;
    int16_t flow_y = 0;
    uint8_t history = 0;
    while (1)
    {
        read(fd, buf, 1);

        if (mavlink_parse_char(0, buf[0], &msg, &status))
        {
            //printf("packet received\n");
            //printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
        
	    if (msg.msgid == 100) {
	        flow_x = 
(flow_x + mavlink_msg_optical_flow_get_flow_x(&msg))/2;
		flow_y = 
(flow_y + mavlink_msg_optical_flow_get_flow_y(&msg))/2;
		
		history <<= 1;
		history++;

//		printf("%i\n", history);
		if (history == 0x0F) {
	    	    x += flow_x;
		    y += flow_y;
		    printf("%i, %i\n", x, y);
	            
		    flow_x = flow_y = history = 0;
		}
            }
	}
    
    }
}

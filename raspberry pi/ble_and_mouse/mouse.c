#include <stdio.h>
#include <stdint.h>
#include <error.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>
#include <limits.h>


#include "fcntl.h"
#include "string.h"


#include <stdlib.h>
#include <fcntl.h>
#include <getopt.h>
#include <sys/param.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <ctype.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <arpa/inet.h>


#define subs_verticle_cursor_event   1
#define subs_horizon_cursor_event  2
#define subs_mouse_button_event   4

#define Verticle   0
#define Horizon  1
#define Button    2



#define mouse_button_event  0x01
#define mouse_cursor_event  0x05

#define is_verticle_cursor(c)               (c & 0x80)
#define is_cursor_up_or_right(c)        (c & 0x40)
#define cursor_level(c)                         (c & 0x3F)

#define button_left       0x00
#define button_right    0x01
#define button_both    0x02
#define button_long     0x03
#define button_end      0x04


#define mouse_button_left_click                   1
#define mouse_button_roll_push                  2
#define mouse_button_right_click                3
#define mouse_button_left_push                  4
#define mouse_button_right_push                5
#define mouse_button_left_end_push          6
#define mouse_button_roll_end_push          7
#define mouse_button_right_end_push        8
#define mouse_button_roll_up                       9
#define mouse_button_roll_down                  10





typedef enum
{
    WAIT_START = 0,
    WAIT_LONG_OR_END,
    WAIT_END,
}Button_State;

typedef enum
{
    BUTTON_BOTH = 0,
    BUTTON_LEFT,
    BUTTON_RIGHT,
}Button_Type;

static Button_State button_state = WAIT_START;
static Button_Type button_type;

static struct itimerval timer;
static struct itimerval wheel_timer = {{0,0},{0,0,}};

static uint8_t cursor_level[2] = {0x00,0x00};
static uint8_t wheel_lock = 0; 



static volatile int signal_received = 0;





#define mouse_buffer_size  8
int mouse_buf[3][2] = {{subs_verticle_cursor_event,0},{subs_horizon_cursor_event,0},{subs_mouse_button_event,0}};

int fd;
int sockfd;
uint8_t sockbuf;
struct sockaddr_in ser_addr;



void handle_timer_sig(int signo)
{   
    switch (signo)
    {
        case SIGALRM:
        {
            if (wheel_lock)
            {
                write(fd,mouse_buf[Button],mouse_buffer_size);
                setitimer(ITIMER_REAL,&wheel_timer,NULL);
                return;
            }
            if (cursor_level[Verticle] && cursor_level[Horizon]) write(fd,mouse_buf[Verticle],mouse_buffer_size << 1);
            else
            {
                if (cursor_level[Verticle]) write(fd,mouse_buf[Verticle],mouse_buffer_size);
                else if (cursor_level[Horizon]) write(fd,mouse_buf[Horizon],mouse_buffer_size);
                else return;
            }
            setitimer(ITIMER_REAL,&timer,NULL);

        }
    }

    
}


void sig_handler(int sig)
{
    signal_received = sig;
}




void ble_thread_start(void* ptr);
static int recv_adv(int dev_describe);
static uint8_t cursor_data_handle(uint8_t cur_data);
static uint8_t button_data_handle(uint8_t button_data);


int transform_cursor_move(uint8_t level, uint8_t other_level, int* other_cursor_ptr)
{
    if (wheel_lock)
    {
        switch (level)
        {
        case 0x01:
            wheel_timer.it_value.tv_usec = 900000;
            return 0;
        case 0x02:
            wheel_timer.it_value.tv_usec = 500000;
            return 0;
        case 0x03:
            wheel_timer.it_value.tv_usec = 200000;
            return 0;
        case 0x04:
            wheel_timer.it_value.tv_usec = 100000;
            return 0;
        case 0x05:
            wheel_timer.it_value.tv_usec = 80000;
            return 0;
        case 0x06:
            wheel_timer.it_value.tv_usec = 40000;
            return 0;
        }
    }
    else if (other_level)
    {
        if (level > other_level) return (level - other_level);
        else if (level == other_level) return 1;
        if (*other_cursor_ptr < 0) *other_cursor_ptr = level - other_level;
        else *other_cursor_ptr = other_level - level;
    }
    switch (level)
    {
    case 0x01:
        timer.it_value.tv_usec = 10000;
        return 1;
    case 0x02:
        timer.it_value.tv_usec = 7000;
        return 1;
    case 0x03:
        timer.it_value.tv_usec = 5000;
        return 1;
    case 0x04:
        timer.it_value.tv_usec = 3000;
        return 1;
    case 0x05:
        timer.it_value.tv_usec = 900;
        return 1;
    case 0x06:
        timer.it_value.tv_usec = 600;
        return 1;
    }

}


int main()
{
    fd = open("/sys/devices/platform/virmouse/vmevent",O_RDWR);
    
    sockfd = socket(AF_INET, SOCK_DGRAM,0);
    memset(&ser_addr,0,sizeof(ser_addr));
    ser_addr.sin_family = AF_INET;
    ser_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    ser_addr.sin_port = htons(8999);


    signal(SIGALRM, handle_timer_sig);
    timer.it_value.tv_sec = 0;
    timer.it_value.tv_usec = 0;
    timer.it_interval.tv_sec = 0;
    timer.it_interval.tv_usec = 0;
   
    ble_thread_start(NULL);    

    close(fd);
    return 0;

}


void ble_thread_start(void* ptr)
{
    int dev_describe = hci_open_dev(hci_get_route(NULL));
    if (dev_describe < 0) 
    {
        perror("BLE device can't use");
        exit(1);
    }
    uint8_t own_type = 0x00;//ramdon
    uint8_t scan_type = 0x00;//passive scan
    uint8_t filter_type = 0x00;
    uint8_t filter_policy = 0x00;
    uint8_t filter_dup = 0x00;//accept repeat adv
    uint16_t interval = htobs(0x0010);
    uint16_t window = htobs(0x0010);
        
    if (hci_le_set_scan_parameters(dev_describe, scan_type, interval, window,
			               own_type, filter_policy, 10) < 0)//1000 is poll timeout
    {
        perror("Set scan parameters failed");
        exit(1);
    }
    if (hci_le_set_scan_enable(dev_describe, 0x01, filter_dup, 10) < 0)
    {
        perror("Enable scan failed");
        exit(1);
    }
    recv_adv(dev_describe);
    if (hci_le_set_scan_enable(dev_describe, 0x00, filter_dup, 10) < 0)
    {
        perror("Disable scan failed");
        exit(1);
    }

}


static int recv_adv(int dev_describe)
{
    unsigned char buf[HCI_MAX_EVENT_SIZE], *ptr;
    struct hci_filter nf, of;
    socklen_t olen;
    int len;

    olen = sizeof(of);
    if (getsockopt(dev_describe, SOL_HCI, HCI_FILTER, &of, &olen) < 0) 
    {
	printf("Could not get socket options\n");
        return -1;
    }
    hci_filter_clear(&nf);
    hci_filter_set_ptype(HCI_EVENT_PKT, &nf);
    hci_filter_set_event(EVT_LE_META_EVENT, &nf);

    if (setsockopt(dev_describe, SOL_HCI, HCI_FILTER, &nf, sizeof(nf)) < 0) 
    {
        printf("Could not set socket options\n");
        return -1;
    }
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_flags = SA_NOCLDSTOP;
    sa.sa_handler = sig_handler;
    sigaction(SIGINT, &sa, NULL);


    while (1)
    {
        evt_le_meta_event *meta;
	le_advertising_info *info;
        while ((len = read(dev_describe, buf, sizeof(buf))) < 0)
        {
            if (errno == EINTR && signal_received == SIGINT)
            {
                len = 0;
                goto done;
            }

            if (errno == EAGAIN || errno == EINTR) continue;
            goto done;
        }

        ptr = buf + (1 + HCI_EVENT_HDR_SIZE);
	len -= (1 + HCI_EVENT_HDR_SIZE);

	meta = (evt_le_meta_event*) ptr;

	if (meta->subevent != 0x02) goto done;
        info = (le_advertising_info *) (meta->data + 1);

        
        uint8_t ad_type = *(info->data + 1);
        if (ad_type == 0xFD)
        {
        
              uint8_t event = *(info->data + 3);
              if (event == mouse_cursor_event)
              {
                  uint8_t cur_data = *(info->data + 4);
			      if (cursor_data_handle(cur_data)) continue;

              }
              else if (event == mouse_button_event)
              {
                  uint8_t button_data = *(info->data + 4);
				  if (button_data_handle(button_data)) continue;
              }


        }
         

    }
    done:
	setsockopt(dev_describe, SOL_HCI, HCI_FILTER, &of, sizeof(of));
        

	if (len < 0) return -1;

	return 0;
}


static uint8_t cursor_data_handle(uint8_t cur_data)
{
	uint8_t level = cursor_level(cur_data);


	int i = (is_verticle_cursor(cur_data) ? Verticle : Horizon);
	int j = (is_verticle_cursor(cur_data) ? Horizon : Verticle);
	if (wheel_lock && i == Horizon) return 0x01;
	if (level == 0)
	{
		if (wheel_lock && i == Verticle)
		{
			wheel_timer.it_value.tv_sec = 0;
			setitimer(ITIMER_REAL, &wheel_timer, NULL);
		}
		else
		{
			cursor_level[i] = 0x00;
			if (cursor_level[j] != 0x00)
			{
				if (mouse_buf[j][1] < 0) mouse_buf[j][1] = -transform_cursor_move(cursor_level[j], 0, NULL);
				else mouse_buf[j][1] = transform_cursor_move(cursor_level[j], 0, NULL);
			}
			else timer.it_interval.tv_usec = 0;
			setitimer(ITIMER_REAL, &timer, NULL);
		}
		return 0x01;
	}
	int cur_range;
	if (cursor_level[i] == level) return 0x01;
	cursor_level[i] = level;
	if (!wheel_lock) cur_range = transform_cursor_move(level, cursor_level[j], &(mouse_buf[Button][1]));
	else cur_range = transform_cursor_move(level, 0, NULL);
	if (!(is_cursor_up_or_right(cur_data)))
	{
		if (wheel_lock && i == Verticle) mouse_buf[Button][1] = mouse_button_roll_down;
		else cur_range = -(cur_range);
	}
	else if (wheel_lock && i == Verticle) mouse_buf[Button][1] = mouse_button_roll_up;
	if (wheel_lock && i == Verticle)
	{
		write(fd, mouse_buf[Button], mouse_buffer_size);
		setitimer(ITIMER_REAL, &wheel_timer, NULL);
		return 0x01;
	}
	mouse_buf[i][1] = cur_range;
	if (cursor_level[j] == 0x00) setitimer(ITIMER_REAL, &timer, NULL);
	return 0x00;
}

static uint8_t button_data_handle(uint8_t button_data)
{
	static uint8_t write_mode = 0x01;
	if (button_state == WAIT_START)
	{
		wheel_lock = 0x00;
		if (button_data == button_left) button_type = BUTTON_LEFT, button_state = WAIT_LONG_OR_END;
		else if (button_data == button_right) button_type = BUTTON_RIGHT, button_state = WAIT_LONG_OR_END;
		else if (button_data == button_both) button_type = BUTTON_BOTH, button_state = WAIT_LONG_OR_END;
	}
	else if (button_state == WAIT_LONG_OR_END)
	{
		if (button_data == button_end)
		{
			button_state = WAIT_START;
			if (button_type == BUTTON_LEFT) mouse_buf[Button][1] = mouse_button_left_click;
			else if (button_type == BUTTON_RIGHT) mouse_buf[Button][1] = mouse_button_right_click;
			else return 0x01;
			write(fd, mouse_buf[Button], mouse_buffer_size);
		}
		else if (button_data == button_long)
		{
			button_state = WAIT_END;
			if (button_type == BUTTON_BOTH)
			{
				wheel_timer.it_value.tv_sec = 0;
				setitimer(ITIMER_REAL, &wheel_timer, NULL);
				wheel_lock = 0x01;
				mouse_buf[Button][1] = mouse_button_roll_push;
			}
			else if (button_type == BUTTON_LEFT)
			{
				if (write_mode == 0x00) mouse_buf[Button][1] = mouse_button_left_push;
				else
				{
					sockbuf = 1;
					sendto(sockfd, &sockbuf, 1, 0, (struct sockaddr*)&ser_addr, sizeof(ser_addr));
					return 0x01;
				}
			}
			else if (button_type == BUTTON_RIGHT)
			{
				write_mode ^= 0x01;
				return 0x01;
			}
			write(fd, mouse_buf[Button], mouse_buffer_size);
		}
	}
	else if (button_state == WAIT_END)
	{
		button_state = WAIT_START;
		if (button_type == BUTTON_BOTH)
		{
			wheel_timer.it_value.tv_sec = 0;
			setitimer(ITIMER_REAL, &wheel_timer, NULL);
			wheel_lock = 0x00;
		}
		else if (button_type == BUTTON_LEFT)
		{
			if (write_mode == 0x00)
			{
				mouse_buf[Button][1] = mouse_button_left_end_push;
				write(fd, mouse_buf[Button], mouse_buffer_size);
			}
			else
			{
				sockbuf = 0;
				sendto(sockfd, &sockbuf, 1, 0, (struct sockaddr*)&ser_addr, sizeof(ser_addr));

			}
		}

	}
	return 0x00;


}

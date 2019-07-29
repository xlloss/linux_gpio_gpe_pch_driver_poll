#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdbool.h>
#include <sys/poll.h>
#include <fcntl.h>
#include <pthread.h>
#include <errno.h>
#include "../DFI_GPIO_INT.h"


#define DFI_GPIO_DEV "/dev/GPIO_DEVICE"
char exit_code;

#define handle_error_en(en, msg) \
        do { errno = en; perror(msg); exit(EXIT_FAILURE); } while (0)

#define handle_error(msg) \
        do { perror(msg); exit(EXIT_FAILURE); } while (0)

static void *
thread_start(void *arg)
{
    int *fd = (int *)arg;
	int n;
	char buffer;
	struct pollfd pfd;

    fcntl(*fd, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK);
    pfd.fd = *fd;
    pfd.events = POLLIN;


	while (1) {
		printf("wait...\r\n");
		n = read(*fd, &buffer, 1);
		n = poll(&pfd, 1, -1);
		if (n < 0)
			break;

		printf("GPE : %02X\n\n", buffer);
	};

    return 0;
}

int main(int argc, char *argv[]) {

	int fd, s;
	int processPid;
    unsigned char outByte , inByte;
	struct sigaction sig;
	char buffer;
	pthread_attr_t attr;
	pthread_t thread_id;

    struct pollfd pfd;
    int n;

    s = pthread_attr_init(&attr);
    if (s != 0) {
        handle_error_en(s, "pthread_attr_init");\
		return -1;
	}


	fd = open(DFI_GPIO_DEV , O_RDWR);
	if(fd < 0) {
		printf("open dev fail: %d\n",fd);
		return -1;
	}

	s = pthread_create(&thread_id, &attr,
					&thread_start, &fd);
	if (s != 0) {
		handle_error_en(s, "pthread_create");
		return -1;
	}

    if(argc > 1) {
        outByte = strtol(argv[1] , NULL, 16);

        if (outByte & SUPER_IO_23_IOCTL)
			printf("Set SUPER_IO_23 HI\r\n");

        if (outByte & SUPER_IO_26_IOCTL)
			printf("Set SUPER_IO_26 HI\r\n");

        if(ioctl(fd, GPIO_IOCTL_SET_OUTPUT , &outByte) < 0){
			printf("ioctl set output fail\n");
            goto ERROR_HANDLE;
	    }
    }

	while (1) {
		exit_code = getchar();
	};

	s = pthread_join(thread_id, NULL);
	if (s != 0)
		handle_error_en(s, "pthread_join");

	printf("exit\r\n");

ERROR_HANDLE:

    close(fd);

	return 0;
}

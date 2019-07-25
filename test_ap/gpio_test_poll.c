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
#include "../DFI_GPIO_INT.h"

#define DFI_GPIO_DEV "/dev/GPIO_DEVICE"

int main(int argc, char *argv[]) {

	int fd;
	int processPid;
    unsigned char outByte , inByte;
	struct sigaction sig;
	char buffer;

    struct pollfd pfd;
    int n;

	fd = open(DFI_GPIO_DEV , O_RDWR);
	if(fd < 0) {
		printf("open dev fail: %d\n",fd);
		return -1;
	}

    fcntl(fd, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK);
    pfd.fd = fd;
    pfd.events = POLLIN;

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
		printf("wait...\n");
		n = read(fd, &buffer, 1);
		n = poll(&pfd, 1, -1);
		if (n < 0)
			break;

		printf("GPE : %02X\n\n", buffer);
	}

	printf("exit\r\n");

ERROR_HANDLE:

    close(fd);

	return 0;
}

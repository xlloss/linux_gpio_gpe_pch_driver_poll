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


#define GPIO_IOCTL_MAGIC 'i'
#define GPIO_IOCTL_SET_OUTPUT _IOW(GPIO_IOCTL_MAGIC, 1, int)
#define GPIO_IOCTL_GET_INPUT _IOW(GPIO_IOCTL_MAGIC, 2, int)
#define GPIO_IOCTL_SET_PID_CMD _IOW(GPIO_IOCTL_MAGIC, 3, int)
#define DFI_GPIO_DEV "/dev/GPIO_DEVICE"

#define GPO_23 0
#define GPO_26 1
#define SIG_TEST 44


void receiveData(int n, siginfo_t *info, void *unused) {

	printf("receive GPE value: 0x%x\n", info->si_int);
}

int main(int argc, char *argv[]) {

	int fd;
	int processPid;
    unsigned char outbyte , inbyte;
	struct sigaction sig;

	sig.sa_sigaction = receiveData;
	sig.sa_flags = SA_SIGINFO;

	sigaction(SIG_TEST, &sig, NULL);

	fd = open(DFI_GPIO_DEV, O_RDWR);

	if (fd < 0) {
		printf("open dev fail: %d\n",fd);
		return -1;
	}

	processPid = getpid();
	printf("pid:%d\n", processPid);

	if (ioctl(fd, GPIO_IOCTL_SET_PID_CMD, &processPid) < 0) {
		printf("ioctl set pid fail\n");
        goto ERROR_HANDLE;
	}

    if (argc > 1) {
        outbyte = strtol(argv[1], NULL, 16);

		if (outbyte & (1 << GPO_23))
			printf("ioctl set super io gpo_23 output\n");

		if (outbyte & (1 << GPO_26))
			printf("ioctl set super io gpo_26 output\n");

        if(ioctl(fd, GPIO_IOCTL_SET_OUTPUT , &outbyte) < 0){
		    printf("ioctl set output fail\n");
            goto ERROR_HANDLE;
	    }
    }

    if (ioctl(fd, GPIO_IOCTL_GET_INPUT, &inbyte) < 0) {
        printf("ioctl get input fail\n");
        goto ERROR_HANDLE;
    }

    printf("input status: %02X\n\n",inbyte);

    printf("wait...\n");

    while (1) {
		usleep(1000000);
    }



ERROR_HANDLE:

    close(fd);

	return 0;
}

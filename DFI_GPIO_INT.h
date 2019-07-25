#define GPIO_IOCTL_MAGIC		'i'
#define GPIO_IOCTL_SET_OUTPUT		_IOW(GPIO_IOCTL_MAGIC, 1 ,int)
#define GPIO_IOCTL_GET_INPUT		_IOW(GPIO_IOCTL_MAGIC, 2 ,int)
#define GPIO_IOCTL_SET_PID_CMD		_IOW(GPIO_IOCTL_MAGIC, 3 ,int)
#define SIG_TEST 			44

/* set this filed  by application */
#define SUPER_IO_23_IOCTL (1 << 0)
#define SUPER_IO_26_IOCTL (1 << 1)


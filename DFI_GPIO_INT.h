

#define	DEVICE_NAME	"GPIO_DEVICE"
#define MODULE_NAME	"GPIO_DEVICE"


//---SIGNAL DEFINE------------------------------------------------------------


#define		GPIO_IOCTL_MAGIC		'i'

#define		GPIO_IOCTL_SET_OUTPUT		_IOW(GPIO_IOCTL_MAGIC, 1 ,int)
#define		GPIO_IOCTL_GET_INPUT		_IOW(GPIO_IOCTL_MAGIC, 2 ,int)
#define		GPIO_IOCTL_SET_PID_CMD		_IOW(GPIO_IOCTL_MAGIC, 3 ,int)

#define 	SIG_TEST 			44
#define     NCT6106_SIO_ADDR            0x4E
#define     NCT6106_SIO_DATA            NCT6106_SIO_ADDR + 1

#define GPI0_GPE_NUM	0x16
#define GPI1_GPE_NUM	0x17



typedef struct _ioctlGpioMap{
	unsigned int * gpio_0_addr;
	unsigned int * gpio_1_addr;
	unsigned int * gpio_2_addr;
	unsigned int * gpio_3_addr;
}ioctlGpioMap;





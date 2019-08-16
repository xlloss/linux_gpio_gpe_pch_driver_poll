/*
#include <linux/version.h>
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,18)
#include <linux/config.h>
#endif
*/

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/acpi.h>
#include <linux/rcupdate.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,13,0)
#include <linux/sched/signal.h>
#else
#include <linux/signal.h>
#endif
#include <linux/spinlock.h>
#include <linux/time.h>


#include <asm/siginfo.h>
#include <asm/io.h>

#include "DFI_GPIO_INT.h"

#define DRIVER_TITLE "GPIO_DEVICE v0.0.0 \n Copyright (C) 2019 www.dfi.com"

//#define DEBUG_GPIO


#ifdef DEBUG_GPIO
#define MSG(format, arg...) printk(KERN_INFO "GPIO_DEVICE: " format "\n", ## arg)
#else
#define MSG(format, arg...)
#endif


//---------------------------------------------------------------------------------------
//-- Function declarations --------------------------------------------------------------
static int devone_devs = 1;
static int dev_major = 0;
       int pid = 0;

static struct cdev gpio_cdev;
struct class *devone_class = NULL;
static dev_t devone_dev;

//static DEFINE_SPINLOCK(outlock);

ioctlGpioMap gpioMap;

//---------------------------------------------------------------------------
int gpio_int_release(struct inode *, struct file *);
int gpio_int_open(struct inode *, struct file *);
long gpio_int_ioctl(struct file *,unsigned int, unsigned long);
int gpio_irq_setting(unsigned int uiGpeNum);
int gpio_value_setting(unsigned char gpioStatus);
int gpio_value_input(int * pValue);
int gpio_irq_remove_setting(unsigned int uiGpeNum);
static void superIo_init(void);

//---------------------------------------------------------------------------
long gpio_int_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    unsigned char gpioStatus;
    int  __user *p    = argp;
    int inValue;    
	
	switch(cmd)
	{

		case GPIO_IOCTL_SET_OUTPUT:

			if(copy_from_user(&gpioStatus, (void __user *)arg,sizeof(gpioStatus))){

				return -1;
			}

            gpio_value_setting(gpioStatus);

		break;


        case GPIO_IOCTL_GET_INPUT:
            
            if(gpio_value_input(&inValue) == 0){
                    put_user(inValue , p);
            }else{
                return -1;
            }
            
        break;
        

		case GPIO_IOCTL_SET_PID_CMD:
			
			if(copy_from_user(&pid, (void __user *)arg,sizeof(pid))){

				return -1;
			}

			printk(KERN_INFO  "%s user_application pid %d \n",DEVICE_NAME ,pid);

		break;

		default:
			return -ENOTTY;
	}

    	return 0;
}
//---------------------------------------------------------------------------
static const struct file_operations Fops = 
{
	open:		gpio_int_open,
	release:	gpio_int_release,
	unlocked_ioctl:	gpio_int_ioctl,
	owner:		THIS_MODULE,
	
};
//---------------------------------------------------------------------------
int gpio_int_release(struct inode *inode, struct file *filp)
{
	printk(KERN_INFO "%s exit gpio_int_release() \n",DEVICE_NAME);	
	return 0;
};

//---------------------------------------------------------------------------
int gpio_int_open(struct inode *inode, struct file *filp)
{
	printk(KERN_INFO "%s enter gpio_int_open() \n", DEVICE_NAME);	
	return 0;
};

int gpio_int_resume(struct device *dev)
{

	printk(KERN_INFO  "%s gpio_int_resume enter \n", DEVICE_NAME);

	return 0;
}

const struct dev_pm_ops gpio_pm_ops = {

	.resume = gpio_int_resume,
};

int gpio_value_input(int * pValue){

	unsigned int *F22_reg;
    unsigned int *F23_reg;

    F22_reg = gpioMap.gpio_0_addr;
    F23_reg = gpioMap.gpio_1_addr;

	if(F22_reg == NULL || F23_reg == NULL){
		return -1;
	}
    
    *pValue = ((*F22_reg & 0x02) >> 1) + (*F23_reg & 0x02);

    MSG("F22 gpio DW1 status: 0x%08X ", *F22_reg)
    MSG("F23 gpio DW1 status: 0x%08X ", *F23_reg)

    return 0;
}


static int send_msg_to_pid(unsigned char ucGpioStatus){
	
	struct siginfo 		info;
	struct task_struct 	*t;
	int ret;

	if(pid == 0)	
		return -1;

	memset(&info, 0, sizeof(struct siginfo));
	info.si_signo = SIG_TEST;

	info.si_code  = SI_QUEUE;
	info.si_int   = ucGpioStatus;

	rcu_read_lock();
	t = pid_task(find_pid_ns(pid, &init_pid_ns), PIDTYPE_PID);

	if(t == NULL){
		printk(KERN_INFO  "%s no such pid: %d \n", DEVICE_NAME, pid);
		rcu_read_unlock();
		return -1;
	}
	rcu_read_unlock();

	ret = send_sig_info(SIG_TEST, &info, t);

	if(ret < 0){
		printk(KERN_INFO  "%s send signal error: %d \n", DEVICE_NAME, ret);
		return ret;
	}

	return 0;
}


static unsigned int gpio_irq_handler(acpi_handle gpe_device ,unsigned int gpe_number ,void *data) 
{ 
    unsigned int *reg;

	printk(KERN_INFO  "%s gpio_irq_handler gep: 0x%02X \n", DEVICE_NAME , gpe_number);

    switch(gpe_number){
		
		case 0x16:
			reg = gpioMap.gpio_0_addr;
		break;

		case 0x17:
			reg = gpioMap.gpio_1_addr;
		break;

		default:
			return ACPI_INTERRUPT_HANDLED | ACPI_REENABLE_GPE;
		break;
	}

    if(!(*reg & 0x02)){
        send_msg_to_pid((unsigned char)gpe_number);
    }

	return ACPI_INTERRUPT_HANDLED | ACPI_REENABLE_GPE;
}

static void superIo_CR_WriteByte(unsigned char addr,unsigned char LD, unsigned char data){

    outb(0x07 , NCT6106_SIO_ADDR);   
    outb(LD   , NCT6106_SIO_DATA);  //switch Logic Device Number.

    outb(addr , NCT6106_SIO_ADDR);   
    outb(data   , NCT6106_SIO_DATA);
    
    return;
}

static unsigned char superIo_CR_ReadByte(unsigned int addr, unsigned char LD){

    unsigned char ucByte;

    outb(0x07 , NCT6106_SIO_ADDR);   
    outb(LD   , NCT6106_SIO_DATA);  //switch Logic Device Number.

    outb(addr , NCT6106_SIO_ADDR);
    ucByte = inb(NCT6106_SIO_DATA);

    return ucByte;
}

static void superIo_init(){

    unsigned char ucByte;

    outb(0x87 , NCT6106_SIO_ADDR);
    outb(0x87 , NCT6106_SIO_ADDR);

    ucByte = superIo_CR_ReadByte(0x30,0x07);
    ucByte = ucByte | 0x04;

    superIo_CR_WriteByte(0x30,0x07,ucByte);

    superIo_CR_WriteByte(0xE2,0x08,0xB7); //switch GP23,GP26 to GPIO 
    
    superIo_CR_WriteByte(0xE8,0x07,0xB7); //switch GP23,GP26 to Output

    return;
}
//---------------------------------------------------------------------------
int gpio_value_setting(unsigned char gpioStatus){

    unsigned char ucByte;

    ucByte = superIo_CR_ReadByte(0xE9 , 0x07);

    if(gpioStatus & 0x01){  //GP23
        ucByte = ucByte | 0x08;
    }else{
        ucByte = ucByte & 0xF7;
    }

    if(gpioStatus & 0x02){  //GP26
        ucByte = ucByte | 0x40;
    }else{
        ucByte = ucByte & 0xBF;
    }
        
    superIo_CR_WriteByte(0xE9 , 0x07 , ucByte);

    return 0;
}
//---------------------------------------------------------------------------
int __init gpio_int_init(void)
{
	int result;
	dev_t dev = MKDEV(dev_major, 0);
	int cdev_err = 0;
	int alloc_ret = 0;

	MSG(DRIVER_TITLE);

	
	MSG("alloc_chrdev_region");
    result = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME);
	dev_major = MAJOR(dev);

    if (result < 0) {
        MSG("unable to get major %d",dev_major);
        MSG("result number %d",result);
                goto error;
				//return result;
    }
		
	cdev_init(&gpio_cdev, &Fops);
	cdev_err = cdev_add(&gpio_cdev, MKDEV(dev_major, 0), devone_devs);
		
	if(cdev_err)
		goto error;
		

	devone_class = class_create(THIS_MODULE,DEVICE_NAME);
	devone_class->pm = &gpio_pm_ops;
	
	if(IS_ERR(devone_class)){

		goto error;
	}

	devone_dev = MKDEV(dev_major, 0);

	device_create(
			devone_class,
			NULL,
			devone_dev,
			NULL,
			MODULE_NAME
			);
	
	printk(KERN_INFO "%s driver(major %d) installed. \n", DEVICE_NAME, dev_major);

	gpioMap.gpio_0_addr = ioremap_nocache(0xFD6B0B30,4);
	gpioMap.gpio_1_addr = ioremap_nocache(0xFD6B0B40,4);

	gpio_irq_setting(GPI0_GPE_NUM);
	gpio_irq_setting(GPI1_GPE_NUM);

    superIo_init();

	return 0;
error:
	if(cdev_err == 0)
		cdev_del(&gpio_cdev);
	if(alloc_ret == 0)
		unregister_chrdev_region(dev, devone_devs);

	return -1;
}


int gpio_irq_setting(unsigned int uiGpeNum){

	acpi_status status;

	
	printk(KERN_INFO  "========================================================\n");
	status = acpi_install_gpe_handler(NULL,
					  uiGpeNum, 	
					  ACPI_GPE_EDGE_TRIGGERED,
					  &gpio_irq_handler,
					  NULL);


	printk(KERN_INFO  "%s acpi_install_gpe_handler GPE 0x%02X: 0x%d \n", DEVICE_NAME , uiGpeNum , status);	//


	printk(KERN_INFO  "========================================================\n");	

	status = acpi_enable_gpe(NULL ,uiGpeNum);

	return 0;
}

int gpio_irq_remove_setting(unsigned int uiGpeNum){

	acpi_status status;
	
	status = acpi_remove_gpe_handler(NULL,
					 uiGpeNum, 	
					 &gpio_irq_handler);

	printk(KERN_INFO  "%s acpi_remove_gpe_handler: 0x%d \n", DEVICE_NAME, status);	//

	return status; 
}


//---------------------------------------------------------------------------
void __exit gpio_int_exit(void)
{
	dev_t dev = MKDEV(dev_major, 0);

	device_destroy(devone_class , devone_dev);
	class_destroy(devone_class);	

	unregister_chrdev_region(dev, devone_devs);		
	cdev_del(&gpio_cdev);

	gpio_irq_remove_setting(GPI0_GPE_NUM);
	gpio_irq_remove_setting(GPI1_GPE_NUM);


	if(gpioMap.gpio_0_addr != NULL)	{
		iounmap(gpioMap.gpio_0_addr);
	}

	if(gpioMap.gpio_1_addr != NULL)	{
		iounmap(gpioMap.gpio_1_addr);
	}

	printk(KERN_ALERT "%s driver removed. \n", DEVICE_NAME);

}

//---------------------------------------------------------------------------
MODULE_LICENSE("GPL");
MODULE_AUTHOR("jack.lan@dfi.com");
MODULE_DESCRIPTION("www.dfi.com");

module_init(gpio_int_init);
module_exit(gpio_int_exit);



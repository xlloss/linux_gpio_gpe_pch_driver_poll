/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 * This program modify from jack DFI_GPIO_INT.c 
 * slash.huang@dfi.com
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
#include <linux/sched/signal.h>
#include <linux/spinlock.h>
#include <linux/poll.h>
#include <linux/time.h>
#include <asm/siginfo.h>
#include <asm/io.h>
#include "DFI_GPIO_INT.h"

#define DRIVER_TITLE "GPIO_DEVICE v0.0.0 \n Copyright (C) 2019 www.dfi.com"

//#define DEBUG_GPIO


#ifdef DEBUG_GPIO
#define MSG(format, arg...) pr_info("GPIO_DEVICE: " format "\n", ## arg)
#else
#define MSG(format, arg...)
#endif



#define CHAR_MIN_BASE 0
#define CHAR_MIN_COUNT 1


#define	DEVICE_NAME "GPIO_DEVICE"
#define MODULE_NAME "GPIO_DEVICE"


#define PCH_GPI0_GPE_NUM 0x16
#define PCH_GPI1_GPE_NUM 0x17
#define PCH_BAR_GPIO_0 0xFD6B0B30
#define PCH_BAR_GPIO_1 0xFD6B0B40


typedef struct _ioctlgpiomap {
	unsigned int * gpio_0_addr;
	unsigned int * gpio_1_addr;
	unsigned int * gpio_2_addr;
	unsigned int * gpio_3_addr;
} ioctlgpiomap;


#define NCT6106_SIO_ADDR 0x4E
#define NCT6106_SIO_DATA NCT6106_SIO_ADDR + 1

enum {
SUPER_GPIO_20_OFF = 0,
SUPER_GPIO_21_OFF,
SUPER_GPIO_22_OFF,
SUPER_GPIO_23_OFF,
SUPER_GPIO_24_OFF,
SUPER_GPIO_25_OFF,
SUPER_GPIO_26_OFF,
SUPER_GPIO_27_OFF,
};


#define SUPER_EN_EXT_MODE 0x87
#define SUPER_LOGIC_DEV_SET 0x07
#define SUPER_LOGIC_GPIO 0x07
#define LOGIC_DEV_CR(x) (x)

#define SUPER_LOGIC_GPIO_WDT1 0x08
#define SUPER_GPIO2_GROUP (1 << 2)

#define READ_SIZE 1

#define PCH_GPE_NUM_GPIO0 0x16
#define PCH_GPE_NUM_GPIO1 0x17
#define PCH_GPIO_RX_SAT 0x02

static DECLARE_WAIT_QUEUE_HEAD(read_wait);
static struct fasync_struct *fasync;

static int devone_devs = 1;
static int dev_major = 0;
       int pid = 0;

int gpio_input_gpe;

static struct cdev gpio_cdev;
struct class *devone_class = NULL;
static dev_t devone_dev;

ioctlgpiomap gpiomap;

int gpio_int_release(struct inode *, struct file *);
int gpio_int_open(struct inode *, struct file *);
long gpio_int_ioctl(struct file *,unsigned int, unsigned long);
int gpio_irq_setting(unsigned int uiGpeNum);
int gpio_value_setting(unsigned char gpio_sat);
int gpio_value_input(int * pValue);
int gpio_irq_remove_setting(unsigned int uiGpeNum);
static void super_io_init(void);


long gpio_int_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    unsigned char gpio_sat;
    int  __user *p = argp;
    int inValue;

	switch(cmd) {

		case GPIO_IOCTL_SET_OUTPUT:
			if (copy_from_user(&gpio_sat, (void __user *)arg, sizeof(gpio_sat)))
				return -1;

            gpio_value_setting(gpio_sat);
			break;

        case GPIO_IOCTL_GET_INPUT:
            if (gpio_value_input(&inValue) == 0)
				put_user(inValue , p);
            else
                return -1;
			break;


		case GPIO_IOCTL_SET_PID_CMD:
			if (copy_from_user(&pid, (void __user *)arg,sizeof(pid)))
				return -1;

			pr_debug("%s user_application pid %d \n",DEVICE_NAME ,pid);
			break;

		default:
			return -ENOTTY;
	}

    return 0;
}



static ssize_t
gpio_input_read(struct file *file, char __user *buf, size_t nbytes, loff_t *ppos)
{
	ssize_t n;
	int inValue;

	if (nbytes == 0)
		return 0;

	if (nbytes > 0) {
		n = nbytes;
		if (n > READ_SIZE)
			n = READ_SIZE;
	}

	wait_event_interruptible(read_wait, gpio_input_gpe);

	inValue = gpio_input_gpe;
	put_user(inValue , buf);
	n = 1;

	return n;
}


static unsigned int
gpio_int_poll(struct file *file, poll_table * wait)
{
	unsigned int mask;

	poll_wait(file, &read_wait, wait);
	mask = 0;

	if (gpio_input_gpe)
		mask |= POLLIN | POLLRDNORM;

	gpio_input_gpe = 0;

	return mask;
}

static int gpio_int_fasync(int fd, struct file *filp, int on)
{
	return fasync_helper(fd, filp, on, &fasync);
}

static const struct file_operations Fops =
{
	open:		gpio_int_open,
	read:		gpio_input_read,
	release:	gpio_int_release,
	unlocked_ioctl:	gpio_int_ioctl,
	poll: gpio_int_poll,
	fasync:	gpio_int_fasync,
	owner:		THIS_MODULE,

};

int gpio_int_release(struct inode *inode, struct file *filp)
{
	pr_debug("%s exit gpio_int_release() \n",DEVICE_NAME);
	return 0;
};

int gpio_int_open(struct inode *inode, struct file *filp)
{
	pr_debug("%s enter gpio_int_open() \n", DEVICE_NAME);
	return 0;
};

int gpio_int_resume(struct device *dev)
{
	pr_debug("%s gpio_int_resume enter \n", DEVICE_NAME);
	return 0;
}

const struct dev_pm_ops gpio_pm_ops = {
	.resume = gpio_int_resume,
};

int gpio_value_input(int * pValue)
{
	unsigned int *gpio_f22_reg;
    unsigned int *gpio_f23_reg;

    gpio_f22_reg = gpiomap.gpio_0_addr;
    gpio_f23_reg = gpiomap.gpio_1_addr;

	if (gpio_f22_reg == NULL || gpio_f23_reg == NULL)
		return -1;

    *pValue = ((*gpio_f22_reg & 0x02) >> 1) + (*gpio_f23_reg & 0x02);

    MSG("PCH F22 gpio DW1 status: 0x%08X ", *gpio_f22_reg)
    MSG("PCH F23 gpio DW1 status: 0x%08X ", *gpio_f23_reg)

    return 0;
}


static int send_msg_to_pid(unsigned char ucgpio_sat)
{
	struct siginfo 		info;
	struct task_struct 	*t;
	int ret;

	if(pid == 0)
		return -1;

	memset(&info, 0, sizeof(struct siginfo));
	info.si_signo = SIG_TEST;

	info.si_code  = SI_QUEUE;
	info.si_int   = ucgpio_sat;

	rcu_read_lock();
	t = pid_task(find_pid_ns(pid, &init_pid_ns), PIDTYPE_PID);

	if(t == NULL){
		pr_info("%s no such pid: %d \n", DEVICE_NAME, pid);
		rcu_read_unlock();
		return -1;
	}
	rcu_read_unlock();

	ret = send_sig_info(SIG_TEST, &info, t);

	if(ret < 0){
		pr_info("%s send signal error: %d \n", DEVICE_NAME, ret);
		return ret;
	}

	return 0;
}

static unsigned int gpio_irq_handler(acpi_handle gpe_device
	,unsigned int gpe_number ,void *data)
{
    unsigned int *reg;

	pr_debug("%s gpio_irq_handler gep: 0x%02X \n",
		DEVICE_NAME , gpe_number);

    switch(gpe_number) {

		case PCH_GPE_NUM_GPIO0:
			reg = gpiomap.gpio_0_addr;
		break;

		case PCH_GPE_NUM_GPIO1:
			reg = gpiomap.gpio_1_addr;
		break;

		default:
			return ACPI_INTERRUPT_HANDLED | ACPI_REENABLE_GPE;
		break;
	}


    if ((*reg & PCH_GPIO_RX_SAT) == 0) {
		gpio_input_gpe = gpe_number;
        send_msg_to_pid((unsigned char)gpe_number);
	}

	wake_up_interruptible(&read_wait);
	kill_fasync(&fasync, SIGIO, POLL_IN);

	return ACPI_INTERRUPT_HANDLED | ACPI_REENABLE_GPE;
}


static void superio_cr_writebyte(unsigned char target_cr,
	unsigned char target_ld, unsigned char data)
{

	/* switch Logic Device Number. */
    outb(SUPER_LOGIC_DEV_SET ,NCT6106_SIO_ADDR);
    outb(target_ld ,NCT6106_SIO_DATA);

    outb(target_cr ,NCT6106_SIO_ADDR);
    outb(data ,NCT6106_SIO_DATA);
}


static unsigned char superIo_CR_ReadByte(unsigned int target_cr,
	unsigned char target_ld)
{

    unsigned char byte;

	/* switch Logic Device Number */
    outb(SUPER_LOGIC_DEV_SET ,NCT6106_SIO_ADDR);
    outb(target_ld ,NCT6106_SIO_DATA);

    outb(target_cr ,NCT6106_SIO_ADDR);
    byte = inb(NCT6106_SIO_DATA);

    return byte;
}


static void super_io_init()
{
    unsigned char byte;
    unsigned char gpio2_mux = 0xFF;

    outb(SUPER_EN_EXT_MODE ,NCT6106_SIO_ADDR);
    outb(SUPER_EN_EXT_MODE ,NCT6106_SIO_ADDR);

    byte = superIo_CR_ReadByte(LOGIC_DEV_CR(0x30), SUPER_LOGIC_GPIO);
    byte = byte | SUPER_GPIO2_GROUP;

    superio_cr_writebyte(LOGIC_DEV_CR(0x30), SUPER_LOGIC_GPIO, byte);

	/* setting SUPER_GPIO2_GROUPROUP_2_MUX for gp23, gp26 */
	gpio2_mux &= ~((1 << SUPER_GPIO_26_OFF) | (1 << SUPER_GPIO_23_OFF));
    superio_cr_writebyte(LOGIC_DEV_CR(0xE2), SUPER_LOGIC_GPIO_WDT1, gpio2_mux);

    /* set gp23, gp26 as output */
    superio_cr_writebyte(LOGIC_DEV_CR(0xE8), SUPER_LOGIC_GPIO, gpio2_mux);

    return;
}

int gpio_value_setting(unsigned char gpio_sat)
{

	unsigned char byte;

	byte = superIo_CR_ReadByte(LOGIC_DEV_CR(0xE9) ,SUPER_LOGIC_GPIO);

	/* SUPER_IO_GP23 */
    if (gpio_sat & SUPER_IO_23_IOCTL)
        byte = byte | (1 << SUPER_GPIO_23_OFF);
    else
        byte = byte & ~(1 << SUPER_GPIO_23_OFF);


	/* SUPER_IO_GP26 */
	if (gpio_sat & SUPER_IO_26_IOCTL)
		byte = byte | (1 << SUPER_GPIO_26_OFF);
	else
		byte = byte & ~(1 << SUPER_GPIO_26_OFF);

	superio_cr_writebyte(LOGIC_DEV_CR(0xE9), SUPER_LOGIC_GPIO, byte);

	return 0;
}

int __init gpio_int_init(void)
{
	int result;
	dev_t dev = MKDEV(dev_major, 0);
	int cdev_err = 0;
	int alloc_ret = 0;

	MSG(DRIVER_TITLE);

	MSG("alloc_chrdev_region");
    result = alloc_chrdev_region(&dev, CHAR_MIN_BASE, CHAR_MIN_COUNT, DEVICE_NAME);
	dev_major = MAJOR(dev);
    if (result < 0) {
        MSG("unable to get major %d", dev_major);
        MSG("result number %d", result);
			goto error;
    }

	cdev_init(&gpio_cdev, &Fops);
	cdev_err = cdev_add(&gpio_cdev, MKDEV(dev_major, 0), devone_devs);

	if(cdev_err)
		goto error;

	devone_class = class_create(THIS_MODULE,DEVICE_NAME);
	devone_class->pm = &gpio_pm_ops;

	if(IS_ERR(devone_class))
		goto error;

	devone_dev = MKDEV(dev_major, 0);

	device_create(
			devone_class, NULL, devone_dev, NULL, MODULE_NAME);

	pr_debug("%s driver(major %d) installed.\n", DEVICE_NAME, dev_major);

	gpio_input_gpe = 0;

	gpiomap.gpio_0_addr = ioremap_nocache(PCH_BAR_GPIO_0, 4);
	gpiomap.gpio_1_addr = ioremap_nocache(PCH_BAR_GPIO_1, 4);

	gpio_irq_setting(PCH_GPI0_GPE_NUM);
	gpio_irq_setting(PCH_GPI1_GPE_NUM);

    super_io_init();

	return 0;

error:
	if(cdev_err == 0)
		cdev_del(&gpio_cdev);

	if(alloc_ret == 0)
		unregister_chrdev_region(dev, devone_devs);

	return -1;
}


int gpio_irq_setting(unsigned int uiGpeNum)
{

	acpi_status status;
	status = acpi_install_gpe_handler(NULL,
					  uiGpeNum,
					  ACPI_GPE_EDGE_TRIGGERED,
					  &gpio_irq_handler,
					  NULL);


	pr_debug("%s acpi_install_gpe_handler GPE 0x%02X: 0x%d\n",
		DEVICE_NAME, uiGpeNum, status);

	status = acpi_enable_gpe(NULL ,uiGpeNum);

	return 0;
}

int gpio_irq_remove_setting(unsigned int uiGpeNum)
{

	acpi_status status;

	status = acpi_remove_gpe_handler(NULL, uiGpeNum, &gpio_irq_handler);

	pr_debug("%s acpi_remove_gpe_handler: 0x%d\n", DEVICE_NAME, status);

	return status;
}



void __exit gpio_int_exit(void)
{
	dev_t dev = MKDEV(dev_major, 0);

	device_destroy(devone_class, devone_dev);
	class_destroy(devone_class);

	unregister_chrdev_region(dev, devone_devs);
	cdev_del(&gpio_cdev);

	gpio_irq_remove_setting(PCH_GPI0_GPE_NUM);
	gpio_irq_remove_setting(PCH_GPI1_GPE_NUM);


	if(gpiomap.gpio_0_addr != NULL)
		iounmap(gpiomap.gpio_0_addr);

	if(gpiomap.gpio_1_addr != NULL)
		iounmap(gpiomap.gpio_1_addr);

	pr_debug(KERN_ALERT "%s driver removed\n", DEVICE_NAME);

}


MODULE_LICENSE("GPL");
MODULE_AUTHOR("jack.lan@dfi.com");
MODULE_DESCRIPTION("www.dfi.com");

module_init(gpio_int_init);
module_exit(gpio_int_exit);

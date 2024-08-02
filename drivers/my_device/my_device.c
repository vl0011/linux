// Misc Device driver

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>

#include <linux/fs.h>
#include <linux/miscdevice.h>

#define DEVICE_NAME ("my_device")

static struct miscdevice driver;

static int my_open(struct inode *inode, struct file *file)
{
	dev_info(driver.this_device, "my device open");
	return 0;
}

static int my_release(struct inode *inode, struct file *file)
{
	dev_info(driver.this_device, "my device close");
	return 0;
}

static ssize_t my_read(struct file *file, char *buf, size_t length, loff_t *off)
{
	dev_info(driver.this_device, "my device read");
	return 0;
}

static ssize_t my_write(struct file *file, const char *buf, size_t length, loff_t *off)
{
	dev_info(driver.this_device, "my device write");
	return 0;
}

static struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = my_open,
	.release = my_release,
	.read = my_read,
	.write = my_write,
};

static struct miscdevice driver = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &fops,
};

static int __init my_device_init(void)
{
	pr_info("my device init");

	return misc_register(&driver);
}

static void __exit my_device_exit(void)
{
	pr_info("my device exit");
	misc_deregister(&driver);
}

module_init(my_device_init);
module_exit(my_device_exit);

MODULE_AUTHOR("Bob");
MODULE_DESCRIPTION("my_device");
MODULE_LICENSE("GPL");
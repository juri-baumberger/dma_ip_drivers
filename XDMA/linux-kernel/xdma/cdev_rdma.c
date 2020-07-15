
#include <linux/ioctl.h>
#include "xdma_cdev.h"
#include "cdev_ctrl.h"

/*
 * character device file operations for control bus (through control bridge)
 */
static const struct file_operations ctrl_fops = {
	.owner = THIS_MODULE,
	.open = char_open,
	.release = char_close,
};

void cdev_rdma_init(struct xdma_cdev *xcdev)
{
	cdev_init(&xcdev->cdev, &ctrl_fops);
}

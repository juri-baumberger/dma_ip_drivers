
#include <linux/ioctl.h>
#include <linux/types.h>
#include <nv-p2p.h>
#include "xdma_cdev.h"
#include "cdev_ctrl.h"
#include "cdev_rdma.h"

#define GPU_PAGE_SHIFT	16
#define GPU_PAGE_SIZE	(((u64)1) << GPU_PAGE_SHIFT)
#define GPU_PAGE_OFFSET	(GPU_PAGE_SIZE - 1)
#define GPU_PAGE_MASK	(~GPU_PAGE_OFFSET)

struct rdma_cuda_surface {
	struct xdma_cdev		*xcdev;
	u64				va;
	u64				offset;
	u64				len;
	int				handle;
	struct nvidia_p2p_page_table	*page_table;
};

struct idr indexPool;

static void rdma_p2p_free_callback(void *data)
{
	struct rdma_cuda_surface *cusurf = data;
	nvidia_p2p_free_page_table(cusurf->page_table);
	kfree(cusurf);
}

static int rdma_ioctl_pin_cuda(struct xdma_cdev *xcdev, unsigned long arg)
{
	printk(KERN_INFO "RDMA IOCTL PIN CUDA\n");
	void __user *argp = (void __user *)arg;
	struct rdma_pin_cuda pin_params;
	struct rdma_cuda_surface *cusurf;
	u64 aligned_len;
	int ret;
	
	if (copy_from_user(&pin_params, argp, sizeof(pin_params)))
		return -EFAULT;
	
	cusurf = kzalloc(sizeof(*cusurf), GFP_KERNEL);
	if (!cusurf)
		return -ENOMEM;
		
	cusurf->xcdev = xcdev;
	cusurf->va = pin_params.va & GPU_PAGE_MASK;
	cusurf->offset = pin_params.va & GPU_PAGE_OFFSET;
	cusurf->len = pin_params.size;	
	aligned_len = (cusurf->offset + cusurf->len + GPU_PAGE_SIZE - 1) & GPU_PAGE_MASK;
	
	ret = nvidia_p2p_get_pages(0, 0, cusurf->va, aligned_len, &cusurf->page_table, rdma_p2p_free_callback, cusurf);
	if (ret < 0) {
		kfree(cusurf);
		return ret;
	}
	
	// create handle for pinned memory, used later for unpinning
	cusurf->handle = idr_alloc(&indexPool, cusurf, 0, 0, GFP_KERNEL);
	pin_params.handle = cusurf->handle;
	
	printk(KERN_INFO "CUDA PIN PageTable Entries %d, Size %d, physical addresses:\n", cusurf->page_table->entries, cusurf->page_table->page_size);
	int numEntries = cusurf->page_table->entries;
	if (numEntries > 0)
	{
	    pin_params.startAddr = cusurf->page_table->pages[0]->physical_address;
	    printk(KERN_INFO "Pinned memory starting at physical address 0x%llx \n", cusurf->page_table->pages[0]->physical_address);
	}
	
	if (copy_to_user(argp, &pin_params, sizeof(pin_params)))
		return -EFAULT;
	
	return 0;
}

static int rdma_ioctl_unpin_cuda(struct xdma_cdev *xcdev, unsigned long arg)
{
	printk(KERN_INFO "RDMA IOCTL UNPIN CUDA\n");
	void __user *argp = (void __user *)arg;
	struct rdma_unpin_cuda unpin_params;
	struct rdma_cuda_surface *cusurf;
	
	if (copy_from_user(&unpin_params, argp, sizeof(unpin_params)))
		return -EFAULT;
		
	cusurf = idr_find(&indexPool, unpin_params.handle);
	if (!cusurf) {
		return -EINVAL;
	}
	
	idr_remove(&indexPool, unpin_params.handle);
	cusurf->handle = -1;
	
	nvidia_p2p_put_pages(0, 0, cusurf->va, cusurf->page_table);
}

static long rdma_fops_unlocked_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	struct xdma_cdev *xcdev = (struct xdma_cdev *)filep->private_data;
	int result;

	result = xcdev_check(__func__, xcdev, 0);
	if (result < 0)
		return result;
		
	switch (cmd) {
	case RDMA_IOC_PIN:
		return rdma_ioctl_pin_cuda(xcdev, arg);
	case RDMA_IOC_UNPIN:
		return rdma_ioctl_unpin_cuda(xcdev, arg);
	default:
		return -EINVAL;
	}
}

/*
 * character device file operations for control bus (through control bridge)
 */
static const struct file_operations ctrl_fops = {
	.owner = THIS_MODULE,
	.open = char_open,
	.release = char_close,
	.unlocked_ioctl = rdma_fops_unlocked_ioctl,
};

void cdev_rdma_init(struct xdma_cdev *xcdev)
{
	int result;
	
	idr_init(&indexPool);
	cdev_init(&xcdev->cdev, &ctrl_fops);
	result = pcie_set_readrq(xcdev->xpdev->pdev, 1024);
	printk(KERN_INFO "Set XDMA PCI max read to 1024. Res %d\n", result);
}

struct rdma_pin_cuda {
	/* In */
	__u64 va;
	__u64 size;
	/* Out */
	__u32 handle;
	__u64 startAddr;
};

struct rdma_unpin_cuda {
	/* In */
	__u32 handle;
};

struct rdma_h2c2h_dma {
	/* In */
	/* Malloc: Pointer, CUDA: Handle from RDMA_IOC_PIN */
	__u64 src;
	/* Malloc: Pointer, CUDA: Handle from RDMA_IOC_PIN */
	__u64 dst;
	__u64 len;
	__u64 flags;
	/* Out */
	__u64 dma_time_ns;
};
#define RDMA_H2C2H_DMA_FLAG_SRC_IS_CUDA (1 << 0)
#define RDMA_H2C2H_DMA_FLAG_DST_IS_CUDA (1 << 1)

enum RDMA_IOC_TYPES {
	RDMA_IOC_PIN,
	RDMA_IOC_UNPIN,
	RDMA_IOC_H2C2H_DMA,
	RDMA_IOC_MAX
};

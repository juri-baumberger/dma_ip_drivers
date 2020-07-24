/*
 * Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#define GL_GLEXT_PROTOTYPES
#include <cuda.h>
#include <cuda_gl_interop.h>
#include <cuda_runtime_api.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "../../xdma/cdev_rdma.h"

#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glut.h>

#define SURFACE_W	3840
#define SURFACE_H	2160
#define SURFACE_SIZE	(SURFACE_W * SURFACE_H)

#define INPUT_WIDTH_BYTES ((SURFACE_W * 20)/8)
#define INPUT_STRIDE_BYTES 0x4000
#define INPUT_BYTES_PER_SAMPLE 5

#define OFFSET(x, y)	(((y) * SURFACE_W) + x)
#define DATA(x, y)	(((y & 0xffff) << 16) | ((x) & 0xffff))

inline __device__ __host__ float clamp(float f, float a, float b)
{
    return fmaxf(a, fminf(f, b));
}

// round up n/m
inline int iDivUp(int n, int m)
{
    return (n + m - 1) / m;
}

extern "C" __global__ void fill_surface(uint32_t *output, uint32_t xor_val)
{
	unsigned int pos_x = (blockIdx.x * blockDim.x) + threadIdx.x;
	unsigned int pos_y = (blockIdx.y * blockDim.y) + threadIdx.y;

	output[OFFSET(pos_x, pos_y)] = DATA(pos_x, pos_y) ^ xor_val;
}

extern "C" __global__ void convertYuv10ToRGBA(unsigned char *input, unsigned char *output, unsigned int width, unsigned int stride)
{
	unsigned int orig_pos_x = (blockIdx.x * blockDim.x) + threadIdx.x;
	unsigned int pos_x = ((blockIdx.x * blockDim.x) + threadIdx.x) * INPUT_BYTES_PER_SAMPLE;
	unsigned int pos_y = (blockIdx.y * blockDim.y) + threadIdx.y;
	
	// YUV 10bit is 20bit per pixel. We take 2 samples, so 40bit = 5 bytes worth of data, and output 2 RGBA samples, so 64 bit = 8 bytes of data
	// Pixels are shifted in batches of 512bit = 64 bytes, so the first bytes will the MSBs of that batch
	const unsigned int BatchSize = 64;
	unsigned int batchIndex = pos_x % BatchSize;
	unsigned int nextBatchOffset = ((pos_x / BatchSize) + 1) * BatchSize;
	int xOffset = (nextBatchOffset-INPUT_BYTES_PER_SAMPLE) - batchIndex;
	int offset = xOffset + pos_y * stride;
	const unsigned char inputPx0 = input[offset+0];
	const unsigned char inputPx1 = input[offset+1];
	const unsigned char inputPx2 = input[offset+2];
	const unsigned char inputPx3 = input[offset+3];
	const unsigned char inputPx4 = input[offset+4];
	
	float y0 = inputPx0 + (inputPx1 & 0x03) * 256.0f;
	float u = (inputPx1 & 0xFC) / 4.0f + (inputPx2 & 0x0F) * 64.0f;
	float y1 = (inputPx2 & 0xF0) / 16.0f + (inputPx3 & 0x3F) * 16.0f;
	float v = (inputPx3 & 0xC0) / 64.0f +  inputPx4 * 4.0f;
	
	const float4 px0 = make_float4( y0 + 1.140f * v,
					 y0 - 0.395f * u - 0.581f * v,
					 y0 + 2.032f * u, 255.0f );

	const float4 px1 = make_float4( y1 + 1.140f * v,
					 y1 - 0.395f * u - 0.581f * v,
					 y1 + 2.032f * u, 255.0f );
	
	const uchar4 rgb1 = make_uchar4(
				    	clamp(px0.x/4.0f, 0.0f, 255.0f),
				    	clamp(px0.y/4.0f, 0.0f, 255.0f),
		    			clamp(px0.z/4.0f, 0.0f, 255.0f),
					255.0f );
	const uchar4 rgb2 = make_uchar4( 
				    	clamp(px1.x/4.0f, 0.0f, 255.0f),
				    	clamp(px1.y/4.0f, 0.0f, 255.0f),
		    			clamp(px1.z/4.0f, 0.0f, 255.0f),
					255.0f );
						
	unsigned int outputStride = SURFACE_W * 4;	
	unsigned int outputXOffset = (orig_pos_x * 8);
	unsigned int outputOffset = outputXOffset + pos_y * outputStride;				
	output[outputOffset] = rgb1.x;
	output[outputOffset+1] = rgb1.y;
	output[outputOffset+2] = rgb1.z;
	output[outputOffset+3] = rgb1.w;
	output[outputOffset+4] = rgb2.x;
	output[outputOffset+5] = rgb2.y;
	output[outputOffset+6] = rgb2.z;
	output[outputOffset+7] = rgb2.w;
}

GLuint pbo = 0;
GLuint tex = 0;
struct cudaGraphicsResource *cuda_pbo_resource;
uint32_t *src_d, *dst_d;

void drawTexture()
{
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, SURFACE_W, SURFACE_H, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
	glEnable(GL_TEXTURE_2D);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f, 0.0f); glVertex2f(0, 0);
	glTexCoord2f(0.0f, 1.0f); glVertex2f(0, SURFACE_H);
	glTexCoord2f(1.0f, 1.0f); glVertex2f(SURFACE_W, SURFACE_H);
	glTexCoord2f(1.0f, 0.0f); glVertex2f(SURFACE_W, 0);
	glEnd();
	glDisable(GL_TEXTURE_2D);
}

void render()
{
	cudaGraphicsMapResources(1, &cuda_pbo_resource, 0);
	uchar4 *d_out = 0;
	size_t size = 0;
	cudaGraphicsResourceGetMappedPointer((void **)&d_out, &size, cuda_pbo_resource);
	dim3 dimBlock(16, 16);
	dim3 dimGrid(iDivUp(INPUT_WIDTH_BYTES, INPUT_BYTES_PER_SAMPLE * dimBlock.x), iDivUp(SURFACE_H, dimBlock.y));
	convertYuv10ToRGBA<<<dimGrid, dimBlock>>>(reinterpret_cast<unsigned char*>(src_d), reinterpret_cast<unsigned char*>(d_out), INPUT_WIDTH_BYTES, INPUT_STRIDE_BYTES);
	cudaDeviceSynchronize();
	//cudaMemcpy(d_out, src_d, SURFACE_SIZE * 4, cudaMemcpyDeviceToDevice);
	
	cudaGraphicsUnmapResources(1, &cuda_pbo_resource, 0);
}

void display()
{
	render();
	drawTexture();
	glutSwapBuffers();
	glutPostRedisplay();
}

void onKeyboardPress(unsigned char key, int x, int y)
{
	if (key == 'p')
	{
	    printf("onKeyboardPress %d\n", key);
	}
}

void initOpenGL(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(2160, 2160);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Title");
	glutDisplayFunc(display);
	glutKeyboardFunc(onKeyboardPress);
	gluOrtho2D(0, SURFACE_W, SURFACE_H, 0);
}

void initPixelBuffer()
{
	// PBO are a "source" of a texture, its buffer data can be "unpacked" to texture.
	glGenBuffers(1, &pbo);
	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo);
	glBufferData(GL_PIXEL_UNPACK_BUFFER, SURFACE_SIZE * 4, NULL, GL_STREAM_DRAW);
	
	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
	
	cudaGraphicsGLRegisterBuffer(&cuda_pbo_resource, pbo, cudaGraphicsRegisterFlagsWriteDiscard);
}

int main(int argc, char **argv)
{
	cudaError_t ce;
	CUresult cr;
	uint32_t *dst_cpu;
	uint32_t y, x;
	int fd, ret;
	unsigned int flag = 1;
	struct rdma_pin_cuda pin_params_src, pin_params_dst;
	struct rdma_h2c2h_dma dma_params;
	struct rdma_unpin_cuda unpin_params_src, unpin_params_dst;

	if (argc != 1) {
		fprintf(stderr, "usage: rdma-cuda\n");
		return 1;
	}
	
	initOpenGL(argc, argv);
	initPixelBuffer();
	
	fd = open("/dev/xdma1_rdma", O_RDWR);
	if (fd < 0) {
		perror("open() failed");
		return 1;
	}

#ifdef NV_BUILD_DGPU
	ce = cudaMalloc(&src_d, SURFACE_SIZE * sizeof(*src_d));
#else
	ce = cudaHostAlloc(&src_d, SURFACE_SIZE * sizeof(*src_d),
		cudaHostAllocDefault);
#endif
	if (ce != cudaSuccess) {
		fprintf(stderr, "Allocation of src_d failed: %d\n", ce);
		return 1;
	}

	cr = cuPointerSetAttribute(&flag, CU_POINTER_ATTRIBUTE_SYNC_MEMOPS,
		(CUdeviceptr)src_d);
	if (cr != CUDA_SUCCESS) {
		fprintf(stderr, "cuPointerSetAttribute(src_d) failed: %d\n", cr);
		return 1;
	}

	pin_params_src.va = (__u64)src_d;
	pin_params_src.size = SURFACE_SIZE * sizeof(*src_d);
	ret = ioctl(fd, RDMA_IOC_PIN, &pin_params_src);
	if (ret != 0) {
		fprintf(stderr, "ioctl(RDMA_IOC_PIN src) failed: ret=%d errno=%d\n", ret, errno);
		return 1;
	}

#ifdef NV_BUILD_DGPU
	ce = cudaMalloc(&dst_d, SURFACE_SIZE * sizeof(*dst_d));
#else
	ce = cudaHostAlloc(&dst_d, SURFACE_SIZE * sizeof(*dst_d),
		cudaHostAllocDefault);
#endif
	if (ce != cudaSuccess) {
		fprintf(stderr, "Allocation of dst_d failed: %d\n", ce);
		return 1;
	}

	cr = cuPointerSetAttribute(&flag, CU_POINTER_ATTRIBUTE_SYNC_MEMOPS,
		(CUdeviceptr)dst_d);
	if (cr != CUDA_SUCCESS) {
		fprintf(stderr, "cuPointerSetAttribute(dst_d) failed: %d\n", cr);
		return 1;
	}

	pin_params_dst.va = (__u64)dst_d;
	pin_params_dst.size = SURFACE_SIZE * sizeof(*dst_d);
	ret = ioctl(fd, RDMA_IOC_PIN, &pin_params_dst);
	if (ret != 0) {
		fprintf(stderr, "ioctl(RDMA_IOC_PIN dst) failed: ret=%d errno=%d\n", ret, errno);
		return 1;
	}

#if (SURFACE_W < 16) || (SURFACE_H < 16)
#error Grid and block sizes must be shrunk for small surfaces
#endif
#if (SURFACE_W & 15) || (SURFACE_H & 15)
#error Grid and block sizes are not a multiple of the surface size
#endif
	dim3 dimGrid(SURFACE_W / 16, SURFACE_H / 16);
	dim3 dimBlock(16, 16);
	fill_surface<<<dimGrid, dimBlock>>>(src_d, 0);
	fill_surface<<<dimGrid, dimBlock>>>(dst_d, 0xffffffffU);

	ce = cudaDeviceSynchronize();
	if (ce != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize() failed: %d\n", ce);
		return 1;
	}
	
	// Breakout here, as SGDMA not implemented
	glutMainLoop();
	return 1;

	dma_params.src = pin_params_src.handle;
	dma_params.dst = pin_params_dst.handle;
	dma_params.len = SURFACE_SIZE * sizeof(*src_d);
	dma_params.flags = RDMA_H2C2H_DMA_FLAG_SRC_IS_CUDA |
		RDMA_H2C2H_DMA_FLAG_DST_IS_CUDA;
	ret = ioctl(fd, RDMA_IOC_H2C2H_DMA, &dma_params);
	if (ret != 0) {
		fprintf(stderr, "ioctl(DMA) failed: %d\n", ret);
		return 1;
	}

	/*
	 * dGPU on x86 does not allow GPUDirect RDMA on host pinned memory
	 * (cudaMalloc), so we must allocate device memory, and manually copy
	 * it to the host for validation.
	 */
#ifdef NV_BUILD_DGPU
	ce = cudaMallocHost(&dst_cpu, SURFACE_SIZE * sizeof(*dst_cpu), 0);
	if (ce != cudaSuccess) {
		fprintf(stderr, "cudaMallocHost(dst_cpu) failed\n");
		return 1;
	}
	ce = cudaMemcpy(dst_cpu, dst_d, SURFACE_SIZE * sizeof(*dst_cpu), cudaMemcpyDeviceToHost);
	if (ce != cudaSuccess) {
		fprintf(stderr, "cudaMemcpy() failed: %d\n", ce);
		return 1;
	}
#else
	dst_cpu = dst_d;
#endif

	ret = 0;
	for (y = 0; y < SURFACE_H; y++) {
		for (x = 0; x < SURFACE_W; x++) {
			uint32_t expected = DATA(x, y);
			uint32_t offset = OFFSET(x, y);
			uint32_t actual = dst_cpu[offset];
			if (actual != expected) {
				fprintf(stderr,
					"dst[0x%x] is 0x%x not 0x%x\n",
					offset, actual, expected);
				ret = 1;
			}
		}
	}
	if (ret)
		return 1;

#ifdef NV_BUILD_DGPU
	ce = cudaFreeHost(dst_cpu);
	if (ce != cudaSuccess) {
		fprintf(stderr, "cudaFreeHost(dst_cpu) failed: %d\n", ce);
		return 1;
	}
#endif

	unpin_params_dst.handle = pin_params_dst.handle;
	ret = ioctl(fd, RDMA_IOC_UNPIN, &unpin_params_dst);
	if (ret != 0) {
		fprintf(stderr, "ioctl(RDMA_IOC_UNPIN dst) failed: %d\n", ret);
		return 1;
	}

#ifdef NV_BUILD_DGPU
	ce = cudaFree(dst_d);
#else
	ce = cudaFreeHost(dst_d);
#endif
	if (ce != cudaSuccess) {
		fprintf(stderr, "Free of dst_d failed: %d\n", ce);
		return 1;
	}

	unpin_params_src.handle = pin_params_src.handle;
	ret = ioctl(fd, RDMA_IOC_UNPIN, &unpin_params_src);
	if (ret != 0) {
		fprintf(stderr, "ioctl(RDMA_IOC_UNPIN src) failed: %d\n", ret);
		return 1;
	}

#ifdef NV_BUILD_DGPU
	ce = cudaFree(src_d);
#else
	ce = cudaFreeHost(src_d);
#endif
	if (ce != cudaSuccess) {
		fprintf(stderr, "Free of src_d failed: %d\n", ce);
		return 1;
	}

	ret = close(fd);
	if (ret < 0) {
		perror("close() failed");
		return 1;
	}

	return 0;
}


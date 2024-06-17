#include "lfs_port.h"
#include "w25qxx.h"
#include "delay.h"

#define W25Q256_PAGE 256
#define W25Q256_SECTOR 4096
#define W25Q256_32K_BLOCK 32768
#define W25Q256_64K_BLOCK 65536
#define W25Q256_CHIP 33554432

#define W25Q256_SECTOR_NUM 8192

/**
 * \brief 
 * \param c 
 * \param block lfs管理的块序号, block_size中定义的块
 * \param off 在块内的偏移 
 * \param buffer 数据缓冲区
 * \param size 读取的数据长度
 * \return int 返回的欧五代码
 */
int lfs_w25qxx_read(const struct lfs_config *c, lfs_block_t block,
				lfs_off_t off, void *buffer, lfs_size_t size)
{
	SPI_FLASH_BufferRead((uint8_t*)buffer,block*c->block_size + off, size);
	return LFS_ERR_OK;				
}

int lfs_w25qxx_prog(const struct lfs_config *c, lfs_block_t block,
                lfs_off_t off, const void *buffer, lfs_size_t size)
{
    SPI_FLASH_PageWrite((uint8_t*)buffer,block*c->block_size + off, size);
    return LFS_ERR_OK;
}

int lfs_w25qxx_erase(const struct lfs_config *c, lfs_block_t block)
{ 
    SPI_FLASH_SectorErase(block*c->block_size);
    return LFS_ERR_OK;
}

int lfs_w25qxx_sync(const struct lfs_config *c)
{
    return LFS_ERR_OK;
}


uint8_t read_buffer[100];
uint8_t prog_buffer[256];
uint8_t lookahead_buffer[64];

const struct lfs_config disk0_w25q256 = {
    // block device operations
    .read  = lfs_w25qxx_read,
    .prog  = lfs_w25qxx_prog,
    .erase = lfs_w25qxx_erase,
    .sync  = lfs_w25qxx_sync,

    // block device configuration
    //lfs的block指最小操作单元, 和w25的block概念不同
    .read_size = 1, //读取最小可以读取1byte
    .prog_size = W25Q256_PAGE, //单位byte, 指最小操作单元的大小.
    .block_size = W25Q256_SECTOR,
    .block_count = W25Q256_SECTOR_NUM,
    .cache_size = 512, //缓存大小, 是read_size和prog_size的倍数, block_size的因子,单位byte
    .lookahead_size = 32,  //32*8 = 256个blcok
    .block_cycles = 500,
	
	.read_buffer = read_buffer,
	.prog_buffer = prog_buffer,
	.lookahead_buffer = lookahead_buffer,
};

lfs_t disk0;

volatile uint32_t time = 0;

void lfs_init(void)
{
    Flash_Init();
    //可以mount多个disk
    int err = lfs_mount(&disk0, &disk0_w25q256);
    if (err) {
        lfs_format(&disk0, &disk0_w25q256);
        lfs_mount(&disk0, &disk0_w25q256);
    }
}

lfs_t* lfs_disk(uint8_t no)
{
    switch(no){
        case 0:
            return &disk0;
        default:
            return NULL;
    }
}
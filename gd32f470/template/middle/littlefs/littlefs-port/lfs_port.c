#include "lfs_port.h"
#include "w25qxx.h"
#include "delay.h"
#include <stdio.h>

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
 * \param size 读取的数据长度. 最大值为 cache_size 的值. 最小为 read_size 的值.
 * \return int 返回错误代码枚举 lfs_error
 */
int lfs_w25qxx_read(const struct lfs_config *c, lfs_block_t block,
                    lfs_off_t off, void *buffer, lfs_size_t size)
{
    SPI_FLASH_BufferRead((uint8_t *)buffer, block * c->block_size + off, size);
    return LFS_ERR_OK;
}

/**
 * \brief lfs写入w25qxx的操作函数. 内部实现的操作flash的函数要能够支持输入最大值为 size 参数.
 * \param c 配置回调结构体, 可以从中获取块大小之类的配置信息
 * \param block block序号,block_size中定义的块 (比如这里的w25q256对于的就是4K的扇区)
 * \param off 块内的偏移值
 * \param buffer 数据缓冲区
 * \param size 写入的数据长度, 单位byte, 最大为 cache_size 的值. 最小为 prog_size 的值.
 * \return int 返回错误代码枚举 lfs_error
 */
int lfs_w25qxx_prog(const struct lfs_config *c, lfs_block_t block,
                    lfs_off_t off, const void *buffer, lfs_size_t size)
{
    SPI_FLASH_BufferWrite((uint8_t *)buffer, block * c->block_size + off, size);
    return LFS_ERR_OK;
}

/**
 * \brief 块擦除函数
 * \param c 
 * \param block 
 * \return int 返回错误代码枚举 lfs_error
 */
int lfs_w25qxx_erase(const struct lfs_config *c, lfs_block_t block)
{
    SPI_FLASH_SectorErase(block * c->block_size);
    return LFS_ERR_OK;
}

/**
 * \brief 同步存储函数, 一般prog函数已经写入到flash了, 所以可以不实现该函数.
 * \param c 
 * \return int 返回错误代码枚举 lfs_error
 */
int lfs_w25qxx_sync(const struct lfs_config *c)
{
    return LFS_ERR_OK;
}

const struct lfs_config disk0_w25q256 = {
    // block device operations
    .read = lfs_w25qxx_read,
    .prog = lfs_w25qxx_prog,
    .erase = lfs_w25qxx_erase,
    .sync = lfs_w25qxx_sync,

    // block device configuration
    // lfs的block指最小操作单元, 和w25的block概念不同
    .read_size = 256,            // 读取最小可以读取1byte
    .prog_size = W25Q256_PAGE, // 单位byte, 指最小操作单元的大小.
    .block_size = W25Q256_SECTOR,
    .block_count = W25Q256_SECTOR_NUM,
    .cache_size = 256,    // 缓存大小, 是read_size和prog_size的倍数, block_size的因子,单位byte
    .lookahead_size = 32, // 32*8 = 256个blcok
    .block_cycles = 500,
};

lfs_t disk0;


void mlfs_init(void)
{
    Flash_Init();
    // 可以mount多个disk
   int err = lfs_mount(&disk0, &disk0_w25q256);
   if (err)
   {
        lfs_format(&disk0, &disk0_w25q256);
        lfs_mount(&disk0, &disk0_w25q256);
   }
}

lfs_t *mlfs_disk(uint8_t no)
{
    switch (no)
    {
    case 0:
        return &disk0;
    default:
        return NULL;
    }
}

/**
 * \brief 类似于linux下的ls命令. 提供一个路径, 打印该路径下的文件列表(目录和文件).
 */
#ifdef LFS_NO_PRINT
    char mlfs_ls_buf[50][100];
    
#endif
int mlfs_ls(lfs_t *lfs, const char *path)
{
    lfs_dir_t dir;
    int err = lfs_dir_open(lfs, &dir, path);
    if (err)
    {
        return err;
    }

    struct lfs_info info;
#ifdef LFS_NO_PRINT
    int count = 0;
#endif    
    while (true)
    {
        int res = lfs_dir_read(lfs, &dir, &info);
        if (res < 0)
        {
            return res;
        }

        if (res == 0)
        {
            break;
        }
#ifndef LFS_NO_PRINT
        switch (info.type)
        {
        case LFS_TYPE_REG:
            printf("reg ");
            break;
        case LFS_TYPE_DIR:
            printf("dir ");
            break;
        default:
            printf("?   ");
            break;
        }
        printf("%s ", info.name);
        static const char *prefixes[] = {"", "K", "M", "G"};
        for (int i = sizeof(prefixes) / sizeof(prefixes[0]) - 1; i >= 0; i--)
        {
            if (info.size >= (1 << 10 * i) - 1)
            {
                printf("%*u%sB ", 4 - (i != 0), info.size >> 10 * i, prefixes[i]);
                break;
            }
        }

        printf("%s\n", info.name);
#else
        switch (info.type)
        {
        case LFS_TYPE_REG:
            sprintf(&mlfs_ls_buf[count][0], "reg ");
            break;
        case LFS_TYPE_DIR:
            sprintf(&mlfs_ls_buf[count][0], "dir ");
            break;
        default:
            sprintf(&mlfs_ls_buf[count][0], "?   ");
            break;
        }
        sprintf(&mlfs_ls_buf[count][4], "%s", info.name);
        int size = strlen(info.name) + 4;
        static const char *prefixes[] = {"", "K", "M", "G"};
        for (int i = sizeof(prefixes) / sizeof(prefixes[0]) - 1; i >= 0; i--)
        {
            if (info.size >= (1 << 10 * i) - 1)
            {
                sprintf(&mlfs_ls_buf[count][size], "%*u%sB ", 4 - (i != 0), info.size >> 10 * i, prefixes[i]);
                break;
            }
        }
        count++;
#endif
    }

    err = lfs_dir_close(lfs, &dir);
    if (err)
    {
        return err;
    }

    return 0;
}
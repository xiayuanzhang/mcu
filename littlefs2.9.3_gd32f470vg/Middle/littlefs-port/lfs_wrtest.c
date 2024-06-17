#include "lfs_wrtest.h"
#include "delay.h"


uint16_t test_buffer[128][512];
void buffer_set(void)
{
    for(int i =0;i<128;i++)
    {
        for(int j = 0;j<512;j++)
        {
            test_buffer[i][j] = (i<<8) + j%0xff;
        }
    }

}

void buffer_clear(void)
{
    for(int i =0;i<128;i++)
    {
        for(int j = 0;j<512;j++)
        {
            test_buffer[i][j] = 0;
        }
    }

}

// return 1 if buffer is correct
// return 0 if buffer is incorrect
int buffer_check(uint32_t size)
{
    size = size / 2;
    int r128 = size / 512;
    int r512 = size % 512;
    for(int i =0;i<r128;i++)
    {
        for(int j = 0;j<r512;j++)
        {
            if(test_buffer[i][j] != (i<<8) + j%0xff)
            {
                return 0xee;
            }
        }
    }
    return 0x88;
}


//128B x不使用
//128B result
//256B x
//256B result
//... 对齐time_result
uint8_t result[100];

//128B write
//128B read
//128B delete
//256B write
//256B read
//2048B write
//2048B read
//4096B write
//4096B read
//8192B write
//8192B read
//128Kb write
//128Kb read
uint32_t time_reulst[100];

void result_reset(void)
{
    for(int i = 0;i<100;i++)
    {
        result[i] = 0;
        time_reulst[i] = 0;
    }
}

void lfs_wrtest(void)
{
    lfs_file_t file;

    result_reset();

    buffer_set();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_128B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    time_reulst[0] = time_test_getus();

    buffer_clear();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_128B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    time_reulst[1] = time_test_getus();

    result[2] = buffer_check(128);

	
    buffer_set();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_256B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 256);
    lfs_file_close(&disk0, &file);
    time_reulst[2] = time_test_getus();

    buffer_clear();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_256B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&disk0, &file, test_buffer, 256);
    lfs_file_close(&disk0, &file);
    time_reulst[3] = time_test_getus();

    result[4] = buffer_check(256);
}
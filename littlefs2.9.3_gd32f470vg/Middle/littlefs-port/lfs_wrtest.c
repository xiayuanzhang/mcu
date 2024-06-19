#include "lfs_wrtest.h"
#include "delay.h"

uint16_t test_buffer[128][512];
void buffer_set(void)
{
    for (int i = 0; i < 128; i++)
    {
        for (int j = 0; j < 512; j++)
        {
            test_buffer[i][j] = (i << 8) + j % 0xff;
        }
    }
}

void buffer_clear(void)
{
    for (int i = 0; i < 128; i++)
    {
        for (int j = 0; j < 512; j++)
        {
            test_buffer[i][j] = 0xff;
        }
    }
}

// return 1 if buffer is correct
// return 0 if buffer is incorrect
int buffer_check(uint32_t size)
{
    size = size / 2 - 1;
    int r128 = size / 512;
    int r512 = size % 512;
    for (int i = 0; i <= r128; i++)
    {
        for (int j = 0; j <= r512; j++)
        {
            if (test_buffer[i][j] != (i << 8) + j % 0xff)
            {
                return 0xee;
            }
        }
    }
    return 0x88;
}

// 128B x不使用
// 128B result
// 256B x
// 256B result
//... 对齐time_result
uint8_t result[20];

// 128B write
// 128B read
// 128B delete
// 256B write
// 256B read
// 2048B write
// 2048B read
// 4096B write
// 4096B read
// 8192B write
// 8192B read
// 128Kb write
// 128Kb read
float time_reulst[40];
float speed_kb_s[40];

void result_reset(void)
{
    for (int i = 0; i < 100; i++)
    {
        result[i] = 0;
        time_reulst[i] = 0;
    }
}

// TEST_MODE = 0 格式化后空白读写测试
// TEST_MODE = 1 覆盖式的读写测试
// TEST_MODE = 2 追加式的读写测试
// TEST_MODE = 3 同等大小的文件,创建多个文件测试文件系统多写入的影响
#define TEST_MODE 2

#if TEST_MODE == 0
void mlfs_wrtest(void)
{

    lfs_file_t file;

    result_reset();

    // 强制格式化
    lfs_format(&disk0, &disk0_w25q256);
    lfs_mount(&disk0, &disk0_w25q256);

    buffer_set();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_128B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    time_reulst[0] = time_test_gets();
    buffer_clear();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_128B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    time_reulst[1] = time_test_gets();
    result[0] = buffer_check(128);

    // 强制格式化
    lfs_format(&disk0, &disk0_w25q256);
    lfs_mount(&disk0, &disk0_w25q256);

    buffer_set();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_256B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 256);
    lfs_file_close(&disk0, &file);
    time_reulst[2] = time_test_gets();
    buffer_clear();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_256B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&disk0, &file, test_buffer, 256);
    lfs_file_close(&disk0, &file);
    time_reulst[3] = time_test_gets();
    result[1] = buffer_check(256);

    // 强制格式化
    lfs_format(&disk0, &disk0_w25q256);
    lfs_mount(&disk0, &disk0_w25q256);
    delay_ms(100);
    buffer_set();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_700B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 700);
    lfs_file_close(&disk0, &file);
    time_reulst[4] = time_test_gets();
    buffer_clear();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_700B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&disk0, &file, test_buffer, 700);
    lfs_file_close(&disk0, &file);
    time_reulst[5] = time_test_gets();
    result[2] = buffer_check(700);

    // 强制格式化
    lfs_format(&disk0, &disk0_w25q256);
    lfs_mount(&disk0, &disk0_w25q256);
    buffer_set();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_2048B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 2048);
    lfs_file_close(&disk0, &file);
    time_reulst[6] = time_test_gets();
    buffer_clear();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_2048B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&disk0, &file, test_buffer, 2048);
    lfs_file_close(&disk0, &file);
    time_reulst[7] = time_test_gets();
    result[3] = buffer_check(2048);

    // 强制格式化
    lfs_format(&disk0, &disk0_w25q256);
    lfs_mount(&disk0, &disk0_w25q256);

    buffer_set();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_8192B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 8192);
    lfs_file_close(&disk0, &file);
    time_reulst[8] = time_test_gets();
    buffer_clear();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_8192B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&disk0, &file, test_buffer, 8192);
    lfs_file_close(&disk0, &file);
    time_reulst[9] = time_test_gets();
    result[4] = buffer_check(8192);

    // 强制格式化
    lfs_format(&disk0, &disk0_w25q256);
    lfs_mount(&disk0, &disk0_w25q256);

    buffer_set();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_128K", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 128 * 1024);
    lfs_file_close(&disk0, &file);
    time_reulst[10] = time_test_gets();
    buffer_clear();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_128K", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&disk0, &file, test_buffer, 128 * 1024);
    lfs_file_close(&disk0, &file);
    time_reulst[11] = time_test_gets();
    result[5] = buffer_check(128 * 1024);
	
	
	volatile int e1=0,e2=0;
	// 强制格式化
    e1 = lfs_format(&disk0, &disk0_w25q256);
    e2 = lfs_mount(&disk0, &disk0_w25q256);

    buffer_set();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_110K", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 111 * 1024);
    lfs_file_close(&disk0, &file);
    time_reulst[12] = time_test_gets();
    buffer_clear();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_110K", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&disk0, &file, test_buffer, 111 * 1024);
    lfs_file_close(&disk0, &file);
    time_reulst[13] = time_test_gets();
    result[6] = buffer_check(111 * 1024);

    // kb/s
    speed_kb_s[0] = 128 / time_reulst[0] / 1024;          //  128B write
    speed_kb_s[1] = 128 / time_reulst[1] / 1024;          //  128B read
    speed_kb_s[2] = 256 / time_reulst[2] / 1024;          //  256B write
    speed_kb_s[3] = 256 / time_reulst[3] / 1024;          //  256B read
    speed_kb_s[4] = 700 / time_reulst[4] / 1024;          //  700B write
    speed_kb_s[5] = 700 / time_reulst[5] / 1024;          //  700B read
    speed_kb_s[6] = 2048 / time_reulst[6] / 1024;         //  2048B write
    speed_kb_s[7] = 2048 / time_reulst[7] / 1024;         //  2048B read
    speed_kb_s[8] = 8192 / time_reulst[8] / 1024;         //  8192B write
    speed_kb_s[9] = 8192 / time_reulst[9] / 1024;         //  8192B read
    speed_kb_s[10] = 128 * 1024 / time_reulst[10] / 1024; //  128Kb write
    speed_kb_s[11] = 128 * 1024 / time_reulst[11] / 1024; //  128Kb read
	speed_kb_s[12] = 111 * 1024 / time_reulst[12] / 1024; //  111Kb write
    speed_kb_s[13] = 111 * 1024 / time_reulst[13] / 1024; //  111Kb read
    mlfs_ls(&disk0, "/");
}

#elif TEST_MODE == 1
void mlfs_wrtest(void)
{

    lfs_file_t file;

    result_reset();

    // 强制格式化
    lfs_format(&disk0, &disk0_w25q256);
    lfs_mount(&disk0, &disk0_w25q256);
    mlfs_ls(&disk0, "/");
    lfs_file_open(&disk0, &file, "init", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    // 预先写入文件
    lfs_file_open(&disk0, &file, "test_128B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);

    lfs_file_open(&disk0, &file, "test_256B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 256);
    lfs_file_close(&disk0, &file);

    lfs_file_open(&disk0, &file, "test_700B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 700);
    lfs_file_close(&disk0, &file);

    lfs_file_open(&disk0, &file, "test_2048B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 2048);
    lfs_file_close(&disk0, &file);

    lfs_file_open(&disk0, &file, "test_8192B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 8192);
    lfs_file_close(&disk0, &file);

    lfs_file_open(&disk0, &file, "test_128K", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 128 * 1024);
    lfs_file_close(&disk0, &file);

	lfs_file_open(&disk0, &file, "test_128B", LFS_O_RDWR | LFS_O_CREAT);
        lfs_file_write(&disk0, &file, test_buffer, 128);
        lfs_file_close(&disk0, &file);

        buffer_set();
        time_test_run();
        lfs_file_open(&disk0, &file, "test_128B", LFS_O_RDWR | LFS_O_CREAT);
        lfs_file_write(&disk0, &file, test_buffer, 128);
        lfs_file_close(&disk0, &file);
        time_reulst[0] = time_test_gets();
        buffer_clear();
        time_test_run();
        lfs_file_open(&disk0, &file, "test_128B", LFS_O_RDWR | LFS_O_CREAT);
        lfs_file_read(&disk0, &file, test_buffer, 128);
        lfs_file_close(&disk0, &file);
        time_reulst[1] = time_test_gets();
        result[0] = buffer_check(128);

        buffer_set();
        time_test_run();
        lfs_file_open(&disk0, &file, "test_256B", LFS_O_RDWR | LFS_O_CREAT);
        lfs_file_write(&disk0, &file, test_buffer, 256);
        lfs_file_close(&disk0, &file);
        time_reulst[2] = time_test_gets();
        buffer_clear();
        time_test_run();
        lfs_file_open(&disk0, &file, "test_256B", LFS_O_RDWR | LFS_O_CREAT);
        lfs_file_read(&disk0, &file, test_buffer, 256);
        lfs_file_close(&disk0, &file);
        time_reulst[3] = time_test_gets();
        result[1] = buffer_check(256);

        buffer_set();
        time_test_run();
        lfs_file_open(&disk0, &file, "test_700B", LFS_O_RDWR | LFS_O_CREAT);
        lfs_file_write(&disk0, &file, test_buffer, 700);
        lfs_file_close(&disk0, &file);
        time_reulst[4] = time_test_gets();
        buffer_clear();
        time_test_run();
        lfs_file_open(&disk0, &file, "test_700B", LFS_O_RDWR | LFS_O_CREAT);
        lfs_file_read(&disk0, &file, test_buffer, 700);
        lfs_file_close(&disk0, &file);
        time_reulst[5] = time_test_gets();
        result[2] = buffer_check(700);

        buffer_set();
        time_test_run();
        lfs_file_open(&disk0, &file, "test_2048B", LFS_O_RDWR | LFS_O_CREAT);
        lfs_file_write(&disk0, &file, test_buffer, 2048);
        lfs_file_close(&disk0, &file);
        time_reulst[6] = time_test_gets();
        buffer_clear();
        time_test_run();
        lfs_file_open(&disk0, &file, "test_2048B", LFS_O_RDWR | LFS_O_CREAT);
        lfs_file_read(&disk0, &file, test_buffer, 2048);
        lfs_file_close(&disk0, &file);
        time_reulst[7] = time_test_gets();
        result[3] = buffer_check(2048);

        buffer_set();
        time_test_run();
        lfs_file_open(&disk0, &file, "test_8192B", LFS_O_RDWR | LFS_O_CREAT);
        lfs_file_write(&disk0, &file, test_buffer, 8192);
        lfs_file_close(&disk0, &file);
        time_reulst[8] = time_test_gets();
        buffer_clear();
        time_test_run();
        lfs_file_open(&disk0, &file, "test_8192B", LFS_O_RDWR | LFS_O_CREAT);
        lfs_file_read(&disk0, &file, test_buffer, 8192);
        lfs_file_close(&disk0, &file);
        time_reulst[9] = time_test_gets();
        result[4] = buffer_check(8192);

        buffer_set();
        time_test_run();
        lfs_file_open(&disk0, &file, "test_128K", LFS_O_RDWR | LFS_O_CREAT);
        lfs_file_write(&disk0, &file, test_buffer, 128 * 1024);
        lfs_file_close(&disk0, &file);
        time_reulst[10] = time_test_gets();
        buffer_clear();
        time_test_run();
        lfs_file_open(&disk0, &file, "test_128K", LFS_O_RDWR | LFS_O_CREAT);
        lfs_file_read(&disk0, &file, test_buffer, 128 * 1024);
        lfs_file_close(&disk0, &file);
        time_reulst[11] = time_test_gets();
        result[5] = buffer_check(128 * 1024);

    // kb/s
    speed_kb_s[0] = 128 / time_reulst[0] / 1024;          //  128B write
    speed_kb_s[1] = 128 / time_reulst[1] / 1024;          //  128B read
    speed_kb_s[2] = 256 / time_reulst[2] / 1024;          //  256B write
    speed_kb_s[3] = 256 / time_reulst[3] / 1024;          //  256B read
    speed_kb_s[4] = 700 / time_reulst[4] / 1024;          //  700B write
    speed_kb_s[5] = 700 / time_reulst[5] / 1024;          //  700B read
    speed_kb_s[6] = 2048 / time_reulst[6] / 1024;         //  2048B write
    speed_kb_s[7] = 2048 / time_reulst[7] / 1024;         //  2048B read
    speed_kb_s[8] = 8192 / time_reulst[8] / 1024;         //  8192B write
    speed_kb_s[9] = 8192 / time_reulst[9] / 1024;         //  8192B read
    speed_kb_s[10] = 128 * 1024 / time_reulst[10] / 1024; //  128Kb write
    speed_kb_s[11] = 128 * 1024 / time_reulst[11] / 1024; //  128Kb read
    mlfs_ls(&disk0, "/");
}

#elif TEST_MODE == 2

void mlfs_wrtest(void)
{
    lfs_file_t file;

    result_reset();

    // 强制格式化
    lfs_format(&disk0, &disk0_w25q256);
    lfs_mount(&disk0, &disk0_w25q256);
    mlfs_ls(&disk0, "/");
    lfs_file_open(&disk0, &file, "init", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    // 预先写入文件
    lfs_file_open(&disk0, &file, "test_128B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);

    lfs_file_open(&disk0, &file, "test_256B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 256);
    lfs_file_close(&disk0, &file);

    lfs_file_open(&disk0, &file, "test_700B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 700);
    lfs_file_close(&disk0, &file);

    lfs_file_open(&disk0, &file, "test_2048B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 2048);
    lfs_file_close(&disk0, &file);

    lfs_file_open(&disk0, &file, "test_8192B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 8192);
    lfs_file_close(&disk0, &file);

    lfs_file_open(&disk0, &file, "test_128K", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 128 * 1024);
    lfs_file_close(&disk0, &file);

	lfs_file_open(&disk0, &file, "test_128B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
	
    buffer_set();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_128B", LFS_O_APPEND | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    time_reulst[0] = time_test_gets();
    buffer_clear();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_128B", LFS_O_APPEND | LFS_O_CREAT);
    lfs_file_read(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    time_reulst[1] = time_test_gets();
    result[0] = buffer_check(128);

    buffer_set();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_256B", LFS_O_APPEND | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 256);
    lfs_file_close(&disk0, &file);
    time_reulst[2] = time_test_gets();
    buffer_clear();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_256B", LFS_O_APPEND | LFS_O_CREAT);
    lfs_file_read(&disk0, &file, test_buffer, 256);
    lfs_file_close(&disk0, &file);
    time_reulst[3] = time_test_gets();
    result[1] = buffer_check(256);

    buffer_set();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_700B", LFS_O_APPEND | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 700);
    lfs_file_close(&disk0, &file);
    time_reulst[4] = time_test_gets();
    buffer_clear();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_700B", LFS_O_APPEND | LFS_O_CREAT);
    lfs_file_read(&disk0, &file, test_buffer, 700);
    lfs_file_close(&disk0, &file);
    time_reulst[5] = time_test_gets();
    result[2] = buffer_check(700);

    buffer_set();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_2048B", LFS_O_APPEND | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 2048);
    lfs_file_close(&disk0, &file);
    time_reulst[6] = time_test_gets();
    buffer_clear();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_2048B", LFS_O_APPEND | LFS_O_CREAT);
    lfs_file_read(&disk0, &file, test_buffer, 2048);
    lfs_file_close(&disk0, &file);
    time_reulst[7] = time_test_gets();
    result[3] = buffer_check(2048);

    buffer_set();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_8192B", LFS_O_APPEND | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 8192);
    lfs_file_close(&disk0, &file);
    time_reulst[8] = time_test_gets();
    buffer_clear();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_8192B", LFS_O_APPEND | LFS_O_CREAT);
    lfs_file_read(&disk0, &file, test_buffer, 8192);
    lfs_file_close(&disk0, &file);
    time_reulst[9] = time_test_gets();
    result[4] = buffer_check(8192);

    buffer_set();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_128K", LFS_O_APPEND | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 128 * 1024);
    lfs_file_close(&disk0, &file);
    time_reulst[10] = time_test_gets();
    buffer_clear();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_128K", LFS_O_APPEND | LFS_O_CREAT);
    lfs_file_read(&disk0, &file, test_buffer, 128 * 1024);
    lfs_file_close(&disk0, &file);
    time_reulst[11] = time_test_gets();
    result[5] = buffer_check(128 * 1024);

    // kb/s
    speed_kb_s[0] = 128 / time_reulst[0] / 1024;          //  128B write
    speed_kb_s[1] = 128 / time_reulst[1] / 1024;          //  128B read
    speed_kb_s[2] = 256 / time_reulst[2] / 1024;          //  256B write
    speed_kb_s[3] = 256 / time_reulst[3] / 1024;          //  256B read
    speed_kb_s[4] = 700 / time_reulst[4] / 1024;          //  700B write
    speed_kb_s[5] = 700 / time_reulst[5] / 1024;          //  700B read
    speed_kb_s[6] = 2048 / time_reulst[6] / 1024;         //  2048B write
    speed_kb_s[7] = 2048 / time_reulst[7] / 1024;         //  2048B read
    speed_kb_s[8] = 8192 / time_reulst[8] / 1024;         //  8192B write
    speed_kb_s[9] = 8192 / time_reulst[9] / 1024;         //  8192B read
    speed_kb_s[10] = 128 * 1024 / time_reulst[10] / 1024; //  128Kb write
    speed_kb_s[11] = 128 * 1024 / time_reulst[11] / 1024; //  128Kb read
    mlfs_ls(&disk0, "/");
}
#elif TEST_MODE == 3
void mlfs_wrtest(void)
{
    lfs_file_t file;

    result_reset();

    // 强制格式化
    lfs_format(&disk0, &disk0_w25q256);
    lfs_mount(&disk0, &disk0_w25q256);

    buffer_set();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_128B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    time_reulst[0] = time_test_gets();
    buffer_clear();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_128B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    time_reulst[1] = time_test_gets();
    result[0] = buffer_check(128);


    buffer_set();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_256B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    time_reulst[2] = time_test_gets();
    buffer_clear();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_256B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    time_reulst[3] = time_test_gets();
    result[1] = buffer_check(128);

    delay_ms(100);
    buffer_set();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_700B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    time_reulst[4] = time_test_gets();
    buffer_clear();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_700B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    time_reulst[5] = time_test_gets();
    result[2] = buffer_check(128);

    buffer_set();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_2048B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    time_reulst[6] = time_test_gets();
    buffer_clear();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_2048B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    time_reulst[7] = time_test_gets();
    result[3] = buffer_check(128);

    buffer_set();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_8192B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    time_reulst[8] = time_test_gets();
    buffer_clear();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_8192B", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    time_reulst[9] = time_test_gets();
    result[4] = buffer_check(128);


    buffer_set();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_128K", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    time_reulst[10] = time_test_gets();
    buffer_clear();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_128K", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    time_reulst[11] = time_test_gets();
    result[5] = buffer_check(128);

    buffer_set();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_1281K", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    time_reulst[12] = time_test_gets();
    buffer_clear();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_1281K", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    time_reulst[13] = time_test_gets();
    result[6] = buffer_check(128);


    time_test_run();
    lfs_file_open(&disk0, &file, "test_12812K", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    time_reulst[14] = time_test_gets();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_12812K", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    time_reulst[15] = time_test_gets();

    time_test_run();
    lfs_file_open(&disk0, &file, "test_128121K", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    time_reulst[16] = time_test_gets();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_128121K", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    time_reulst[17] = time_test_gets();

    time_test_run();
    lfs_file_open(&disk0, &file, "test_1281211K", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    time_reulst[18] = time_test_gets();
    time_test_run();
    lfs_file_open(&disk0, &file, "test_1281211K", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&disk0, &file, test_buffer, 128);
    lfs_file_close(&disk0, &file);
    time_reulst[19] = time_test_gets();
	

    // kb/s
    speed_kb_s[0] = 128 / time_reulst[0] / 1024;          //  128B write
    speed_kb_s[1] = 128 / time_reulst[1] / 1024;          //  128B read
    speed_kb_s[2] = 128 / time_reulst[2] / 1024;          //  256B write
    speed_kb_s[3] = 128 / time_reulst[3] / 1024;          //  256B read
    speed_kb_s[4] = 128 / time_reulst[4] / 1024;          //  700B write
    speed_kb_s[5] = 128 / time_reulst[5] / 1024;          //  700B read
    speed_kb_s[6] = 128 / time_reulst[6] / 1024;         //  2048B write
    speed_kb_s[7] = 128 / time_reulst[7] / 1024;         //  2048B read
    speed_kb_s[8] = 128 / time_reulst[8] / 1024;         //  8192B write
    speed_kb_s[9] = 128 / time_reulst[9] / 1024;         //  8192B read
    speed_kb_s[10] = 128 / time_reulst[10] / 1024; //  128Kb write
    speed_kb_s[11] = 128 / time_reulst[11] / 1024; //  128Kb read
    speed_kb_s[12] = 128 / time_reulst[12] / 1024; //  128Kb write
    speed_kb_s[13] = 128 / time_reulst[13] / 1024; //  128Kb read
    speed_kb_s[14] = 128 / time_reulst[14] / 1024; //  128Kb write
    speed_kb_s[15] = 128 / time_reulst[15] / 1024; //  128Kb read
    speed_kb_s[16] = 128 / time_reulst[16] / 1024; //  128Kb write
    speed_kb_s[17] = 128 / time_reulst[17] / 1024; //  128Kb read
    speed_kb_s[18] = 128 / time_reulst[18] / 1024; //  128Kb write
    speed_kb_s[19] = 128 / time_reulst[19] / 1024; //  128Kb read
    mlfs_ls(&disk0, "/");
}


#endif



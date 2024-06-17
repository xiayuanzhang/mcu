#include "led.h"
#include "delay.h"
#include "w25qxx.h"
#include "lfs_port.h"


int main(void)
{
	led_init();
	dwt_init();

    lfs_init();

	while(1)
	{
		LED_MCU_TOG;
		delay_ms(500);
		
		lfs_file_t file;
		uint32_t boot_count = 0;
		
		lfs_file_open(&disk0, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
		lfs_file_read(&disk0, &file, &boot_count, sizeof(boot_count));

		// update boot count
		boot_count += 1;
		lfs_file_rewind(&disk0, &file);
		lfs_file_write(&disk0, &file, &boot_count, sizeof(boot_count));

		// remember the storage is not updated until the file is closed successfully
		lfs_file_close(&disk0, &file);
	}
}


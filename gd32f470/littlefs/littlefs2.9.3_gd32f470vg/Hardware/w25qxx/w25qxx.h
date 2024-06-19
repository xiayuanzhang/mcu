#ifndef __W25QXX_H
#define __W25QXX_H

#include "gd32f4xx.h"

/* Private typedef -----------------------------------------------------------*/
#define SPI_FLASH_EndAddr       0x2000000
//#define SPI_FLASH_PageSize      4096
#define SPI_FLASH_PageSize      256
#define SPI_FLASH_PerWritePageSize      256
#define SPI_FLASH_EraseSectorSize       4096
#define SPI_FLASH_Erase64kBlockSize     65536
/* Private define ------------------------------------------------------------*/


#define SPI_FLASH_CS_LOW()       gpio_bit_reset(GPIOA, GPIO_PIN_4)
#define SPI_FLASH_CS_HIGH()      gpio_bit_set(GPIOA, GPIO_PIN_4)


void Flash_Init(void);
void Flash_Test(void);
void SPI_FLASH_SectorErase(uint32_t SectorAddr);
void SPI_FLASH_64kBlockErase(uint32_t SectorAddr);
void SPI_FLASH_BulkErase(void);
void SPI_FLASH_PageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void SPI_FLASH_BufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void SPI_FLASH_BufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
uint32_t SPI_FLASH_ReadID(void);
uint32_t SPI_FLASH_ReadDeviceID(void);
void SPI_FLASH_StartReadSequence(uint32_t ReadAddr);
void SPI_Flash_PowerDown(void);
void SPI_Flash_WAKEUP(void);


uint8_t SPI_FLASH_ReadByte(void);
uint8_t SPI_FLASH_SendByte(uint8_t byte);
uint16_t SPI_FLASH_SendHalfWord(uint16_t HalfWord);
void SPI_FLASH_WriteEnable(void);
void SPI_FLASH_WaitForWriteEnd(void);

#endif /* __SPI_FLASH_H */


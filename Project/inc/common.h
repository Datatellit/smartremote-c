#ifndef _COMMON_H
#define _COMMON_H
#include "stm8l15x.h"
#include "publicDefine.h"

#define WWDG_COUNTER                    0x7f
#define WWDG_WINDOW                     0x77

// Starting Flash block number of backup config
#define BACKUP_CONFIG_BLOCK_NUM         2
#define BACKUP_CONFIG_ADDRESS           (FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS + BACKUP_CONFIG_BLOCK_NUM * FLASH_BLOCK_SIZE)
#define STATUS_DATA_NUM                 4
#define STATUS_DATA_ADDRESS             (FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS + STATUS_DATA_NUM * FLASH_BLOCK_SIZE)

bool isIdentityEmpty(const UC *pId, UC nLen);
bool isIdentityEqual(const UC *pId1, const UC *pId2, UC nLen);
uint8_t *Read_UniqueID(uint8_t *UniqueID, uint16_t Length);

void wwdg_init();
void feed_wwdg(void);
int8_t wait_flashflag_status(uint8_t flag,uint8_t status);
void Flash_ReadBuf(uint32_t Address, uint8_t *Buffer, uint16_t Length);
bool Flash_WriteBuf(uint32_t Address, uint8_t *Buffer, uint16_t Length);
bool Flash_WriteDataBlock(uint16_t nStartBlock, uint8_t *Buffer, uint16_t Length);

#endif /* _COMMON_H */
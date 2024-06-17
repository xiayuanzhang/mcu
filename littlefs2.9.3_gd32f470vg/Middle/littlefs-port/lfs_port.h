#ifndef _LFS_PORT_H_
#define _LFS_PORT_H_

#include "lfs.h"

extern lfs_t disk0;


void lfs_init(void);

lfs_t* lfs_disk(uint8_t no);

#endif // _LFS_PORT_H_



#ifndef _LFS_PORT_H_
#define _LFS_PORT_H_

#include "lfs.h"

extern lfs_t disk0;
extern const struct lfs_config disk0_w25q256;

void mlfs_init(void);

lfs_t* mlfs_disk(uint8_t no);

int mlfs_ls(lfs_t *lfs, const char *path);

#endif // _LFS_PORT_H_



/**
 *    ||        ____  _ __  ______
 * +------+    / __ )(_) /_/  ____/_________ ____  ____
 * | 0xBC |   / __ /  / __/ /    / ___/ __ `/_  / / _  \
 * +------+  / /_/ / / /_/ /___ / /  / /_/ / / /_/   __/
 *  ||  ||  /_____/_/\__/\____//_/   \__,_/ /___/ \___/
 *
 * CrazyLoader firmware
 *
 * Copyright (C) 2011-2013 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * squeue.h - Simple non-reentrant queue implementation
 * The queue is single sized and can contains only CRTP packets
 */

#ifndef __SQUEUE_H__
#define __SQUEUE_H__

#define SQUEUE_SIZE 8

#include "crtp.h"

#define SQ_OK    0
#define SQ_ERROR -1

typedef struct {
  int head;
  int tail;
  CRTPPacket data[SQUEUE_SIZE];
} squeue_t;

void sqInit(squeue_t *q);

int sqPut(squeue_t *q, CRTPPacket *p);
int sqGet(squeue_t *q, CRTPPacket *p);

#endif /* __SQUEUE_H__ */


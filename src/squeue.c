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
 * Simple non-reentrant queue implementation
 */

#include <string.h>

#include "squeue.h"
#include "crtp.h"


void sqInit(squeue_t *q)
{
  q->head = 0;
  q->tail = 0;
}

int sqPut(squeue_t *q, CRTPPacket *p)
{
  //Check if the queue is full
  if (((q->head+1)%SQUEUE_SIZE) == q->tail)
    return SQ_ERROR;
  
  //Add the new item
  memcpy(&q->data[q->head], p, sizeof(CRTPPacket));
  q->head = (q->head+1)%SQUEUE_SIZE;
  
  return SQ_OK;
}

int sqGet(squeue_t *q, CRTPPacket *p)
{
  //Check if the queue contains at least one element
  if (q->head==q->tail)
    return SQ_ERROR;
  
  //Get one element of the queue
  memcpy(p, &q->data[q->tail], sizeof(CRTPPacket));
  q->tail = (q->tail+1)%SQUEUE_SIZE;
  
  return SQ_OK;
}


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
 * crtp.h - CRTP stack
 */

#ifndef CRTP_H_
#define CRTP_H_

#define CRTP_MAX_DATA_SIZE 31

#define TASK_MASK 0xF0
#define NBR_MASK 0x03
#define CRTP_GET_NBR(x) (x & NBR_MASK)
#define CRTP_GET_TASK(x) ((x & TASK_MASK) >> 4)
#define CRTP_PORT(prio, task, nbr) ( task << 4 | 0x3 << 2 | nbr)

typedef struct _CRTPPacket
{
  uint8_t size;
  uint8_t port;
  uint8_t data[CRTP_MAX_DATA_SIZE];
} __attribute__((packed)) CRTPPacket;

#endif /*CRTP_H_*/

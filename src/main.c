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
 * Crazyflie Radio Bootloader
 * This program is clocked around the radio activity
 * Does NOT work with the stm32-XL (does not take care about the banks)
 */
#include <string.h>

/*Project includes*/
#include "config.h"

#include "led.h"
#include "radio.h"
#include "version.h"
#include "cpuid.h"

#include "loaderCommands.h"

/* Cortex Include */
#include "core_cm3.h"

/*ST includes */
#include "stm32f10x.h"
#include "stm32f10x_bkp.h"
#include "stm32f10x_pwr.h"


/* Private functions */
static void prvClockInit(void);

/* Private variables */
static char buffer[BUFFER_PAGES*PAGE_SIZE]; //Memory buffer, writable by the PC


int main() {
  int i;
  int ctr = 0;
  int led = 0;
  CRTPPacket pk;
  int resetMode = 0;
  int cloaderComm = 0;  //Flag that indicate if there had cloader communication

  //Initialise the clock frequency and bus/flash clock
  prvClockInit();
  //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  //Initialise the modules
  ledInit();
  radioInit();

  //Reset detector
  ledSetRed(1);
  ledSetGreen(1);
  for(i=0;i<640000;i++) asm( "  nop" );
  ledSetRed(0);
  ledSetGreen(0);

  //Unlock the flash
  FLASH_Unlock();
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
  //Reset the boot flag to always reboot on the bootloader by default
  PWR_BackupAccessCmd(ENABLE);
  BKP_WriteBackupRegister(BKP_DR1, 0);
  PWR_BackupAccessCmd(DISABLE);
  
  //Main Loop
  while(1) {
    //Wait for a radio interruption
    while( !radioIsIrq() ) 
    {
      if (ctr++>640000)
      {
        //If no CLoader communication since the begining, launch the firmware
        //Do not reboot if the flash is empty ...
        if (!cloaderComm && (((uint32_t*)FLASH_BASE)[10240] != 0xFFFFFFFF))
        {
          //Firmware reset
          //Set the firmware boot flag
          PWR_BackupAccessCmd(ENABLE);
          BKP_WriteBackupRegister(BKP_DR1, 1);
          
          //Soft Reset
          SCB->AIRCR = 0x05fa0604;
        }
        
        ctr=0;
        led = !led;
        ledSetGreen(led);
      }
    }
    //Run the radio interruption handler
    radioIsr();
    
    //Verify if there is something to do ...
    if (radioReceivePacket(&pk))
    {
      //Test if the packet is an echo request
      if((CRTP_GET_TASK(pk.port)==15) && (CRTP_GET_NBR(pk.port)==0))
      {
        radioSendPacket(&pk);
      }
      //Look for a magic packet that math this CPU ID
      //else if ((pk.port == 0xFF) && (pk.data[0]==0xFF) && ((pk.data[1]&0xF0)==0xF0)
      //     && pk.size>12 && !memcmp(&pk.data[2], cpuidGetId(), 12)) 
      //FIXME: Disabled CPU id detection!
      else if ((pk.port == 0xFF) && (pk.data[0]==0xFF) && ((pk.data[1]&0xF0)==0xF0)
           && pk.size>12) 
      {
        //If ack
        if (pk.data[1] == 0xF0)
        {
          if (resetMode==0xFF)
          {
            //Firmware reset
            //Set the firmware boot flag
            PWR_BackupAccessCmd(ENABLE);
            BKP_WriteBackupRegister(BKP_DR1, 1);
            
            //Soft Reset
            SCB->AIRCR = 0x05fa0604;
          }
          else if (resetMode==0xFE)
          {
            //Bootloader reset (Soft reset will end to the bootloader anyway)
            SCB->AIRCR = 0x05fa0604;
          }
        } else {
          //If reset command, set the resetMode and send back the command
          resetMode = pk.data[1];
          radioSendPacket(&pk);
        }
      }
      //Decode and execute bootloader commands
      else if ((pk.size>1) && (pk.port == 0xFF) && (pk.data[0]==0xFF))
      {
        //Set the communication flag
        cloaderComm=1;
        
        if (pk.data[1] == CMD_GET_INFO)
        {
          GetInfoReturns_t * info = (GetInfoReturns_t *)&pk.data[2];
          
          info->pageSize = PAGE_SIZE;
          info->nBuffPages = BUFFER_PAGES;
          info->nFlashPages = FLASH_PAGES;
          info->flashStart = FLASH_START;
          memcpy(info->cpuId, cpuidGetId(), CPUID_LEN);
          
          pk.size = 2+sizeof(GetInfoReturns_t);
          
          radioSendPacket(&pk);
        }
        else if (pk.data[1] == CMD_LOAD_BUFFER)
        {
          int i=0;
          LoadBufferParameters_t *params = (LoadBufferParameters_t *)&pk.data[2];
          char *data = (char*) &pk.data[2+sizeof(LoadBufferParameters_t)];
          
          //Fill the buffer with the given datas
          for(i=0; i<(pk.size-(2+sizeof(LoadBufferParameters_t))) && (i+(params->page*PAGE_SIZE)+params->address)<(BUFFER_PAGES*PAGE_SIZE); i++)
          {
            buffer[(i+(params->page*PAGE_SIZE)+params->address)] = data[i];
          }
        }
        else if (pk.data[1] == CMD_READ_BUFFER)
        {
          int i=0;
          ReadBufferParameters_t *params = (ReadBufferParameters_t *)&pk.data[2];
          char *data = (char*) &pk.data[2+sizeof(ReadBufferParameters_t)];
          
          //Return the datas required
          for(i=0; i<25 && (i+(params->page*PAGE_SIZE)+params->address)<(BUFFER_PAGES*PAGE_SIZE); i++)
          {
            data[i] = buffer[(i+(params->page*PAGE_SIZE)+params->address)];
          }
          
          pk.size += i;
          
          radioSendPacket(&pk);
        }
        else if (pk.data[1] == CMD_READ_FLASH)
        {
          int i=0;
          ReadFlashParameters_t *params = (ReadFlashParameters_t *)&pk.data[2];
          char *data = (char*) &pk.data[2+sizeof(ReadFlashParameters_t)];
          char *flash= (char*)FLASH_BASE;
          
          //Return the datas required
          for(i=0; i<25 && (i+(params->page*PAGE_SIZE)+params->address)<(FLASH_PAGES*PAGE_SIZE); i++)
          {
            //data[i] = flash[(i+(params->page*PAGE_SIZE)+params->address)];
            //data[i] = *((char*)(FLASH_BASE+i+(params->page*PAGE_SIZE)+params->address));
            data[i] = flash[(i+(params->page*PAGE_SIZE)+params->address)];
          }
          
          pk.size += i;
          
          radioSendPacket(&pk);
        }
        else if (pk.data[1] == CMD_WRITE_FLASH)
        {
          int i;
          unsigned int error = 0xFF;
          int flashAddress;
          uint32_t *bufferToFlash;
          WriteFlashParameters_t *params = (WriteFlashParameters_t *)&pk.data[2];
          WriteFlashReturns_t *returns = (WriteFlashReturns_t *)&pk.data[2];
          
          //Test if it is an acceptable write request
          if ( (params->flashPage<FLASH_START) || (params->flashPage>=FLASH_PAGES) ||
               ((params->flashPage+params->nPages)>FLASH_PAGES) || (params->bufferPage>=BUFFER_PAGES)
             )
          {
            //Return a failure answer
            returns->done = 0;
            returns->error = 1;
            pk.size = 2+sizeof(WriteFlashReturns_t);
            radioSendPacket(&pk);
          }
          // Else, if everything is OK, flash the page(s)
          else
          {
            //Erase the page(s)
            for(i=0; i<params->nPages; i++)
            {
              if ( FLASH_ErasePage((params->flashPage+i)*PAGE_SIZE) != FLASH_COMPLETE)
              {
                error = 2;
                goto failure;
              }
            }
            
            //Write the data, long per long
            flashAddress = FLASH_BASE + (params->flashPage*PAGE_SIZE);
            bufferToFlash = (uint32_t *) (&buffer + (params->bufferPage*PAGE_SIZE));
            for (i=0; i<((params->nPages*PAGE_SIZE)/sizeof(uint32_t)); i++, flashAddress+=4)
            {
              if(FLASH_ProgramWord(flashAddress, bufferToFlash[i]) != FLASH_COMPLETE)
              {
                error = 3;
                goto failure;
              }
            }
            
            //Everything OK! great, send back an OK packet
            returns->done = 1;
            returns->error = 0;
            pk.size = 2+sizeof(WriteFlashReturns_t);
            radioSendPacket(&pk);
            
            goto finally;
            
            failure:
            //If the write procedure failed, send the error packet
            //TODO: see if it is necessary or wanted to send the reason as well
            returns->done = 0;
            returns->error = error;
            pk.size = 2+sizeof(WriteFlashReturns_t);
            radioSendPacket(&pk);
            
            finally:
            ; //None...
          }
        }
      }
    }
    
  }

  while(1);

  return 0;
}

//Clock configuration
static void prvClockInit(void)
{
  ErrorStatus HSEStartUpStatus;

  RCC_DeInit();
  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);
  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if (HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);

    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);

    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1);

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* ADCCLK = PCLK2/6 = 72 / 6 = 12 MHz*/
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    /* PLLCLK = 8MHz/2 * 16 = 64 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);

    /* Enable PLL */
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08);
  }
}



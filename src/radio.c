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
 * Radio.c - low level nRF24L01+ radio driver
 * The nRF24L chip is connected to the SPI1 port
 */
#include <string.h>

#include "stm32f10x_conf.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_spi.h"

#include "config.h"

#include "squeue.h"
#include "led.h"
#include "radio.h"

#include "nRF24L01.h"

/* Defines for the SPI and GPIO pins used to drive the SPI Flash */
#define RADIO_GPIO_CS             GPIO_Pin_12
#define RADIO_GPIO_CS_PORT        GPIOB
#define RADIO_GPIO_CS_PERIF       RCC_APB2Periph_GPIOB

#define RADIO_GPIO_CLK            GPIO_Pin_8
#define RADIO_GPIO_CLK_PORT       GPIOA
#define RADIO_GPIO_CLK_PERIF      RCC_APB2Periph_GPIOA

#define RADIO_GPIO_CE             GPIO_Pin_10
#define RADIO_GPIO_CE_PORT        GPIOA
#define RADIO_GPIO_CE_PERIF       RCC_APB2Periph_GPIOA

#define RADIO_GPIO_IRQ            GPIO_Pin_9
#define RADIO_GPIO_IRQ_PORT       GPIOA
#define RADIO_GPIO_IRQ_PERIF      RCC_APB2Periph_GPIOA
#define RADIO_GPIO_IRQ_SRC_PORT   GPIO_PortSourceGPIOA
#define RADIO_GPIO_IRQ_SRC        GPIO_PinSource9
#define RADIO_GPIO_IRQ_LINE       EXTI_Line9

#define RADIO_SPI                 SPI2
#define RADIO_SPI_CLK             RCC_APB1Periph_SPI2
#define RADIO_GPIO_SPI_PORT       GPIOB
#define RADIO_GPIO_SPI_CLK        RCC_APB2Periph_GPIOB
#define RADIO_GPIO_SPI_SCK        GPIO_Pin_13
#define RADIO_GPIO_SPI_MISO       GPIO_Pin_14
#define RADIO_GPIO_SPI_MOSI       GPIO_Pin_15

#define DUMMY_BYTE    0xA5

/* nRF24L SPI commands */
#define CMD_R_REG             0x00
#define CMD_W_REG             0x20
#define CMD_R_RX_PAYLOAD      0x61
#define CMD_W_TX_PAYLOAD      0xA0
#define CMD_FLUSH_TX          0xE1
#define CMD_FLUSH_RX          0xE2
#define CMD_REUSE_TX_PL       0xE3
#define CMD_ACTIVATE          0x50
#define CMD_RX_PL_WID         0x60
#define CMD_W_ACK_PAYLOAD     0xA8
#define CMD_W_PAYLOAD_NO_ACK  0xD0
#define CMD_NOP               0xFF

/* Usefull macro */
#define RADIO_EN_CS() GPIO_ResetBits(RADIO_GPIO_CS_PORT, RADIO_GPIO_CS)
#define RADIO_DIS_CS() GPIO_SetBits(RADIO_GPIO_CS_PORT, RADIO_GPIO_CS)
#define RADIO_DIS_CE() GPIO_ResetBits(RADIO_GPIO_CE_PORT, RADIO_GPIO_CE)
#define RADIO_EN_CE() GPIO_SetBits(RADIO_GPIO_CE_PORT, RADIO_GPIO_CE)
#define ACTIVATE_DATA   0x73

/* Private methods definition */
void radioHighInit();

// Private variables
static squeue_t rxQueue;
static squeue_t txQueue;


/* Low level SPI */
char radioSpiSendByte(char byte);
char radioSpiReceiveByte();

/* nRF24L commands, Every commands return the status byte */
unsigned char radioSpiWrite(unsigned char address, char *buffer, int len);
unsigned char radioSpiRead(unsigned char address, char *buffer, int len);
unsigned char radioSpiNop();
unsigned char radioSpiWrite1(unsigned char address, char byte);

//Union used to efficiently handle the packets (Private type)
typedef union
{
  CRTPPacket crtp;
  struct {
    uint8_t size;
    uint8_t data[32];
  } __attribute__((packed)) raw;
} RadioPacket;

/* Initialisation */
void radioInit(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
//  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable SPI and GPIO clocks */
  RCC_APB2PeriphClockCmd(RADIO_GPIO_SPI_CLK | RADIO_GPIO_CS_PERIF | 
                         RADIO_GPIO_CE_PERIF | RADIO_GPIO_IRQ_PERIF, ENABLE);

  /* Enable SPI and GPIO clocks */
  RCC_APB1PeriphClockCmd(RADIO_SPI_CLK, ENABLE);

  /* Configure main clock */
  GPIO_InitStructure.GPIO_Pin = RADIO_GPIO_CLK;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(RADIO_GPIO_CLK_PORT, &GPIO_InitStructure);

  /* Configure SPI pins: SCK, MISO and MOSI */
  GPIO_InitStructure.GPIO_Pin = RADIO_GPIO_SPI_SCK |  RADIO_GPIO_SPI_MOSI;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(RADIO_GPIO_SPI_PORT, &GPIO_InitStructure);

  //* Configure MISO */
  GPIO_InitStructure.GPIO_Pin = RADIO_GPIO_SPI_MISO;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(RADIO_GPIO_SPI_PORT, &GPIO_InitStructure);

  /* Configure I/O for the Chip select */
  GPIO_InitStructure.GPIO_Pin = RADIO_GPIO_CS;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(RADIO_GPIO_CS_PORT, &GPIO_InitStructure);

  /* Configure the interruption (EXTI Source) */
  GPIO_InitStructure.GPIO_Pin = RADIO_GPIO_IRQ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(RADIO_GPIO_IRQ_PORT, &GPIO_InitStructure);

/*  GPIO_EXTILineConfig(RADIO_GPIO_IRQ_SRC_PORT, RADIO_GPIO_IRQ_SRC);
  EXTI_InitStructure.EXTI_Line = RADIO_GPIO_IRQ_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
*/
  // Clock the radio with 16MHz
  RCC_MCOConfig(RCC_MCO_HSE);

  /* disable the chip select */
  RADIO_DIS_CS();

  /* Configure I/O for the Chip Enable */
  GPIO_InitStructure.GPIO_Pin = RADIO_GPIO_CE;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(RADIO_GPIO_CE_PORT, &GPIO_InitStructure);

  /* disable the chip enable */
  RADIO_DIS_CE();

  /* SPI configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(RADIO_SPI, &SPI_InitStructure);
  
  /* Enable the SPI  */
  SPI_Cmd(RADIO_SPI, ENABLE);

  //Initialise the queues
  sqInit(&rxQueue);
  sqInit(&txQueue);

  /* high level radio initialisation */
  radioHighInit();
}

/* SPI methods */
char radioSpiSendByte(char byte)
{
  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(RADIO_SPI, SPI_I2S_FLAG_TXE) == RESET);

  /* Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(RADIO_SPI, byte);

  /* Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(RADIO_SPI, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(RADIO_SPI);
}

char radioSpiReceiveByte()
{
  return radioSpiSendByte(DUMMY_BYTE);
}

/* nRF24L commands, Every commands return the status byte */

/* Read len bytes from a nRF24L register. 5 Bytes max */
unsigned char radioSpiRead(unsigned char address, char *buffer, int len)
{
  unsigned char status;
  int i;

  RADIO_EN_CS();

  /* Send the read command with the address */
  status = radioSpiSendByte( CMD_R_REG | (address&0x1F) );
  /* Read LEN bytes */
  for(i=0; i<len; i++)
    buffer[i]=radioSpiReceiveByte();

  RADIO_DIS_CS();

  return status;
}

/* Write len bytes a nRF24L register. 5 Bytes max */
unsigned char radioSpiWrite(unsigned char address, char *buffer, int len)
{
  unsigned char status;
  int i;

  RADIO_EN_CS();

  /* Send the write command with the address */
  status = radioSpiSendByte( CMD_W_REG | (address&0x1F) );
  /* Write LEN bytes */
  for(i=0; i<len; i++)
    radioSpiSendByte(buffer[i]);

  RADIO_DIS_CS();

  return status;
}

/* Write only one byte (useful for most of the reg.) */
unsigned char radioSpiWrite1(unsigned char address, char byte) {
  return radioSpiWrite(address, &byte, 1);
}

/* Read only one byte (useful for most of the reg.) */
unsigned char radioSpiRead1(unsigned char address) {
  char byte;

  radioSpiRead(address, &byte, 1);

  return byte;
}

/* Sent the NOP command. Used to get the status byte */
unsigned char radioSpiNop()
{
  unsigned char status;

  RADIO_EN_CS();
  status = radioSpiSendByte(CMD_NOP);
  RADIO_DIS_CS();

  return status;
}

unsigned char radioSpiFlushRx()
{
  unsigned char status;

  RADIO_EN_CS();
  status = radioSpiSendByte(CMD_FLUSH_RX);
  RADIO_DIS_CS();

  return status;
}

unsigned char radioSpiFlushTx()
{
  unsigned char status;

  RADIO_EN_CS();
  status = radioSpiSendByte(CMD_FLUSH_TX);
  RADIO_DIS_CS();

  return status;
}

// Return the payload length
unsigned char radioSpiRxLength()
{
  unsigned char length;

  RADIO_EN_CS();
  radioSpiSendByte(CMD_RX_PL_WID);
  length = radioSpiReceiveByte();
  RADIO_DIS_CS();

  return length;
}

//Send the activate command required by the "non-plus" radio device
unsigned char radioSpiActivate()
{
  unsigned char status;
  
  RADIO_EN_CS();
  status = radioSpiSendByte(CMD_ACTIVATE);
  radioSpiSendByte(ACTIVATE_DATA);
  RADIO_DIS_CS();

  return status;
}

// Write the ack payload of the pipe 0
unsigned char radioSpiWriteAck(char *buffer, int len)
{
  unsigned char status;
  int i;

  RADIO_EN_CS();

  /* Send the read command with the address */
  status = radioSpiSendByte(CMD_W_ACK_PAYLOAD);
  /* Read LEN bytes */
  for(i=0; i<len; i++)
    radioSpiSendByte(buffer[i]);

  RADIO_DIS_CS();

  return status;
}

// Read the RX payload
unsigned char radioSpiReadRX(char *buffer, int len)
{
  unsigned char status;
  int i;

  RADIO_EN_CS();

  /* Send the read command with the address */
  status = radioSpiSendByte(CMD_R_RX_PAYLOAD);
  /* Read LEN bytes */
  for(i=0; i<len; i++)
    buffer[i]=radioSpiReceiveByte();

  RADIO_DIS_CS();

  return status;
}


//Hard interrupt handler
/*
void extiInterruptHandler(void)
{
  radioIsr();
  EXTI_ClearITPendingBit(EXTI_Line11);
}
*/

/* Interrupt service routine, fetch the datas and setup a new transaction
 * TODO: fetch the data using a DMA channel
 */
void radioIsr()
{
  uint8_t dataLen;
  RadioPacket pk;

  //Receive the radio packet
  //Dis the radio during the reception
  RADIO_DIS_CE();
  ledSetRed(1);
  //Fetch all the data (Loop until the RX Fifo is NOT empty)
  while( !(radioSpiRead1(REG_FIFO_STATUS)&0x01) )
  {
    dataLen = radioSpiRxLength();

    if (dataLen>32)          //If a packet has a wrong size it is dropped
      radioSpiFlushRx();
    else                     //Else, it is processed
    {
      //Fetch the data
      pk.raw.size = dataLen-1;
      radioSpiReadRX((char *)pk.raw.data, dataLen);

      //Push the received packet into the RX queue (except if it is a null packet)
      if ( (pk.raw.size>0) || (pk.raw.data[0] != 0xFF))
        sqPut(&rxQueue, (CRTPPacket *)&pk);
    }
  }

  //Push the data to send (Loop until the TX Fifo is full or there is no more data to send)
  while( (sqGet(&txQueue, (CRTPPacket *)&pk) == SQ_OK) && !(radioSpiRead1(REG_FIFO_STATUS)&0x20) )
  {
    pk.raw.size++;

    radioSpiWriteAck((char *)pk.raw.data, pk.raw.size);
  }

  //clear the interruptions flags
  radioSpiWrite1(REG_STATUS, 0x70);

  ledSetRed(0);
  //Re-enable the radio
  RADIO_EN_CE();

  return;
}


/* Radio High level initialisatio (ie. the SPI is inistialised. 
 * This function init the radio chip itself */
void radioHighInit() {
  char status;
  int i;

  /* Halt the radio (if it was enable) */
  radioSpiWrite1(REG_CONFIG, 0x04);
  for(i=0;i<64000;i++) asm( "  nop" ); //Yeld
  
  //NRF24L01 special activation command
  radioSpiActivate();

  status = radioSpiNop();

  //Set the radio chanel if it has been defined
#ifdef RADIO_CHANEL
  radioSpiWrite1(REG_RF_CH, RADIO_CHANEL);
#else
  radioSpiWrite1(REG_RF_CH, 2);
#endif
  //Power the radio, Enable the DS interruption, set the radio in receptor mode
  radioSpiWrite1(REG_CONFIG, 0x3F);
  for(i=0;i<64000;i++) asm( "  nop" ); //Wait for the chip to be ready
  // Enable the dynamic payload size and the ack payload for the pipe 0
  radioSpiWrite1(REG_FEATURE, 0x06);
  radioSpiWrite1(REG_DYNPD, 0x01);

  //Flush RX
  for(i=0;i<3;i++)
    radioSpiFlushRx();
  //Flush TX
  for(i=0;i<3;i++)
    radioSpiFlushTx();


  //Enable the radio
  RADIO_EN_CE();
}

/****** Public functions ********/
//Check the IRQ line
int radioIsIrq()
{
  return !GPIO_ReadInputDataBit(RADIO_GPIO_IRQ_PORT, RADIO_GPIO_IRQ);
}

//Send and receive packets
int radioReceivePacket(CRTPPacket *p)
{
  return !sqGet(&rxQueue, p);
}

int radioSendPacket(CRTPPacket *p)
{
  return !sqPut(&txQueue, p);
}


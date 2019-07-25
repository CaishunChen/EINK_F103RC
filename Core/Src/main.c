
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "fatfs.h"
#include "iwdg.h"
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "bsp_norflash.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
NORFLASH_OBJ FatFlash = {&hspi1, {GPIOA, GPIO_PIN_4}, NULL};

static NORFLASH_OBJ Flash_U9  = {&hspi2, {GPIOC, GPIO_PIN_8}, NULL};
static NORFLASH_OBJ Flash_U10 = {&hspi2, {GPIOC, GPIO_PIN_7}, NULL};
static NORFLASH_OBJ Flash_U25 = {&hspi2, {GPIOC, GPIO_PIN_6}, NULL};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void FileVoltage(void *);
static void FileToFlash(void *, int, void *, void (*)(void *));
static void U9_CallBack(void *);
static void AllCallBack(void *);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define FILE_NAME_CFG "config.txt"
#define FILE_NAME_U9  "FPGA-DATA.bin"
#define FILE_NAME_U10 "FPGA-LED-4M.bin"
#define FILE_NAME_U25 "IT8951-97.bin"
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_IWDG_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USB_DEVICE_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  __HAL_SPI_ENABLE(&hspi1);
  __HAL_SPI_ENABLE(&hspi2);

  uint8_t state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
  NORFLASH_API *norflash = BSP_NORFLASH_API();

  norflash->Init(&FatFlash);

  retUSER = f_mount(&USERFatFS, USERPath, 1);
  if (retUSER != FR_OK)
    retUSER = f_mkfs(USERPath, 0, 4096);

  FileVoltage(&FatFlash, FILE_NAME_CFG);

  if (state == 0)
  {
    norflash->Init(&Flash_U9);
    norflash->Init(&Flash_U10);
    norflash->Init(&Flash_U25);

    FileToFlash(&Flash_U9,  Flash_U9.Desc->SizeK - 1024, FILE_NAME_U9,  U9_CallBack);
    FileToFlash(&Flash_U10, Flash_U10.Desc->SizeK,       FILE_NAME_U10, AllCallBack);
    FileToFlash(&Flash_U25, Flash_U25.Desc->SizeK,       FILE_NAME_U25, AllCallBack);
  }

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);

  if (state == 0)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_IWDG_Refresh(&hiwdg);
    HAL_Delay(500);
    if (state == 0)
    {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
      if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3))
      {
        __disable_irq();
        HAL_NVIC_SystemReset();
      }
    }
    else
    {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
      if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == 0)
      {
        __disable_irq();
        HAL_NVIC_SystemReset();
      }
    }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
static void FileVoltage(void *obj, void *FileName)
{
  if (obj == NULL)
    return;

  NORFLASH_OBJ *Obj = obj;

  if (Obj->Desc == NULL)
    return;

  NORFLASH_API *norflash = BSP_NORFLASH_API();

  char  linebuf[32] = {0};
  int   rwcnt       = 0;
  float vcomflash   = 0;
  float vcomfile    = 0;
  int   needcreate  = 1;

  DIR     dir   = {0};
  FILINFO finfo = {0};
  TCHAR   lbuf[_MAX_LFN + 1] = {0};

  finfo.lfname = lbuf;
  finfo.lfsize = sizeof(lbuf);

  norflash->DataRead(Obj, 0, &vcomflash, sizeof(float));
  if (!isnormal(vcomflash))
  {
    vcomflash = 0;
    norflash->DataWrite(Obj, 0, &vcomflash, sizeof(float));
  }

  retUSER = f_findfirst(&dir, &finfo, "", FileName);
  if (retUSER != FR_OK)
    goto CREATE_CFG;

  retUSER = f_open(&USERFile, finfo.fname, FA_READ);
  if (retUSER != FR_OK)
    goto CREATE_CFG;

  retUSER = f_read(&USERFile, linebuf, sizeof(linebuf), (UINT *)&rwcnt);
  if (retUSER != FR_OK)
    goto CLOSE_FILE;

  if (strncmp(linebuf, "VCOM:", 5) != 0)
    goto CLOSE_FILE;

  vcomfile = atof(&linebuf[5]);
  if (!isnormal(vcomfile))
    goto CLOSE_FILE;

  if (vcomflash != vcomfile)
    norflash->DataWrite(Obj, 0, &vcomfile, sizeof(float));

  needcreate = 0;

CLOSE_FILE:
  retUSER = f_close(&USERFile);

CREATE_CFG:
  if (!needcreate)
    return;

  retUSER = f_open(&USERFile, FileName, FA_CREATE_ALWAYS | FA_WRITE);
  if (retUSER != FR_OK)
    return;

  memset(linebuf, 0, sizeof(linebuf));

  snprintf(linebuf, sizeof(linebuf), "VCOM:%.3f", vcomflash);

  retUSER = f_write(&USERFile, linebuf, strlen(linebuf), &rwcnt);

  retUSER = f_close(&USERFile);
}

#define BUF_LENS (1024)

static void FileToFlash(void *obj, int SizeK, void *FileName, void (*EraseCallBack)(void *))
{
  if (obj == NULL)
    return;

  NORFLASH_OBJ *Obj = obj;

  if (Obj->Desc == NULL)
    return;

  NORFLASH_API *norflash = BSP_NORFLASH_API();

  char fatbuf[BUF_LENS] = {0};
  char norbuf[BUF_LENS] = {0};

  int  filesize   = 0;
  int  fileptr    = 0;
  int  length     = 0;
  int  rcnt       = 0;
  int  check_pass = 0;

  DIR     dir   = {0};
  FILINFO finfo = {0};
  TCHAR   lbuf[_MAX_LFN + 1] = {0};

  finfo.lfname = lbuf;
  finfo.lfsize = sizeof(lbuf);

  retUSER = f_findfirst(&dir, &finfo, "", FileName);
  if (retUSER != FR_OK)
    return;

  retUSER = f_open(&USERFile, finfo.fname, FA_READ);
  if (retUSER != FR_OK)
    return;

  EraseCallBack(Obj);

  filesize = f_size(&USERFile);
  fileptr  = 0;

  if (filesize > SizeK * 1024)
    filesize = SizeK * 1024;

  while (fileptr < filesize)
  {
    if (filesize - fileptr > sizeof(fatbuf))
      length = sizeof(fatbuf);
    else
      length = filesize - fileptr;

    retUSER = f_read(&USERFile, fatbuf, length, (UINT *)&rcnt);
    if ((retUSER != FR_OK) || (rcnt != length))
      goto RETURN;

    norflash->DataWrite(Obj, fileptr, fatbuf, length);
    norflash->DataRead(Obj, fileptr, norbuf, length);

    for (int i = 0; i < length; i++)
    {
      if (norbuf[i] != fatbuf[i])
        goto RETURN;
    }

    fileptr += length;
  }

  if (fileptr == filesize)
    check_pass = 1;

RETURN:
  retUSER = f_close(&USERFile);

  if (check_pass)
    f_unlink(finfo.fname);
}

static void U9_CallBack(void *obj)
{
  NORFLASH_API *norflash = BSP_NORFLASH_API();
  NORFLASH_OBJ *Obj = obj;

  for (int i = 0; i < Obj->Desc->SizeK - 1024; i += Obj->Desc->BlkSizeK)
    norflash->BlockErase(Obj, i * 1024);
}

static void AllCallBack(void *obj)
{
  NORFLASH_API *norflash = BSP_NORFLASH_API();

  norflash->ChipErase(obj);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/**
  ******************************************************************************
  * File Name          : Flash.c
  * Description        : This file provides code for the control, reading and 
  *                      writing of the Flash Memory on the Design.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 WeatherCloud
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of WeatherCloud nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Flash.h"
#include <math.h>
#include "stdbool.h"

// Frame Structure Define
//FrameStructure Frame_Save  @ 0x08070000;

/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Uncomment this line to Enable Write Protection */
//#define WRITE_PROTECTION_ENABLE

/* Uncomment this line to Disable Write Protection */
#define WRITE_PROTECTION_DISABLE

/* Check the status of the switches */
/* Enable by default the disable protection */
#if !defined(WRITE_PROTECTION_ENABLE)&&!defined(WRITE_PROTECTION_DISABLE)
#define WRITE_PROTECTION_DISABLE
#endif /* !WRITE_PROTECTION_ENABLE && !WRITE_PROTECTION_DISABLE */

/* Both switches cannot be enabled in the same time */
#if defined(WRITE_PROTECTION_ENABLE)&&defined(WRITE_PROTECTION_DISABLE)
#error "Switches WRITE_PROTECTION_ENABLE & WRITE_PROTECTION_DISABLE cannot be enabled in the time!"
#endif /* WRITE_PROTECTION_ENABLE && WRITE_PROTECTION_DISABLE */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t Address = 0;
uint32_t PageError = 0;
uint32_t ProtectedPAGE = 0x0;
__IO TestStatus MemoryProgramStatus = PASSED;
/*Variable used for Erase procedure*/
//static FLASH_EraseInitTypeDef EraseInitStruct;
/*Variable used to handle the Options Bytes*/
//static FLASH_OBProgramInitTypeDef OptionsBytesStruct;

//  uint32_t NbPages;     /*!< NbPages: Number of pages to be erased.
//                             This parameter must be a value between 1 and (max number of pages - value of Initial page)*/


  /**
  * @brief  This function Initializes Option Bytes and writes the specified data to Target Flash memory.
  * @param  uint32_t  FlashProtect: Specifies the sector(s) which are write protected between Sector 0 to 31.
  *                                 This parameter can be a combination of @ref FLASHEx_Option_Bytes_Write_Protection1
  *                                   @defgroup FLASHEx_Option_Bytes_Write_Protection1 FLASHEx Option Bytes Write Protection1
  *                                   Module stm32l1xx_hal_flash_ex.h
  * @retval HAL_StatusTypeDef:     HAL_OK:       Flash Operation success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     Flash is busy.
  *                                HAL_TIMEOUT:  Flash timed out.
  */
HAL_StatusTypeDef RoadBrd_FlashInitOption( uint32_t  FlashProtect)
{
  HAL_StatusTypeDef Status;
  /*Variable used to handle the Options Bytes*/
  static FLASH_OBProgramInitTypeDef OptionsBytesStruct;

  Status = HAL_OK;

  //*
  //*
  //* INITITIALIZE KEY STRUCTURES BEFORE STARTING OPERATION.
  //*
  //*
  Status = HAL_OK;
  /* Unlock the Flash to enable the flash control register access *************/ 
  HAL_FLASH_Unlock();

  /* Unlock the Options Bytes *************************************************/
  HAL_FLASH_OB_Unlock();

  /* Get pages write protection status ****************************************/
  HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct);

#ifdef WRITE_PROTECTION_DISABLE
  /* Check if desired pages are already write protected ***********************/
  if((OptionsBytesStruct.WRPSector0To31 & FlashProtect) == FlashProtect)
  {
    /* Restore write protected pages */
    OptionsBytesStruct.OptionType   = OPTIONBYTE_WRP;
    OptionsBytesStruct.WRPState     = OB_WRPSTATE_DISABLE;
    OptionsBytesStruct.WRPSector0To31 = FlashProtect;
    Status = HAL_FLASHEx_OBProgram(&OptionsBytesStruct);
    if(Status != HAL_OK)
      return Status;

    /* Generate System Reset to load the new option byte values ***************/
    HAL_FLASH_OB_Launch();
  }
#elif defined WRITE_PROTECTION_ENABLE
  /* Get current write protected pages and the new pages to be protected ******/
  ProtectedPAGE =  OptionsBytesStruct.WRPSector0To31 | FlashProtect; 

  /* Check if desired pages are not yet write protected ***********************/
  if((OptionsBytesStruct.WRPSector0To31 & FlashProtect )!= FlashProtect)
  {
    /* Enable the pages write protection **************************************/
    OptionsBytesStruct.OptionType = OPTIONBYTE_WRP;
    OptionsBytesStruct.WRPState   = OB_WRPSTATE_ENABLE;
    OptionsBytesStruct.WRPSector0To31    = FlashProtect;
    Status = HAL_FLASHEx_OBProgram(&OptionsBytesStruct);
    if(Status != HAL_OK)
      return Status;

    /* Generate System Reset to load the new option byte values ***************/
    HAL_FLASH_OB_Launch();
  }
#endif /* WRITE_PROTECTION_DISABLE */
  /* Lock the Options Bytes *************************************************/
  HAL_FLASH_OB_Lock();

  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
  return Status;
}

/**
  * @brief  This function Initializes Option Bytes and writes the specified data to Target Flash memory.
  * @param  uint32_t  FlashProtect: Specifies the sector(s) which are write protected between Sector 0 to 31.
  *                                 This parameter can be a combination of @ref FLASHEx_Option_Bytes_Write_Protection1
  *                                   @defgroup FLASHEx_Option_Bytes_Write_Protection1 FLASHEx Option Bytes Write Protection1
  *                                   Module stm32l1xx_hal_flash_ex.h
  * @param  uint32_t TypeErase:     Page Erase only.
  *                                 This parameter can be a value of @ref FLASHEx_Type_Erase
  *                                   @defgroup FLASHEx_Option_Type FLASHEx Option Type
  *                                   Module stm32l1xx_hal_flash_ex.h
  * @param  uint32_t FlashAddress:  Initial FLASH address to be erased and written.
  *                                 This parameter must be a value belonging to FLASH Programm address (depending on the devices)
  * @param  uint32_t *ReadAddress:  Address of data to be written to flash.
  * @param  uint32_t Size:          Number of bytes/pages to be erased and written. Note that the Pages are an increment of 256 Bytes rounded up.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Flash Operation success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     Flash is busy.
  *                                HAL_TIMEOUT:  Flash timed out.
  */
HAL_StatusTypeDef RoadBrd_FlashInitWrite( uint32_t  FlashProtect,
                                      uint32_t TypeErase,
                                      uint32_t FlashAddress,
                                      uint32_t *ReadAddress,
                                      uint32_t Size)
{
  HAL_StatusTypeDef Status;
  float num_pages;
  uint32_t Address;
  uint32_t EndAddress;
  uint32_t *DataPtr;
  /*Variable used for Erase procedure*/
  static FLASH_EraseInitTypeDef EraseInitStruct;
  /*Variable used to handle the Options Bytes*/
  static FLASH_OBProgramInitTypeDef OptionsBytesStruct;

  Status = HAL_OK;

  //*
  //*
  //* INITITIALIZE KEY STRUCTURES BEFORE STARTING OPERATION.
  //*
  //*
  Status = HAL_OK;
  /* Unlock the Flash to enable the flash control register access *************/ 
  HAL_FLASH_Unlock();

  /* Unlock the Options Bytes *************************************************/
  HAL_FLASH_OB_Unlock();

  /* Get pages write protection status ****************************************/
  HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct);

#ifdef WRITE_PROTECTION_DISABLE
  /* Check if desired pages are already write protected ***********************/
  if((OptionsBytesStruct.WRPSector0To31 & FlashProtect) == FlashProtect)
  {
    /* Restore write protected pages */
    OptionsBytesStruct.OptionType   = OPTIONBYTE_WRP;
    OptionsBytesStruct.WRPState     = OB_WRPSTATE_DISABLE;
    OptionsBytesStruct.WRPSector0To31 = FlashProtect;
    Status = HAL_FLASHEx_OBProgram(&OptionsBytesStruct);
    if(Status != HAL_OK)
      return Status;

    /* Generate System Reset to load the new option byte values ***************/
    HAL_FLASH_OB_Launch();
  }
#elif defined WRITE_PROTECTION_ENABLE
  /* Get current write protected pages and the new pages to be protected ******/
  ProtectedPAGE =  OptionsBytesStruct.WRPSector0To31 | FlashProtect; 

  /* Check if desired pages are not yet write protected ***********************/
  if((OptionsBytesStruct.WRPSector0To31 & FlashProtect )!= FlashProtect)
  {
    /* Enable the pages write protection **************************************/
    OptionsBytesStruct.OptionType = OPTIONBYTE_WRP;
    OptionsBytesStruct.WRPState   = OB_WRPSTATE_ENABLE;
    OptionsBytesStruct.WRPSector0To31    = FlashProtect;
    Status = HAL_FLASHEx_OBProgram(&OptionsBytesStruct);
    if(Status != HAL_OK)
      return Status;

    /* Generate System Reset to load the new option byte values ***************/
    HAL_FLASH_OB_Launch();
  }
#endif /* WRITE_PROTECTION_DISABLE */
  /* Lock the Options Bytes *************************************************/
  HAL_FLASH_OB_Lock();

  //*
  //*
  //* ERASE TARGET FLASH MEMORY.
  //*
  //*
  /* The selected pages are not write protected *******************************/
  if ((OptionsBytesStruct.WRPSector0To31 & FlashProtect) == 0x00)
  {
    /* Fill EraseInit structure************************************************/
    EraseInitStruct.TypeErase   = TypeErase;
    EraseInitStruct.PageAddress = FlashAddress;
    num_pages = Size/FLASH_PAGE_SIZE;
    EraseInitStruct.NbPages     = (uint32_t)ceil(num_pages);

    Status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
    if(Status != HAL_OK)
    {
      /* Lock the Flash to disable the flash control register access (recommended
        to protect the FLASH memory against possible unwanted operation) *********/
      HAL_FLASH_Lock();
      return Status;
    }

    //*
    //*
    //* WRITE TO TARGET FLASH MEMORY.
    //*
    //*
    Address = FlashAddress;
    EndAddress = FlashAddress + Size;
    DataPtr = ReadAddress;
    while (Address < EndAddress)
    {
      Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, *DataPtr++);
      if(Status == HAL_OK)
      {
        Address = Address + 4;
      }
      else
      {
        /* Lock the Flash to disable the flash control register access (recommended
          to protect the FLASH memory against possible unwanted operation) *********/
        HAL_FLASH_Lock();
       /* Error occurred while writing data in Flash memory. */
        return Status;
      }
    }

    //*
    //*
    //* VERIFY WRITE TO TARGET FLASH MEMORY.
    //*
    //*
    /* Check the correctness of written data */
    Address = FlashAddress;
    EndAddress = FlashAddress + Size;
    DataPtr = ReadAddress;

    while (Address < EndAddress)
    {
      if((*(__IO uint32_t*) Address) != *DataPtr++)
      {
        Status = HAL_ERROR;
        /* Lock the Flash to disable the flash control register access (recommended
          to protect the FLASH memory against possible unwanted operation) *********/
        HAL_FLASH_Lock();
        return Status;
      }
      Address += 4;
    }
  } //EndIf ((OptionsBytesStruct.WRPSector0To31 & FlashProtect) == 0x00)
  else
  { 
    /* The desired pages are write protected */ 
    /* Check that it is not allowed to write in this page */
    Address = FlashAddress;
    DataPtr = ReadAddress;
    Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, *DataPtr++);
    if (Status != HAL_OK)
    {
      /* Error returned during programmation. */
      /* Check that WRPERR flag is well set */
      if (HAL_FLASH_GetError() == HAL_FLASH_ERROR_WRP) 
      {
        Status = HAL_ERROR;
      }
      else
      {
        /* Another error occurred.
           User can add here some code to deal with this error */
        Status = HAL_ERROR;
      }
    }
    else
    {
      /* Write operation is successful. Should not occur
         User can add here some code to deal with this error */
      Status = HAL_ERROR;
    }
  } //EndElse ((OptionsBytesStruct.WRPSector0To31 & FlashProtect) == 0x00)

  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
  return Status;
}

/**
  * @brief  This function writes the specified data to Target Flash memory.
  * @param  uint32_t  FlashProtect: Specifies the sector(s) which are write protected between Sector 0 to 31.
  *                                 This parameter can be a combination of @ref FLASHEx_Option_Bytes_Write_Protection1
  *                                   @defgroup FLASHEx_Option_Bytes_Write_Protection1 FLASHEx Option Bytes Write Protection1
  *                                   Module stm32l1xx_hal_flash_ex.h
  * @param  uint32_t TypeErase:     Page Erase only.
  *                                 This parameter can be a value of @ref FLASHEx_Type_Erase
  *                                   @defgroup FLASHEx_Option_Type FLASHEx Option Type
  *                                   Module stm32l1xx_hal_flash_ex.h
  * @param  uint32_t FlashAddress:  Initial FLASH address to be erased and written.
  *                                 This parameter must be a value belonging to FLASH Programm address (depending on the devices)
  * @param  uint32_t *ReadAddress:  Address of data to be written to flash.
  * @param  uint32_t Size:          Number of bytes/pages to be erased and written. Note that the Pages are an increment of 256 Bytes rounded up.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Flash Operation success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     Flash is busy.
  *                                HAL_TIMEOUT:  Flash timed out.
  */
HAL_StatusTypeDef RoadBrd_FlashWrite( uint32_t  FlashProtect,
                                      uint32_t TypeErase,
                                      uint32_t FlashAddress,
                                      uint32_t *ReadAddress,
                                      uint32_t Size)
{
  HAL_StatusTypeDef Status;
  float num_pages;
  uint32_t Address;
  uint32_t EndAddress;
  uint32_t *DataPtr;
  /*Variable used for Erase procedure*/
  static FLASH_EraseInitTypeDef EraseInitStruct;
  /*Variable used to handle the Options Bytes*/
  static FLASH_OBProgramInitTypeDef OptionsBytesStruct;

  Status = HAL_OK;

  /* Unlock the Flash to enable the flash control register access *************/ 
  Status = HAL_FLASH_Unlock();
  if(Status != HAL_OK)
  {
    return Status;
  }

  // Clear any outstanding Flash States before starting...
  SET_BIT(FLASH->SR, FLASH_SR_WRPERR);
  //*
  //*
  //* ERASE TARGET FLASH MEMORY.
  //*
  //*
  /* The selected pages are not write protected *******************************/
  if ((OptionsBytesStruct.WRPSector0To31 & FlashProtect) == 0x00)
  {
    /* Fill EraseInit structure************************************************/
    EraseInitStruct.TypeErase   = TypeErase;
    EraseInitStruct.PageAddress = FlashAddress;
    num_pages = (float)Size/FLASH_PAGE_SIZE;
    EraseInitStruct.NbPages     = (uint32_t)ceil(num_pages);

    Status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
    if(Status != HAL_OK)
    {
      /* Lock the Flash to disable the flash control register access (recommended
        to protect the FLASH memory against possible unwanted operation) *********/
      HAL_FLASH_Lock();
      return Status;
    }

    //*
    //*
    //* WRITE TO TARGET FLASH MEMORY.
    //*
    //*
    Address = FlashAddress;
    EndAddress = FlashAddress + Size;
    DataPtr = ReadAddress;
    while (Address < EndAddress)
    {
      Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, *DataPtr++);
      if(Status == HAL_OK)
      {
        Address = Address + 4;
      }
      else
      {
        /* Lock the Flash to disable the flash control register access (recommended
          to protect the FLASH memory against possible unwanted operation) *********/
        HAL_FLASH_Lock();
       /* Error occurred while writing data in Flash memory. */
        return Status;
      }
    }

    //*
    //*
    //* VERIFY WRITE TO TARGET FLASH MEMORY.
    //*
    //*
    /* Check the correctness of written data */
    Address = FlashAddress;
    EndAddress = FlashAddress + Size;
    DataPtr = ReadAddress;

    while (Address < EndAddress)
    {
      if((*(__IO uint32_t*) Address) != *DataPtr++)
      {
        Status = HAL_ERROR;
        /* Lock the Flash to disable the flash control register access (recommended
          to protect the FLASH memory against possible unwanted operation) *********/
        HAL_FLASH_Lock();
        return Status;
      }
      Address += 4;
    }
  } //EndIf ((OptionsBytesStruct.WRPSector0To31 & FlashProtect) == 0x00)
  else
  { 
    /* The desired pages are write protected */ 
    /* Check that it is not allowed to write in this page */
    Address = FlashAddress;
    DataPtr = ReadAddress;
    Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, *DataPtr++);
    if (Status != HAL_OK)
    {
      /* Error returned during programmation. */
      /* Check that WRPERR flag is well set */
      if (HAL_FLASH_GetError() == HAL_FLASH_ERROR_WRP) 
      {
        Status = HAL_ERROR;
      }
      else
      {
        /* Another error occurred.
           User can add here some code to deal with this error */
        Status = HAL_ERROR;
      }
    }
    else
    {
      /* Write operation is successful. Should not occur
         User can add here some code to deal with this error */
      Status = HAL_ERROR;
    }
  } //EndElse ((OptionsBytesStruct.WRPSector0To31 & FlashProtect) == 0x00)

  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
  return Status;
}

/**
  * @brief  This function reads the specified data from Target Flash memory.
  * @param  uint32_t FlashAddress:  Initial FLASH address to be read.
  * @param  uint32_t *ReadAddress:  Address of data to be read from flash.
  * @param  uint32_t Size:          Number of bytes to read.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Flash Operation success.
  */
HAL_StatusTypeDef RoadBrd_FlashRead(  uint32_t FlashAddress,
                                      uint32_t *ReadAddress,
                                      uint32_t Size)
{
  uint32_t Address;
  uint32_t EndAddress;
  uint32_t *DataPtr;

  //* Read Data to Target Locations.
  Address = FlashAddress;
  EndAddress = FlashAddress + Size;
  DataPtr = ReadAddress;

  while (Address < EndAddress)
  {
    *DataPtr++ = (*(__IO uint32_t*) Address);
    Address += 4;
  }

  return HAL_OK;
}


/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/

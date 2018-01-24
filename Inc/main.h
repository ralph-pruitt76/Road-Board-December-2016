/**
  ******************************************************************************
  * File Name          : main.h
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __main_H
#define __main_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
 
   
/* Definition for USARTx's NVIC */
#define USARTx_IRQn        USART3_IRQn
#define BRD_REV         "Rev M"               // PCB Revision          
#define VERSION_NUM     "N.9.1"                 // Monitor Revision
#define REL_DATE        "Jan 23, 2018"
//#define LEGACY_BANNER   "Rev G+ REV C"        // OLD.....Needed to allow Legacy Design to work
#define LEGACY_BANNER   "N.9.1 1/23/18"        // Needed to allow Legacy Design to work

/* Prototypes */
//int isHexNum(char *ptr);
//int hatoi( char *ptr );
void SystemClock_Config(void);

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/

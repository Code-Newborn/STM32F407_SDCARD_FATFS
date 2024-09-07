/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "sdio.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config( void );
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t count = 0;

// FATFS   SDFatFS;        // File system object
FIL     MyFile;         // File object
FRESULT fres;           // FATFS function common result code
char    buffer[ 100 ];  // Buffer for file operations

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main( void ) {

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

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
    MX_SDIO_SD_Init();
    MX_FATFS_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */

    HAL_SD_Init( &hsd );  // WARNING 初始化不能省略

    if ( HAL_SD_ConfigWideBusOperation( &hsd, SDIO_BUS_WIDE_4B ) != HAL_OK ) {  // WARNING 先设置1-bit后切换4-bit
        // 设置失败，处理错误
        printf( "切换SDIO_4-bit失败，处理错误\n" );
        Error_Handler();
    }

    uint32_t byteswritten;                                 /* File write counts */
    uint32_t bytesread;                                    /* File read counts */
    uint8_t  wtext[] = "This is STM32 working with FatFs"; /* File write buffer */
    uint8_t  rtext[ 100 ];                                 /* File read buffers */
    char     filename[] = "STM32cube.txt";
    char     SensorBuff[ 100 ];
    printf( "********* STM32CubeMX FatFs Example *********\r\n\r\n" );
    retSD = f_mount( &SDFatFS, SDPath, 1 );
    if ( retSD == FR_OK ) {
        printf( "f_mount sucess!!! \r\n" );
        if ( f_open( &SDFile, filename, FA_CREATE_ALWAYS | FA_WRITE ) == FR_OK ) {  // 创建文件并写入
            printf( "f_open file sucess!!! \r\n" );
            retSD = f_write( &SDFile, wtext, sizeof( wtext ), &byteswritten );  // 向文件写入数据
            if ( retSD == FR_OK ) {
                printf( "f_write file sucess!!! \r\n" );
                printf( "f_write Data : %s\r\n", wtext );

                // 关闭文件
                retSD = f_close( &SDFile );
                if ( retSD )
                    printf( "f_close error!!! %d\r\n", retSD );
                else
                    printf( "f_close sucess!!! \r\n" );
            }
            else
                printf( "f_write file error %d\r\n", retSD );
        }
        else
            printf( "f_open file error\r\n" );
    }
    else
        printf( "f_mount error : %d \r\n", retSD );

    retSD = f_open( &SDFile, filename, FA_READ );
    if ( retSD )
        printf( "f_open file error : %d\r\n", retSD );
    else
        printf( "f_open file sucess!!! \r\n" );

    retSD = f_read( &SDFile, rtext, sizeof( rtext ), ( UINT* )&bytesread );
    if ( retSD )
        printf( "f_read error!!! %d\r\n", retSD );
    else {
        printf( "f_read sucess!!! \r\n" );
        printf( "f_read Data : %s\r\n", rtext );
    }

    // 关闭文件
    retSD = f_close( &SDFile );
    if ( retSD )
        printf( "f_close error!!! %d\r\n", retSD );
    else
        printf( "f_close sucess!!! \r\n" );

    // 读取数据量等于写入数据量
    if ( bytesread == byteswritten )
        printf( "FatFs is working well!!!\r\n" );

    if ( f_open( &SDFile, ( const char* )"Sensor.csv", FA_CREATE_ALWAYS | FA_WRITE ) == FR_OK ) {
        printf( "Sensor.csv was opened/created!!!\r\n" );
        sprintf( SensorBuff, "Item,Temp,Humi,Light\r\n" );
        f_write( &SDFile, SensorBuff, strlen( SensorBuff ), &byteswritten );

        for ( int i = 0; i < 10; i++ ) {
            sprintf( SensorBuff, "%d,%d,%d,%d\r\n", i + 1, i + 20, i + 30, i + 40 );
            f_write( &SDFile, SensorBuff, strlen( SensorBuff ), &byteswritten );
            f_sync( &SDFile );
        }
        f_close( &SDFile );
    }

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while ( 1 ) {
        if ( HAL_GPIO_ReadPin( KEY_GPIO_Port, KEY_Pin ) == GPIO_PIN_SET ) {
            HAL_Delay( 50 );
            if ( HAL_GPIO_ReadPin( KEY_GPIO_Port, KEY_Pin ) == GPIO_PIN_SET ) {


                switch ( count ) {
                case 0:
                    SDCard_ShowInfo(); /*显示SD卡的信息*/
                    break;
                case 1:
                    SDCard_EraseBlocks(); /*SD_Card 擦除测试*/
                    break;
                case 2:
                    SDCard_TestWrite(); /*SD_Card 写入测试*/
                    break;
                case 3:
                    SDCard_TestRead(); /*SD_Card 读取测试*/
                    break;

                default:
                    break;
                }

                count = ( ++count ) % 4;

                while ( HAL_GPIO_ReadPin( KEY_GPIO_Port, KEY_Pin ) )
                    ;
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
void SystemClock_Config( void ) {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 4;
    RCC_OscInitStruct.PLL.PLLN       = 168;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 7;
    if ( HAL_RCC_OscConfig( &RCC_OscInitStruct ) != HAL_OK ) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if ( HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_5 ) != HAL_OK ) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler( void ) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while ( 1 ) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed( uint8_t* file, uint32_t line ) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

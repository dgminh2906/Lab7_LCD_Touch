/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "software_timer.h"
#include "led_7seg.h"
#include "button.h"
#include "lcd.h"
#include "picture.h"
#include "ds3231.h"
#include "sensor.h"
#include "buzzer.h"
#include "touch.h"
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
#define INIT 0
#define PLAYING 1
#define GAME_OVER 2

int play_Status = INIT;
int counter = 0;
// Kích thước lưới và màn hình
#define GRID_SIZE 20
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320

typedef struct {
    int x, y;
} Point;

Point snake[100];  // Tối đa 100 phần tử
int snakeLength = 3;
Point food;
int direction = 0;  // 0: Up, 1: Down, 2: Left, 3: Right
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void system_init();
void test_LedDebug();
void touchProcess();
void initGame();
void generateFood();
void drawSnake();
void drawFood();
int checkCollision();
void checkFood();
void moveSnake();
void handleDirection();
uint8_t isButtonStart();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_FSMC_Init();
  MX_I2C1_Init();
  MX_TIM13_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  system_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 touch_Adjust();
 lcd_Clear(BLACK);
 while (1)
  {
	  //scan touch screen
	  touch_Scan();
	  // 50ms task
	  if(flag_timer2 == 1){
		  flag_timer2 = 0;
		  touchProcess();
		  test_LedDebug();
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void system_init(){
	  timer_init();
	  button_init();
	  lcd_init();
	  touch_init();
	  setTimer2(50);
}

uint8_t count_led_debug = 0;

void test_LedDebug(){
	count_led_debug = (count_led_debug + 1)%20;
	if(count_led_debug == 0){
		HAL_GPIO_TogglePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);
	}
}

uint8_t isButtonStart(){
	if(!touch_IsTouched()) return 0;
	return touch_GetX() > 60 && touch_GetX() < 180 && touch_GetY() > 140 && touch_GetY() < 190;
}

void initGame() {
    // Khởi tạo rắn
    snakeLength = 3;
    snake[0].x = 5; snake[0].y = 5;
    snake[1].x = 5; snake[1].y = 6;
    snake[2].x = 5; snake[2].y = 7;
    direction = 0;

    // Khởi tạo thức ăn
    generateFood();
}

void generateFood() {
    food.x = rand() % (SCREEN_WIDTH / GRID_SIZE);
    food.y = rand() % (SCREEN_HEIGHT / GRID_SIZE);
}

void drawSnake() {
    for (int i = 0; i < snakeLength; i++) {
        lcd_Fill(snake[i].x * GRID_SIZE, snake[i].y * GRID_SIZE,
                 (snake[i].x + 1) * GRID_SIZE, (snake[i].y + 1) * GRID_SIZE, GREEN);
    }
}

void drawFood() {
    lcd_Fill(food.x * GRID_SIZE, food.y * GRID_SIZE,
             (food.x + 1) * GRID_SIZE, (food.y + 1) * GRID_SIZE, RED);
}

int checkCollision() {
    // Va cham voi tuong
    if (snake[0].x < 0 || snake[0].x >= SCREEN_WIDTH / GRID_SIZE ||
        snake[0].y < 0 || snake[0].y >= SCREEN_HEIGHT / GRID_SIZE) {
        return 1;
    }

    // Va cham chinh no
    for (int i = 1; i < snakeLength; i++) {
        if (snake[0].x == snake[i].x && snake[0].y == snake[i].y) {
            return 1;
        }
    }

    return 0;
}

void checkFood() {
    if (snake[0].x == food.x && snake[0].y == food.y) {
        snakeLength++;
        generateFood();
    }
}

void moveSnake() {
    // Di chuyen than ran
    for (int i = snakeLength - 1; i > 0; i--) {
        snake[i] = snake[i - 1];
    }

    // Di chuyen dau ran
    if (direction == 0) snake[0].y--;        // Lên
    else if (direction == 1) snake[0].y++;   // Xuống
    else if (direction == 2) snake[0].x--;   // Trái
    else if (direction == 3) snake[0].x++;   // Phải
}

void touchProcess() {
    switch (play_Status) {
        case INIT:
            lcd_Fill(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, BLACK);
            lcd_Fill(60, 140, 180, 190, GBLUE);
            lcd_ShowStr(90, 150, "START", WHITE, BLACK, 24, 1);
            if (isButtonStart()) {
                play_Status = PLAYING;
                initGame();
            }
            break;

        case PLAYING:
            lcd_Fill(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, BLACK);
			lcd_ShowStr(110, 20, "U", RED, BLACK, 24, 1);
			lcd_ShowStr(110, 270, "D", RED, BLACK, 24, 1);
			lcd_ShowStr(20, 160, "L", RED, BLACK, 24, 1);
			lcd_ShowStr(200, 160, "R", RED, BLACK, 24, 1);
			handleDirection();
			counter++;
			if (counter == 10){
				counter = 0;
				moveSnake();
			}
			if (checkCollision()) {
				play_Status = GAME_OVER;
			} else {
				checkFood();
				drawSnake();
				drawFood();
			}
            break;

        case GAME_OVER:
            lcd_Fill(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, BLACK);
            lcd_Fill(60, 140, 180, 190, GBLUE);
            lcd_ShowStr(70, 150, "GAME OVER", RED, BLACK, 24, 1);
            if (isButtonStart()) {
                play_Status = INIT;
            }
            break;

        default:
            break;
    }
}

void handleDirection() {
    if (touch_IsTouched()) {
        int x = touch_GetX();
        int y = touch_GetY();

        if (x > 70 && x < 150 && y > 0 && y < 70) direction = 0;  // Lên
        else if (x > 70 && x < 150 && y > 250 && y < 320) direction = 1;  // Xuống
        else if (x > 0 && x < 70 && y > 120 && y < 200) direction = 2;  // Trái
        else if (x > 170 && x < 240 && y > 120 && y < 200) direction = 3;  // Phải
    }
}

//void touchProcess(){
//	switch (play_Status) {
//		case INIT:
//			//Clear screen
//			lcd_Fill(0, 60, 240, 320, BLACK);
//			lcd_Fill(60, 140, 180, 190, GBLUE);
//			lcd_ShowStr(90, 20, "START", RED, BLACK, 24, 1);
//			if (isButtonStart()){
//				play_Status = PLAYING;
//			}
//			break;
//		case PLAYING:
//			//Clear screen
//			lcd_Fill(0, 60, 240, 320, BLACK);
//
//			lcd_Fill(90, 10, 130, 60, GREEN);
//			lcd_ShowStr(110, 20, "U", RED, BLACK, 24, 1);
//			lcd_Fill(90, 260, 130, 310, GREEN);
//			lcd_ShowStr(110, 270, "D", RED, BLACK, 24, 1);
//			lcd_Fill(10, 140, 50, 180, GREEN);
//			lcd_ShowStr(20, 160, "L", RED, BLACK, 24, 1);
//			lcd_Fill(190, 140, 230, 180, GREEN);
//			lcd_ShowStr(200, 160, "R", RED, BLACK, 24, 1);
//			if (isButtonStart()){
//				play_Status = CLEAR;
//			}
//			break;
//		case RETURN:
//			if(!touch_IsTouched()) play_Status = INIT;
//			//Clear screen
//			lcd_Fill(0, 60, 240, 320, BLACK);
//			break;
//		default:
//			break;
//	}
//}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

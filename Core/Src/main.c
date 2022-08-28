/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "usb_device.h"
#include <stdlib.h>
#include <search.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
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
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
char read_keypad(void);


void send_char_HID(uint8_t keycode, uint8_t modifier);
void release_key_HID(void);

void test_routine_one(void);
void test_routine_two(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern USBD_HandleTypeDef hUsbDeviceFS;

typedef struct{
	uint8_t MODIFIER;
    uint8_t RESERVED;
    uint8_t KEYCODE1;
    uint8_t KEYCODE2;
    uint8_t KEYCODE3;
    uint8_t KEYCODE4;
    uint8_t KEYCODE5;
    uint8_t KEYCODE6;
} keyboard_HID_t;

/* CHARACTER DEFINITIONS  */
#define RELEASE_USB_CHAR 	0x00
#define A_USB_CHAR 			0x04
#define B_USB_CHAR 			0x05
#define C_USB_CHAR 			0x06
#define D_USB_CHAR 			0x07
#define E_USB_CHAR          0x08
#define F_USB_CHAR          0x09
#define G_USB_CHAR          0x0A
#define H_USB_CHAR          0x0B
#define I_USB_CHAR          0x0C
#define J_USB_CHAR          0x0D
#define K_USB_CHAR          0x0E
#define L_USB_CHAR          0x0F
#define M_USB_CHAR          0x10
#define N_USB_CHAR          0x11
#define N_TILDE_USB_CHAR    0x33
#define O_USB_CHAR          0x12
#define P_USB_CHAR          0x13
#define Q_USB_CHAR          0x14
#define R_USB_CHAR          0x15
#define S_USB_CHAR          0x16
#define T_USB_CHAR          0x17
#define U_USB_CHAR          0x18
#define V_USB_CHAR          0x19
#define W_USB_CHAR          0x1A
#define X_USB_CHAR          0x1B
#define Y_USB_CHAR          0x1C
#define Z_USB_CHAR          0x1D

#define NUM_1_USB_CHAR      0x1E
#define NUM_2_USB_CHAR      0x1F
#define NUM_3_USB_CHAR      0x20
#define NUM_4_USB_CHAR      0x21
#define NUM_5_USB_CHAR      0x22
#define NUM_6_USB_CHAR      0x23
#define NUM_7_USB_CHAR      0x24
#define NUM_8_USB_CHAR      0x25
#define NUM_9_USB_CHAR      0x26
#define NUM_0_USB_CHAR      0x27

/* SPECIAL CHARACTER DEFINITIONS  */

#define USB_CHAR_parenthesis_l  0x25  	// (
#define USB_CHAR_parenthesis_r  0x26  	// )
#define USB_CHAR_spacebar       0x2C
#define USB_CHAR_right_bracket	0x30	// ]
#define USB_CHAR_asterisk       0x33  	// sin shift +, con shift *
#define USB_CHAR_semicolon		0x36	// sin shift , , con shift ;

/* MODIFIER DEFINITIONS  */

#define CLEAR       0x00
#define LEFT_SHIFT  0x02
#define RIGHT_SHIFT 0x20

keyboard_HID_t casai_keyboard = {0};

char keypad_char = '!';

// ENTRY e;
// ENTRY *ep;

// const char USB_chars[] = {
//   '1','2','3','4','5','6',
//   '7','8','9','0','A','B',
//   'C','E','F','G','H','I',
//   'J','K','L','M','N','O',
//   'P','Q','R','S','T','U',
//   'V','W','X','Y','Z'
// };

// const uint8_t KEYCODE_MAPPING[] = {
//   NUM_1_USB_CHAR,   NUM_2_USB_CHAR,   NUM_3_USB_CHAR,   NUM_4_USB_CHAR, NUM_5_USB_CHAR, NUM_6_USB_CHAR,
//   NUM_7_USB_CHAR,   NUM_8_USB_CHAR,   NUM_9_USB_CHAR,   NUM_0_USB_CHAR, A_USB_CHAR,     B_USB_CHAR,
//   C_USB_CHAR,       D_USB_CHAR,       E_USB_CHAR,       F_USB_CHAR,     G_USB_CHAR,     H_USB_CHAR, I_USB_CHAR,
//   J_USB_CHAR,       K_USB_CHAR,       L_USB_CHAR,       M_USB_CHAR,     N_USB_CHAR,     O_USB_CHAR,
//   P_USB_CHAR,       Q_USB_CHAR,       R_USB_CHAR,       S_USB_CHAR,     T_USB_CHAR,     U_USB_CHAR,
//   V_USB_CHAR,       W_USB_CHAR,       X_USB_CHAR,       Y_USB_CHAR,     Z_USB_CHAR,
// };


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


  //  const size_t capacity = sizeof(USB_chars) / sizeof(USB_chars[0]);
  //  hcreate(capacity);

  //  for (size_t i = 0; i < capacity - 2; i++) {
  //      e.key = USB_chars[i];
  //      e.data = (void *) KEYCODE_MAPPING[i];

  //      ep = hsearch(e, ENTER);

  //  }
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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  keypad_char = read_keypad();

	  if(keypad_char != '!'){
		HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
    switch (keypad_char){
      case '1': send_char_HID(NUM_1_USB_CHAR, CLEAR); break;
      case '2': send_char_HID(NUM_2_USB_CHAR, CLEAR); break;
      case '3': send_char_HID(NUM_3_USB_CHAR, CLEAR); break;
      case '4': send_char_HID(NUM_4_USB_CHAR, CLEAR); break;
      case '5': send_char_HID(NUM_5_USB_CHAR, CLEAR); break;
      case '6': send_char_HID(NUM_6_USB_CHAR, CLEAR); break;
      case '7': send_char_HID(NUM_7_USB_CHAR, CLEAR); break;
      case '8': send_char_HID(NUM_8_USB_CHAR, CLEAR); break;
      case '9': send_char_HID(NUM_9_USB_CHAR, CLEAR); break;
      case '0': send_char_HID(NUM_0_USB_CHAR, CLEAR); break;

      case 'A': send_char_HID(A_USB_CHAR, LEFT_SHIFT); break;
      case 'B': send_char_HID(B_USB_CHAR, LEFT_SHIFT); break;
      case 'C': send_char_HID(C_USB_CHAR, LEFT_SHIFT); break;
      case 'D': send_char_HID(D_USB_CHAR, LEFT_SHIFT); break;
      case 'E': send_char_HID(E_USB_CHAR, LEFT_SHIFT); break;
      case 'F': send_char_HID(F_USB_CHAR, LEFT_SHIFT); break;
      case 'G': send_char_HID(G_USB_CHAR, LEFT_SHIFT); break;
      case 'H': send_char_HID(H_USB_CHAR, LEFT_SHIFT); break;
      case 'I': send_char_HID(I_USB_CHAR, LEFT_SHIFT); break;
      case 'J': send_char_HID(J_USB_CHAR, LEFT_SHIFT); break;
      case 'K': send_char_HID(K_USB_CHAR, LEFT_SHIFT); break;
      case 'L': send_char_HID(L_USB_CHAR, LEFT_SHIFT); break;
      case 'M': send_char_HID(M_USB_CHAR, LEFT_SHIFT); break;
      case 'N': send_char_HID(N_USB_CHAR, LEFT_SHIFT); break;
      case 'O': send_char_HID(O_USB_CHAR, LEFT_SHIFT); break;
      case 'P': send_char_HID(P_USB_CHAR, LEFT_SHIFT); break;
      case 'Q': send_char_HID(Q_USB_CHAR, LEFT_SHIFT); break;
      case 'R': send_char_HID(R_USB_CHAR, LEFT_SHIFT); break;
      case 'S': send_char_HID(S_USB_CHAR, LEFT_SHIFT); break;
      case 'T': send_char_HID(T_USB_CHAR, LEFT_SHIFT); break;
      case 'U': send_char_HID(U_USB_CHAR, LEFT_SHIFT); break;
      case 'V': send_char_HID(V_USB_CHAR, LEFT_SHIFT); break;
      case 'W': send_char_HID(W_USB_CHAR, LEFT_SHIFT); break;
      case 'X': send_char_HID(X_USB_CHAR, LEFT_SHIFT); break;
      case 'Y': send_char_HID(Y_USB_CHAR, LEFT_SHIFT); break;
      case 'Z': send_char_HID(Z_USB_CHAR, LEFT_SHIFT); break;
      case '*': test_routine_one(); break;
      case '#': test_routine_two(); break;
      // case '*': send_char_HID(NUM_8_USB_CHAR, LEFT_SHIFT); break;
      // case '#': send_char_HID(NUM_3_USB_CHAR, LEFT_SHIFT); break;
      
      default: send_char_HID(X_USB_CHAR, LEFT_SHIFT); break;
    }

		// e.key = keypad_char;
		// ep = hsearch(e, FIND);
		// casai_keyboard.KEYCODE1 = (char)(ep->data);		

	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_ROW_4_Pin|GPIO_ROW_3_Pin|GPIO_ROW_2_Pin|GPIO_ROW_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_BUILTIN_Pin */
  GPIO_InitStruct.Pin = LED_BUILTIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BUILTIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_COLUMN_4_Pin GPIO_COLUMN_3_Pin GPIO_COLUMN_2_Pin GPIO_COLUMN_1_Pin */
  GPIO_InitStruct.Pin = GPIO_COLUMN_4_Pin|GPIO_COLUMN_3_Pin|GPIO_COLUMN_2_Pin|GPIO_COLUMN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_ROW_4_Pin GPIO_ROW_3_Pin GPIO_ROW_2_Pin GPIO_ROW_1_Pin */
  GPIO_InitStruct.Pin = GPIO_ROW_4_Pin|GPIO_ROW_3_Pin|GPIO_ROW_2_Pin|GPIO_ROW_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
char read_keypad(void){
	/* Make ROW 1 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin(GPIO_ROW_1_GPIO_Port, GPIO_ROW_1_Pin, GPIO_PIN_RESET);// Pull the R1 low
	HAL_GPIO_WritePin(GPIO_ROW_2_GPIO_Port, GPIO_ROW_2_Pin, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin(GPIO_ROW_3_GPIO_Port, GPIO_ROW_3_Pin, GPIO_PIN_SET);  // Pull the R3 High
	HAL_GPIO_WritePin(GPIO_ROW_4_GPIO_Port, GPIO_ROW_4_Pin, GPIO_PIN_SET);  // Pull the R4 High

	if(!(HAL_GPIO_ReadPin(GPIO_COLUMN_1_GPIO_Port, GPIO_COLUMN_1_Pin))){			// if the Col 1 is low
		while (!(HAL_GPIO_ReadPin (GPIO_COLUMN_1_GPIO_Port, GPIO_COLUMN_1_Pin)));   // wait till the button is pressed
    return '1';
	}

	if (!(HAL_GPIO_ReadPin (GPIO_COLUMN_2_GPIO_Port, GPIO_COLUMN_2_Pin))){
		while (!(HAL_GPIO_ReadPin (GPIO_COLUMN_2_GPIO_Port, GPIO_COLUMN_2_Pin)));   // wait till the button is pressed
    return '2';
	}

	if (!(HAL_GPIO_ReadPin (GPIO_COLUMN_3_GPIO_Port, GPIO_COLUMN_3_Pin))){
		while (!(HAL_GPIO_ReadPin (GPIO_COLUMN_3_GPIO_Port, GPIO_COLUMN_3_Pin)));   // wait till the button is pressed
    return '3';
	}

	if (!(HAL_GPIO_ReadPin (GPIO_COLUMN_4_GPIO_Port, GPIO_COLUMN_4_Pin))){
		while (!(HAL_GPIO_ReadPin (GPIO_COLUMN_4_GPIO_Port, GPIO_COLUMN_4_Pin)));   // wait till the button is pressed
		return 'A';
	}


	/* Make ROW 2 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin(GPIO_ROW_1_GPIO_Port, GPIO_ROW_1_Pin, GPIO_PIN_SET);  // Pull the R1 high
	HAL_GPIO_WritePin(GPIO_ROW_2_GPIO_Port, GPIO_ROW_2_Pin, GPIO_PIN_RESET);// Pull the R2 low
	HAL_GPIO_WritePin(GPIO_ROW_3_GPIO_Port, GPIO_ROW_3_Pin, GPIO_PIN_SET);  // Pull the R3 High
	HAL_GPIO_WritePin(GPIO_ROW_4_GPIO_Port, GPIO_ROW_4_Pin, GPIO_PIN_SET);  // Pull the R4 High

	if(!(HAL_GPIO_ReadPin(GPIO_COLUMN_1_GPIO_Port, GPIO_COLUMN_1_Pin))){			// if the Col 1 is low
		while (!(HAL_GPIO_ReadPin (GPIO_COLUMN_1_GPIO_Port, GPIO_COLUMN_1_Pin)));   // wait till the button is pressed	
    return '4';
	}

	if (!(HAL_GPIO_ReadPin (GPIO_COLUMN_2_GPIO_Port, GPIO_COLUMN_2_Pin))){
		while (!(HAL_GPIO_ReadPin (GPIO_COLUMN_2_GPIO_Port, GPIO_COLUMN_2_Pin)));   // wait till the button is pressed
		return '5';
	}

	if (!(HAL_GPIO_ReadPin (GPIO_COLUMN_3_GPIO_Port, GPIO_COLUMN_3_Pin))){
		while (!(HAL_GPIO_ReadPin (GPIO_COLUMN_3_GPIO_Port, GPIO_COLUMN_3_Pin)));   // wait till the button is pressed
    return '6';
	}

	if (!(HAL_GPIO_ReadPin (GPIO_COLUMN_4_GPIO_Port, GPIO_COLUMN_4_Pin))){
		while (!(HAL_GPIO_ReadPin (GPIO_COLUMN_4_GPIO_Port, GPIO_COLUMN_4_Pin)));   // wait till the button is pressed
		return 'B';
	}
	/* Make ROW 1 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin(GPIO_ROW_1_GPIO_Port, GPIO_ROW_1_Pin, GPIO_PIN_SET);// Pull the R1 High
	HAL_GPIO_WritePin(GPIO_ROW_2_GPIO_Port, GPIO_ROW_2_Pin, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin(GPIO_ROW_3_GPIO_Port, GPIO_ROW_3_Pin, GPIO_PIN_RESET);  // Pull the R3 low
	HAL_GPIO_WritePin(GPIO_ROW_4_GPIO_Port, GPIO_ROW_4_Pin, GPIO_PIN_SET);  // Pull the R4 High

	if(!(HAL_GPIO_ReadPin(GPIO_COLUMN_1_GPIO_Port, GPIO_COLUMN_1_Pin))){			// if the Col 1 is low
		while (!(HAL_GPIO_ReadPin (GPIO_COLUMN_1_GPIO_Port, GPIO_COLUMN_1_Pin)));   // wait till the button is pressed
    return '7';
	}

	if (!(HAL_GPIO_ReadPin (GPIO_COLUMN_2_GPIO_Port, GPIO_COLUMN_2_Pin))){
		while (!(HAL_GPIO_ReadPin (GPIO_COLUMN_2_GPIO_Port, GPIO_COLUMN_2_Pin)));   // wait till the button is pressed
    return '8';
	}

	if (!(HAL_GPIO_ReadPin (GPIO_COLUMN_3_GPIO_Port, GPIO_COLUMN_3_Pin))){
		while (!(HAL_GPIO_ReadPin (GPIO_COLUMN_3_GPIO_Port, GPIO_COLUMN_3_Pin)));   // wait till the button is pressed
    return '9';
	}

	if (!(HAL_GPIO_ReadPin (GPIO_COLUMN_4_GPIO_Port, GPIO_COLUMN_4_Pin))){
		while (!(HAL_GPIO_ReadPin (GPIO_COLUMN_4_GPIO_Port, GPIO_COLUMN_4_Pin)));   // wait till the button is pressed
		return 'C';
	}
	/* Make ROW 1 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin(GPIO_ROW_1_GPIO_Port, GPIO_ROW_1_Pin, GPIO_PIN_SET);// Pull the R1 High
	HAL_GPIO_WritePin(GPIO_ROW_2_GPIO_Port, GPIO_ROW_2_Pin, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin(GPIO_ROW_3_GPIO_Port, GPIO_ROW_3_Pin, GPIO_PIN_SET);  // Pull the R3 High
	HAL_GPIO_WritePin(GPIO_ROW_4_GPIO_Port, GPIO_ROW_4_Pin, GPIO_PIN_RESET);  // Pull the R4 low

	if(!(HAL_GPIO_ReadPin(GPIO_COLUMN_1_GPIO_Port, GPIO_COLUMN_1_Pin))){			// if the Col 1 is low
		while (!(HAL_GPIO_ReadPin (GPIO_COLUMN_1_GPIO_Port, GPIO_COLUMN_1_Pin)));   // wait till the button is pressed
    	return '*';
	}

	if (!(HAL_GPIO_ReadPin (GPIO_COLUMN_2_GPIO_Port, GPIO_COLUMN_2_Pin))){
		while (!(HAL_GPIO_ReadPin (GPIO_COLUMN_2_GPIO_Port, GPIO_COLUMN_2_Pin)));   // wait till the button is pressed
		return '0';
	}

	if (!(HAL_GPIO_ReadPin (GPIO_COLUMN_3_GPIO_Port, GPIO_COLUMN_3_Pin))){
		while (!(HAL_GPIO_ReadPin (GPIO_COLUMN_3_GPIO_Port, GPIO_COLUMN_3_Pin)));   // wait till the button is pressed
		return '#';
	}

	if (!(HAL_GPIO_ReadPin (GPIO_COLUMN_4_GPIO_Port, GPIO_COLUMN_4_Pin))){
		while (!(HAL_GPIO_ReadPin (GPIO_COLUMN_4_GPIO_Port, GPIO_COLUMN_4_Pin)));   // wait till the button is pressed
		return 'D';
	}

	return '!';
}

void send_char_HID(uint8_t keycode, uint8_t modifier){
  casai_keyboard.MODIFIER = modifier;
  casai_keyboard.KEYCODE1 = keycode;
  USBD_HID_SendReport(&hUsbDeviceFS, &casai_keyboard, sizeof(casai_keyboard));
  HAL_Delay(100);
  release_key_HID();
  HAL_Delay(20);
}

void release_key_HID(void){
		casai_keyboard.MODIFIER = RELEASE_USB_CHAR;
		casai_keyboard.KEYCODE1 = RELEASE_USB_CHAR;
		casai_keyboard.KEYCODE2 = RELEASE_USB_CHAR;
		casai_keyboard.KEYCODE3 = RELEASE_USB_CHAR;
		casai_keyboard.KEYCODE4 = RELEASE_USB_CHAR;
		casai_keyboard.KEYCODE5 = RELEASE_USB_CHAR;
		casai_keyboard.KEYCODE6 = RELEASE_USB_CHAR;
		USBD_HID_SendReport(&hUsbDeviceFS, &casai_keyboard, sizeof(casai_keyboard));
}

void test_routine_one(void){
    send_char_HID(H_USB_CHAR, LEFT_SHIFT);
    send_char_HID(E_USB_CHAR, CLEAR);
    send_char_HID(L_USB_CHAR, CLEAR);
    send_char_HID(L_USB_CHAR, CLEAR);
    send_char_HID(O_USB_CHAR, CLEAR);
    send_char_HID(USB_CHAR_spacebar, CLEAR);
    send_char_HID(W_USB_CHAR, CLEAR);
    send_char_HID(O_USB_CHAR, CLEAR);
    send_char_HID(R_USB_CHAR, CLEAR);
    send_char_HID(L_USB_CHAR, CLEAR);
    send_char_HID(D_USB_CHAR, CLEAR);
    send_char_HID(USB_CHAR_spacebar, CLEAR);
    send_char_HID(USB_CHAR_semicolon, LEFT_SHIFT);
    send_char_HID(USB_CHAR_parenthesis_r, LEFT_SHIFT);
}

void test_routine_two(void){
	casai_keyboard.MODIFIER = RELEASE_USB_CHAR;
	casai_keyboard.KEYCODE1 = C_USB_CHAR;
	casai_keyboard.KEYCODE2 = A_USB_CHAR;
	casai_keyboard.KEYCODE3 = S_USB_CHAR;
	casai_keyboard.KEYCODE4 = A_USB_CHAR;
	casai_keyboard.KEYCODE5 = I_USB_CHAR;
	casai_keyboard.KEYCODE6 = RELEASE_USB_CHAR;
	USBD_HID_SendReport(&hUsbDeviceFS, &casai_keyboard, sizeof(casai_keyboard));
	HAL_Delay(100);
	release_key_HID();
	HAL_Delay(20);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

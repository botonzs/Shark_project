/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "stdio.h"
#include "string.h"
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//#define FekpadTest
#define MotorVezerles
#define KuplungVezerles

//Fékpad változók:
int fekpadState=1;
int FekapadSzervoDeltaFi=0;
int FekapadSzervoFi=0;
uint16_t readValue;
uint8_t MSG[35] = "";

//Kuplungvezérlés változók:
enum ClutchStates
{
	Clutch_Open,
	If,
	PID,
	Sailing,
	Clutch_Closed
};
enum ClutchStates ClutchState = Clutch_Open;


#define clutch_Open 40		//A kuplung teljesen kinyitott állapota
#define clutch_Closed 65	//A kuplung teljesen bezárt állapota
#define clutch_Min 45		//A kuplung csúsztatásnál a minimum szög ami alá nem vezéreljük
#define clutch_Max 60		//A kuplung csúsztatásnál a maximum szög ami felé nem vezéreljük


//PID
int32_t S=0;	//Integrál értéke
int32_t P=0;	//P tag értéke
int32_t I=0;	//I tag értéke
int32_t D=0;	//D tag értéke
int32_t kp=100;	//P tag konstansa
int32_t ki=10;	//I tag konstansa
int32_t kd=0;	//D tag konstansa

int32_t eMot=0;					//Motor szöggyorsulása
int32_t epszilon=20;			//Elég kis különbség esetén bezárjuk a kuplungot
int32_t clutchServoAngle=40;	//PID esetén ezt a szöget írjuk a kuplung szervókra

//Motorvezérlés változók:
enum EngineStates
{
	Engine_Stop,
	Decompressor_Open,
	Wait_Decompressor_RPM,
	Wait_Injection_RPM,
	Injection_Start,
	Starter_Engine_Stop
};
enum EngineStates EngineState = Engine_Stop;

bool start_button=0;

int32_t nKer=0;				//Kerék szögsebessége
int32_t nMot=0;               //motor szögsebessége

#define DKopen 135     //dekompresszor nyitott szöge (aktivált)
#define DKclosed 90    //dekompresszor zárt szöge (deaktivált)
#define MotPosZero 60  //motorvezérlő szervó "nincs befecskendezés" szöge

//Befecskendezés:
const int32_t n[17]={0,150,1100,1200,1300,1400,1500,1600,1700,1800,1900,2000,2100,2200,2400,2600,10000};
const int32_t fi[16]={80,130,126,123,118,117,116,115,114,113,112,111,109,106,103,80};
int32_t currentMotPos;        	//ebbe a pozícióba állítjuk a motor szervót futás közben

//Fordulatszámok:
#define nV 300        			//vész szögsebesség, ha beesik futás közben a motor ez alá akkor leállítjuk
#define nDK 100        			//dekompresszort ezen a sebességen zárjuk be megint
#define nBef 200       			//befecskendezés ezen a fordulatszámon indul el
#define nInd 700      			//indítómotor ezen a sebességen áll meg
#define nMax 3000          		//maximum motorsebesség
#define nVit 400            	//ezen a kerék fordulatszámon azt mondjuk hogy már vitorlázunk
#define nC	1200				//erre a fordulatszámra szabályozza a kuplung a rendszert
#define nSafe_to_control 800	//ez alatt kinyitjuk a kuplungot biztos ami biztos

//Időzítők:
uint32_t StartWait_Dk=0;						//Dekompresszor nyitásának kezdete
uint32_t StartWait_EngineStart_failed=0;		//Motor indítás sikertelen, ha nyolc másodperc alatt nem történik meg
uint32_t Wait_Dk_millis=500;					//Dekompresszor nyitásának ideje
uint32_t Wait__EngineStart_failed_millis=8000;	//Motor indítás nyolc másodperce


//Szervó függvények: =======================================================================================================================================================
#define ServoMin 60
#define ServoMax 240
int16_t ServoInput(int32_t Degree) //servo motor bemenetének a generátora
{//bemenet fokban
	int16_t res = (ServoMax - ServoMin) / 180 * Degree + ServoMin; //PWM alsó határa a 0 fok felső határ a 180
    if(res>ServoMax) res=ServoMax;
    if(res<ServoMin) res=ServoMin;
    return res;
}
void Injection_Servo(int32_t Degree) //befecskendezést szabályozó szervó beállítása
{//bemenet fokban
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ServoInput(Degree));
}
void Decompressor_Servo(int32_t Degree) //dekompresszor szelepet szabályozó szervó beállítása
{//bemenet fokban
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ServoInput(Degree));
}
void Clutch_Servo(int32_t Degree)//kuplung szabályozó szervó beállítása
{//bemenet fokban; a két szervó egymással szemben mozog
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ServoInput(Degree));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ServoInput(180 - Degree));
}
//Szevó függvények vége =======================================================================================================================================================

//millis:=======================================================================================================================================================
// Clock frek= 84MHz, prescale=33600 -> Timer frek= 84MHz/33600 = 2500Hz
//Autoreload=60k -> 60k/2500=24 sec
//WaitLength: [ms]
//StartWait=(TIM4->CNT);
bool Tim4_Wait_Millis(int32_t StartWait, int32_t WaitLength)
{//Bemenet, hogy mikor kezdtük el a számolást, és hogy mennyi ideig számoljon
    if(((TIM4->CNT) + __HAL_TIM_GET_AUTORELOAD(&htim4) - StartWait) % __HAL_TIM_GET_AUTORELOAD(&htim4) > WaitLength*2.5){//__HAL_TIM_GET_AUTORELOAD(&htim4) asszem az autoreloadot adja vissza de nem száz
    return 1;
    }
    else return 0;
}
//millis vége=======================================================================================================================================================

//Gombok =======================================================================================================================================================
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	//Motorvezérlés:
	if(GPIO_Pin==GPIO_PIN_4){	//Bekapcsoló gomb
		start_button=true;
	}
	if(GPIO_Pin==GPIO_PIN_6){	//Kikapcsoló gomb
		start_button=false;
	}
}
//Gombok vége =======================================================================================================================================================

//50 ms =======================================================================================================================================================
uint32_t nMot_unsigned_bitmagic;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
	HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

	//Szögsebesség mérés
	nMot = (((TIM2->CNT))*1200)>>10; //egy fordulat 1024 jelet ad ki magából (256*4), és 50 ms-onként mérünk, de így 1/min-ben kapjuk meg (*1200)
	nKer = ((TIM1->CNT))*1200/1024; //egy fordulat 1024 jelet ad ki magából (256*4), és 50 ms-onként mérünk, de így 1/min-ben kapjuk meg (*1200)

	nMot_unsigned_bitmagic = TIM2->CNT;
	if(nMot_unsigned_bitmagic & 0x80000){
	nMot_unsigned_bitmagic = (((nMot_unsigned_bitmagic * 1200) >> 10) | 0xffc00000);
	}
	nMot=nMot_unsigned_bitmagic;

	__HAL_TIM_SET_COUNTER(&htim5, 0);
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	__HAL_TIM_SET_COUNTER(&htim1, 0);

	//sprintf(MSG,"nMot:%d\n",nMot_unsigned_bitmagic);
	//HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);

	//Szögsebesség mérés vége

#ifdef FekpadTest

#endif

#ifdef KuplungVezerles
	if(clutch_Min+clutchServoAngle>clutch_Min && clutch_Min+clutchServoAngle<clutch_Max){   //Csak akkor számoljuk tovább az integrált, ha a szervók nincsenek szélső helyzetben
		S=S +(nMot-nC);
		//S=S+dt*(nMot0-nC);  //kiemelhetünk dt-t és belevihetjük ki-be
	}
	P=kp*(nMot-nC);
	I=ki*S;
	D=kd*eMot;
	clutchServoAngle=P+I+D;
	clutchServoAngle=clutchServoAngle/1000;
#endif
}
//50 ms vége =======================================================================================================================================================

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  //Motorvez:
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
   HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
   HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1);

   //Fekpad:
   HAL_ADC_Start(&hadc1);
   HAL_ADC_Start(&hadc2);
   //Fékpad vége

   HAL_TIM_Base_Start(&htim4);
   HAL_TIM_Base_Start_IT(&htim5);


   __HAL_TIM_SET_COUNTER(&htim3, 500);
   //uint32_t StartWait=TIM4->CNT;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	   //Tesztek:
	 //__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4, 1000);
	  //Clutch_Servo(40);

	  //Tesztek vége

	  //Fékpad:
#ifdef FekpadTest
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  uint32_t value = HAL_ADC_GetValue(&hadc1);
	  value=(int)value*0.043945;

	  //Decompressor_Servo(value);
	  //Clutch_Servo(value);
	  Injection_Servo(value);
	  sprintf(MSG,"szog:%d\n",value);
	  HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);

	  if(start_button==false)fekpadState=1;
	  switch(fekpadState){
	  case 1:

		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
	  	  Decompressor_Servo(180);
	  	  if(start_button==true){
	  		  fekpadState=2;
	  		StartWait_Dk=TIM4->CNT;
	  	  }
	  break;
	  case 2:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
		  Decompressor_Servo(90);
	  	  if(Tim4_Wait_Millis(StartWait_Dk, Wait_Dk_millis)){
	  		fekpadState=3;
	  		StartWait_Dk=TIM4->CNT;
	  	  }
	  break;
	  case 3:
		  Decompressor_Servo(90);
	  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
	  	  if(Tim4_Wait_Millis(StartWait_Dk, Wait_Dk_millis)){
	  	  		fekpadState=4;
	  	  }
	  break;
	  case 4:
	  	  Decompressor_Servo(180);
	  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
	  	  HAL_Delay(500);
	  	  fekpadState=4;
	  break;
	  }
#endif
	 //Fékpad vége

	 //Motorvezérlés:
#ifdef MotorVezerles
	  //A tényleges állapotgép:
	  	          //->
	  //HAL_ADC_Start(&hadc1);
	  //HAL_ADC_PollForConversion(&hadc1, 1000);
	  //nMot = HAL_ADC_GetValue(&hadc1);
	  //sprintf(MSG,"nMot:%d\n",nMot);
	  //HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);

	  	      if (start_button == false || nMot>nMax) {
	  	          EngineState = Engine_Stop; //indítás állapotgépe
	  	          ClutchState = Clutch_Open; //kuplung állapotgépe
	  	      }

	  	      switch (EngineState) {
	  	          case Engine_Stop: //alapállapot:
	  	              Decompressor_Servo(DKclosed); 							//dekompresszor szelep zárt állapotba áll
	  	              Injection_Servo(MotPosZero); 								// ne legyen befecskendezés
	  	              HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET); 	//indító motor kikapcsol
	  	              //->
	  	              if (start_button == true){								//ha a start gombot megnyomjuk
	  	                EngineState = Decompressor_Open; 						//dekompresszor szelep nyit
	  	                StartWait_Dk=TIM4->CNT;
	  	                StartWait_EngineStart_failed=TIM4->CNT;
	  	              }
	  	          break;
	  	          //================================================================================
	  	          case Decompressor_Open: //dekompresszor nyit
	  	              Decompressor_Servo(DKopen);
	  	              Injection_Servo(MotPosZero);
	  	              //->
	  	              if (Tim4_Wait_Millis(StartWait_Dk, Wait_Dk_millis)) {		//Várunk fél másodpercet hogy kinyisson a dekompresszor
	  	                  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET); 	//indító motor bekapcsol
	  	                  EngineState = Wait_Decompressor_RPM;					//Motorpörgésre várunk
	  	              }
	  	          break;
	  	          //================================================================================

	  	          case Wait_Decompressor_RPM: //Megvárjuk hogy az indító felpörgesse a motort
	  	        	  Decompressor_Servo(DKopen);				//bezárjuk a dekompresszort
	  	        	  Injection_Servo(MotPosZero);
	  	        	  //->
	  	        	  if (nMot > nDK) { 						//Megvárjuk, hogy elérjük a befecskendezési fordulatot
	  	        		  EngineState = Wait_Injection_RPM; 	//Befecskendezés
	  	        	  }
	  	          break;


	  	          case Wait_Injection_RPM: //Megvárjuk hogy az indító felpörgesse a motort
	  	              Decompressor_Servo(DKclosed);				//bezárjuk a dekompresszort
	  	              Injection_Servo(MotPosZero);
	  	              //->
	  	              if (nMot > nBef) { 						//Megvárjuk, hogy elérjük a befecskendezési fordulatot
	  	                  EngineState = Injection_Start; 		//Befecskendezés
	  	              }
	  	          break;
	  	          //================================================================================
	  	          case Injection_Start : //elkezdődik a befecskendezés
	  	              Decompressor_Servo(DKclosed);
	  	              for (int ii = 0; ii < sizeof(fi)/sizeof(fi[0]); ii++) {
	  	                  if (nMot > n[ii] && nMot < n[ii + 1]) {
	  	                      currentMotPos = fi[ii];
	  	                  }

	  	              }
	  	              Injection_Servo(currentMotPos);					//Befecskendezőt beállítjuk
	  	              //->
	  	              if (nMot > nInd) {
	  	                  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET); //indító motor kikapcsol
	  	                  EngineState = Starter_Engine_Stop;
	  	              }
	  	          break;
	  	          //================================================================================
	  	          case Starter_Engine_Stop: //elindult a motor, indító leáll
	  	              Decompressor_Servo(DKclosed);
	  	              for (int i = 0; i < sizeof(fi) / sizeof(fi[0]); i++) {
	  	                  if (nMot > n[i] && nMot < n[i + 1]) {
	  	                      currentMotPos = fi[i];
	  	                  }
	  	                  Injection_Servo(currentMotPos);
	  	              }
	  	              Injection_Servo(currentMotPos);
	  	              //->
	  	              if (nMot < nV) {
	  	                  EngineState = Engine_Stop;
	  	                start_button = false;
	  	              }
	  	          break;
	  	      }

	  	      if (Tim4_Wait_Millis(StartWait_EngineStart_failed, Wait__EngineStart_failed_millis) && EngineState != Starter_Engine_Stop) {
	  	    	start_button = false;
	  	        EngineState = Engine_Stop;
	  	        ClutchState = Clutch_Open;
	  	      }
#endif
	 //Motorvezérlés vége

#ifdef KuplungVezerles
	  	    switch (ClutchState) {
	  	      case Clutch_Open: //Clutch open
	  	    	Clutch_Servo(clutch_Open);

	  	        S = 0;		//Az integrált reseteljük a PID-ben

	  	        //->
	  	        if(start_button == true){
	  	        	ClutchState=If;
	  	        }
	  	        break;
	  	    //================================================================================
	  	      case If: //if
	  	        if(nKer<nMot){
	  	            ClutchState = PID;
	  	        }
	  	        else{
	  	        	ClutchState = Sailing;
	  	        }
	  	        break;
	  	    //================================================================================
	  	      case PID: //Csúsztatás
	  	    	if(nMot>nSafe_to_control){//Ha túlságosan lemegy a motorfordulatszám, akkor kinyitjuk a kuplungot
	  	    		ClutchState = Clutch_Open;

	  	    		if(clutch_Min+clutchServoAngle>clutch_Min && clutch_Min+clutchServoAngle<clutch_Max){   //Csak akkor változtatjuk a szervók állását, ha az nem mozgatja ki őket a szélső értéken
	  	    			Clutch_Servo(clutch_Min+clutchServoAngle);//A PID kiszámolt értékét ráírjuk a kuplung szervókra
	  	            }
	  	    	}
	  	    	else Clutch_Servo(clutch_Open);//Kinyitjuk a kuplungot, mert túl alacsony a motor fordulatszáma

	  	            //->
	  	            if(nMot-nKer<epszilon){
	  	            	ClutchState = Clutch_Closed;
	  	            }
	  	        break;
	  	    //================================================================================
	  	      case Sailing: //Vitorlázás
	  	    	Clutch_Servo(clutch_Open);

	  	        //->
	  	        if(nMot>=nKer){
	  	        	ClutchState = Clutch_Closed;
	  	        }
	  	        break;
	  	    //================================================================================
	  	      case Clutch_Closed: //Clutch closed
	  	        Clutch_Servo(clutch_Closed);
	  	        break;
	  	      }

#endif

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 840;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 33600;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 60000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 8400;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 500;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE4 PE5 PE6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

 4.2. Configurez un template de code avec CubeIDE (CubeMX)
 4.2.1. Timers

On a utiliser TIMER 2 et TIMER 3
Frequence : 

TIM2, TIM3, TIM4, TIM5 (APB1 timers) → 84 MHz
TIM1, TIM9, TIM10, TIM11 (APB2 timers) → 84 MHz

4.2.2. Connectivity

I²C = Inter-Integrated Circuit
→ Un bus série 2 fils seulement :
SCL = Serial Clock Line (la ligne d’horloge, générée par le maître)
SDA = Serial Data Line (la ligne de données, bidirectionnelle)

4.3. Test d'une application simpliste

L’attribut  __weak est utilisé pour fournir une implémentation par défaut d’une fonction, que l’utilisateur peut redéfinir ailleurs dans son code
Ici le mot weak est utilse car on redefinit une fonction qui existe deja.

Le tick système timer (SysTick) est un simple compteur décroissant 24 bits qui produit un petit quantum de temps fixe. Le Soft utilise le SysTick pour créer des delays ou ou produire des interruptions périodique et executer un traitement de façon répété dans le temps.

Le compteur SysTick compte de facon décroissante d'une valeur N-1 jusqu'à 0. Le processeur génère une interruption lorsque le SysTick atteint la valeur 0.
Après avoir atteint la valeur 0, Le SysTick charge la valeur stocké dans un registre spécial appelée le SysTick Reload Register et continue de compter en décroissant à nouveau.
Le SysTick ne s'arrête jamais de compter même quand le processeur est en mode pause (debug session). Les interruptions continuent même en mode debug.

```c
/* TIM3 init function */
void MX_TIM3_Init(void)
{
 
  /* USER CODE BEGIN TIM3_Init 0 */
 
  /* USER CODE END TIM3_Init 0 */
 
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
 
  /* USER CODE BEGIN TIM3_Init 1 */
 
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
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
  /* USER CODE BEGIN TIM3_Init 2 */
 
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
 
}
 
 
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{
 
  if(tim_pwmHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */
 
  /* USER CODE END TIM3_MspInit 0 */
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
  /* USER CODE BEGIN TIM3_MspInit 1 */
 
  /* USER CODE END TIM3_MspInit 1 */
  }
}
 
 
 
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{
 
  if(tim_pwmHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */
 
  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  /* USER CODE BEGIN TIM3_MspDeInit 1 */
 
  /* USER CODE END TIM3_MspDeInit 1 */
  }
}
```

```c
void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
 
  /* Configuration de l’instance TIM3 */
  htim3.Instance = TIM3;                         // Utilisation du timer 3
  htim3.Init.Prescaler = 0;                      // Pas de préscaler (compteur = fréquence d’horloge timer)
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;   // Le compteur s’incrémente vers le haut
  htim3.Init.Period = 65535;                     // Valeur max du compteur (période PWM)
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // Pas de division supplémentaire
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; // Pas de préchargement ARR
 
  // Initialisation du mode PWM sur TIM3
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler(); // Gestion d’erreur si init échoue
  }
 
  /* Configuration maître/esclave du timer */
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;      // Pas de déclenchement externe
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE; // Pas d’esclavage
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
 
  /* Configuration de la sortie PWM */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;            // Mode PWM1 (rapport cyclique défini par CCRx)
  sConfigOC.Pulse = 0;                           // Valeur initiale du rapport cyclique = 0%
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;    // Polarité active à l’état haut
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;     // Mode rapide désactivé
 
  // Configuration canal 1 du PWM (PA6 → TIM3_CH1)
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  // Configuration canal 2 du PWM (PA7 → TIM3_CH2)
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
 
  // Post-initialisation pour configurer les GPIO associés aux sorties
  HAL_TIM_MspPostInit(&htim3);
}
```

$ f_PWM ​= ftimer​​ / (PSC+1)×(ARR+1) $
Formule de la fréquence de PWM

# 6 - Lecture du codeur

`stm32f4xx_hal_msp.`

```c
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
 
/* Inclusion du fichier principal pour accéder aux définitions globales */
#include "main.h"

/**
  * @brief  Initialisation du MSP global (Microcontroller Support Package)
  *         Cette fonction est appelée au démarrage pour initialiser
  *         les horloges systèmes et les interruptions de base.
  */
void HAL_MspInit(void)
{
 
  /* USER CODE BEGIN MspInit 0 */
 
  /* USER CODE END MspInit 0 */
  /* Activation de l’horloge du contrôleur SYSCFG (configuration système) */
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* Activation de l’horloge du contrôleur PWR (alimentation) */
  __HAL_RCC_PWR_CLK_ENABLE();
 
}
```

`tim.c`

```c
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "tim.h"

/* Déclaration du handle du timer 2 (utilisé pour le codeur) */
TIM_HandleTypeDef htim2;
/* Déclaration du handle du timer 2 (utilisé pour le PWM) */
TIM_HandleTypeDef htim3;
 
/* TIM2 init function */
void MX_TIM2_Init(void)
{
 
  /* USER CODE BEGIN TIM2_Init 0 */
 
  /* USER CODE END TIM2_Init 0 */
 
 /* Structure de configuration pour le mode encodeur */
  TIM_Encoder_InitTypeDef sConfig = {0};

  /* Structure pour la configuration maître/esclave */
  TIM_MasterConfigTypeDef sMasterConfig = {0};
 
  /* USER CODE BEGIN TIM2_Init 1 */
 
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;              // Sélection du périphérique TIM2 
  htim2.Init.Prescaler = 0;          // Pas de préscaler : le timer compte à la même fréquence que l’horloge du bus 

  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;  // Le compteur compte vers le haut
  htim2.Init.Period = 4294967295;     // Valeur maximale du compteur (32 bits pour TIM2)
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;  // Division d’horloge interne = 1 (pas de division)
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  // Pas de préchargement automatique
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;  // Mode encodeur : le timer est configuré pour lire les deux signaux A/B du codeur

  /* Configuration de la première entrée (canal 1 du timer) */
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;  // Détection sur front montant
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;  // Entrée directe reliée au codeur
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;   // Pas de division sur la mesure
  sConfig.IC1Filter = 0;                   // Pas de filtrage (signal brut)

  /* Configuration de la deuxième entrée (canal 2 du timer) */
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;  // Détection sur front montant
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;  // Entrée directe reliée au codeur
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;       // Pas de division
  sConfig.IC2Filter = 0;                      // Pas de filtrage

  /* Initialisation du timer 2 en mode encodeur */
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();  // Appel de la fonction d’erreur si l’initialisation échoue
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
 
}


void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* tim_encoderHandle)
{
 
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Vérifie que le handle correspond bien au timer 2 */
  if(tim_encoderHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */
 
  /* USER CODE END TIM2_MspInit 0 */
    /* Activation de l’horloge du périphérique TIM2 */
    __HAL_RCC_TIM2_CLK_ENABLE();
 
    /* Activation de l’horloge du port GPIOA (où sont les broches du codeur) */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /** Configuration des broches PA0 et PA1 comme entrées du codeur :
     *  - PA0 : TIM2_CH1 (canal 1)
     *  - PA1 : TIM2_CH2 (canal 2)
     */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1; // Sélection des deux broches
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;      // Mode alternatif (liaison timer)
    GPIO_InitStruct.Pull = GPIO_NOPULL;          // Pas de résistance interne
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Vitesse faible suffisante
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;   // Fonction alternative : TIM2
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);      // Initialisation des GPIO
 
  /* USER CODE BEGIN TIM2_MspInit 1 */
 
  /* USER CODE END TIM2_MspInit 1 */
  }
}

void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef* tim_encoderHandle)
{
 
  if(tim_encoderHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */
 
  /* USER CODE END TIM2_MspDeInit 0 */
    /* Désactivation de l’horloge du timer 2 */
    __HAL_RCC_TIM2_CLK_DISABLE();
 
    /**TIM2 GPIO Configuration
    PA0-WKUP     ------> TIM2_CH1
    PA1     ------> TIM2_CH2
    */
    /* Désinitialisation des broches PA0 et PA1 utilisées pour le codeur */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);
 
  }
}
```

- `Encoder_init()` 
```c

```

- `Encoder_Read()` 
```c

```
# 7 -  Écriture d'un correcteur PID

## 7.1 - Rappel et implémentation numérique simpliste

## 7.2 - Développement d'un PID que l'on puisse instancier aisément

**À propos de la plage de `output`** : 

On utilise cette sortie dans :
```c
Motor_Pwm_Update(pidHandle->process.output);<br>
```
Alors la plage recommandée est : $-1.0 ≤ output ≤ +1.0 $ car la fonction `Motor_Pwm_Update()` interprète ces bornes comme les rapports cycliques extrêmes du PWM moteur.

- ` pid.c`
```c

```
# 8 - Mise en oeuvre du PID pour asservir le moteur en vitesse

## 8.1 -Implémentation

```c

```

20s 
34 tour 

windup = 1/ki (donc si pas de windup, pas de Ki)

Kp = 0.25
Ki = 0.01
Kd = 0.001
windup = 100
error = 0
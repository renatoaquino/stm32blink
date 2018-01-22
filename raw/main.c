/**
 * Teste de programação do STM32F103C8T6 utilizando
 * apenas os endereços dos registradores.
 * 
 * Este programa deverá servir de referencia para futuros
 * códigos que utilizem os timers desta MCU.
 * 
 * A idéia de utilizar apenas os registradores é reduzir a 
 * quantidade de código (bytes) necessários para tal ação,
 * já que o uso do CMSIS embora fácil consuma muita memória
 * flash do dispositivo.
 * 
 **/

/**
 * Definições de tipos utilizados no código
 **/
#define __IO volatile
typedef __UINT8_TYPE__ uint8_t;
typedef __UINT16_TYPE__ uint16_t;
typedef unsigned int uint32_t;

//Wait For Interrupt
static void __WFI() { __asm ("wfi"); }

/**
 * Endereçamentos
 * 
 * O MCU STM32 possui dois barramentos, APB1 e ABP2.
 * O Timer que utilizaremos TIM2 fica no ABP1.
 * O endereço base para o ABP1 é 0x40000000
 * O TIM2 é o primeiro conjunto de bits deste endereço.
 **/
#define PERIPH_BASE     ((uint32_t)0x40000000)
#define APB1PERIPH_BASE PERIPH_BASE 
#define APB2PERIPH_BASE (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE  (PERIPH_BASE + 0x20000)
#define SCS_BASE (0xE000E000) /*!< System Control Space Base Address */

#define TIM2_BASE (APB1PERIPH_BASE + 0x0000) 
#define GPIOC_BASE (APB2PERIPH_BASE + 0x1000)
#define RCC_BASE (AHBPERIPH_BASE + 0x1000)
#define NVIC_BASE (SCS_BASE +  0x0100)  /*!< NVIC Base Address */

#define GPIO_Pin_13 ((uint16_t)0x2000)
#define GPIO_Mode_Out_PP ((uint16_t)0x10) 
#define GPIO_Speed_2MHz ((uint16_t)2) 

/**
 * Estrutura dos registradores de um TIMER
 **/
typedef struct
{
  __IO uint16_t CR1;
  uint16_t  RESERVED0;
  __IO uint16_t CR2;
  uint16_t  RESERVED1;
  __IO uint16_t SMCR;
  uint16_t  RESERVED2;
  __IO uint16_t DIER;
  uint16_t  RESERVED3;
  __IO uint16_t SR;
  uint16_t  RESERVED4;
  __IO uint16_t EGR;
  uint16_t  RESERVED5;
  __IO uint16_t CCMR1;
  uint16_t  RESERVED6;
  __IO uint16_t CCMR2;
  uint16_t  RESERVED7;
  __IO uint16_t CCER;
  uint16_t  RESERVED8;
  __IO uint16_t CNT;
  uint16_t  RESERVED9;
  __IO uint16_t PSC;
  uint16_t  RESERVED10;
  __IO uint16_t ARR;
  uint16_t  RESERVED11;
  __IO uint16_t RCR;
  uint16_t  RESERVED12;
  __IO uint16_t CCR1;
  uint16_t  RESERVED13;
  __IO uint16_t CCR2;
  uint16_t  RESERVED14;
  __IO uint16_t CCR3;
  uint16_t  RESERVED15;
  __IO uint16_t CCR4;
  uint16_t  RESERVED16;
  __IO uint16_t BDTR;
  uint16_t  RESERVED17;
  __IO uint16_t DCR;
  uint16_t  RESERVED18;
  __IO uint16_t DMAR;
  uint16_t  RESERVED19;
} TIM_TypeDef;

#define TIM2 ((TIM_TypeDef *) TIM2_BASE)

/**
 * Definição do layout do registrador RCC
 * Valido para o STM32F103C8T6
 **/
typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t CFGR;
  __IO uint32_t CIR;
  __IO uint32_t APB2RSTR;
  __IO uint32_t APB1RSTR;
  __IO uint32_t AHBENR;
  __IO uint32_t APB2ENR;
  __IO uint32_t APB1ENR;
  __IO uint32_t BDCR;
  __IO uint32_t CSR;
} RCC_TypeDef;

#define RCC                 ((RCC_TypeDef *) RCC_BASE)

/**
 * Definição de layout do registrador de um GPIO
 **/
typedef struct
{
  __IO uint32_t CRL;
  __IO uint32_t CRH;
  __IO uint32_t IDR;
  __IO uint32_t ODR;
  __IO uint32_t BSRR;
  __IO uint32_t BRR;
  __IO uint32_t LCKR;
} GPIO_TypeDef;

#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)

/**
  memory mapped structure for Nested Vectored Interrupt Controller (NVIC)
 */
typedef struct
{
  __IO uint32_t ISER[8];                      /*!< Offset: 0x000  Interrupt Set Enable Register           */
       uint32_t RESERVED0[24];                                   
  __IO uint32_t ICER[8];                      /*!< Offset: 0x080  Interrupt Clear Enable Register         */
       uint32_t RSERVED1[24];                                    
  __IO uint32_t ISPR[8];                      /*!< Offset: 0x100  Interrupt Set Pending Register          */
       uint32_t RESERVED2[24];                                   
  __IO uint32_t ICPR[8];                      /*!< Offset: 0x180  Interrupt Clear Pending Register        */
       uint32_t RESERVED3[24];                                   
  __IO uint32_t IABR[8];                      /*!< Offset: 0x200  Interrupt Active bit Register           */
       uint32_t RESERVED4[56];                                   
  __IO uint8_t  IP[240];                      /*!< Offset: 0x300  Interrupt Priority Register (8Bit wide) */
       uint32_t RESERVED5[644];                                  
  __IO uint32_t STIR;                         /*!< Offset: 0xE00  Software Trigger Interrupt Register     */
}  NVIC_Type;    

#define NVIC ((NVIC_Type *) NVIC_BASE) /*!< NVIC configuration struct */

/**
 * Registradores do TIM
 **/
#define  TIM_CR1_DIR                         ((uint16_t)0x0010)            /*!< Direction */

#define  TIM_CR1_CMS                         ((uint16_t)0x0060)            /*!< CMS[1:0] bits (Center-aligned mode selection) */

#define  TIM_CR1_CKD                         ((uint16_t)0x0300)            /*!< CKD[1:0] bits (clock division) */

#define TIM_SR_UIF ((uint16_t)0x0001)
#define TIM_IT_Update ((uint16_t)0x0001)
#define TIM_DIER_UIE  ((uint16_t)0x0001) /*!< Update interrupt enable */
#define TIM2_IRQn 28
/**
 * Configurações para o TIMER
 **/
#define TIM_CounterMode_Up                 ((uint16_t)0x0000)
#define TIM_CKD_DIV1                       ((uint16_t)0x0000)
#define TIM_PSCReloadMode_Immediate        ((uint16_t)0x0001)
#define TIM_CR1_CEN                       ((uint16_t)0x0001)

#define SYSCLK_FREQ_72MHz  72000000


/**
 * Registradores do RCC 
 **/

#define RCC_APB1Periph_TIM2              ((uint32_t)0x00000001)
#define RCC_APB2Periph_GPIOC             ((uint32_t)0x00000010)

uint8_t state = 0;

void TIM2_IRQHandler(void) 
{
    if(TIM2->SR & TIM_SR_UIF) { // overflow
        state ^= 0x01;
        if(state)
        {
            //Led On
            GPIOC->BRR = GPIO_Pin_13;
        }else{
            //Led Off
            GPIOC->BSRR = GPIO_Pin_13;
        }
    }
    TIM2->SR = (uint16_t)~TIM_IT_Update;
}

int main(void)
{
    //Inicializando o GPIO PC13 onde o LED da placa se encontra.
    //Inicializa o Clock para o ABP2 / GPIOC
    RCC->APB2ENR |= RCC_APB2Periph_GPIOC;

    //Inicializa o GPIO necessário para utilizar o PC13, Led da placa.
    //O pino 13 fica no registrador de portas altas, os 8 primeiros são
    //no registrador de portas baixas CRL
    //CHR é um registrador de 32 bits que pode ser separado de 4 em 4.
    //Estes 4 bits devem ser separados em 2 pares, sendo o de maior endereço
    //os bits referentes a configuração de operação da porta e os dois
    //subsequentes os de modo de operação.
    //As operações são INPUT, OUTPUT, ALTERNATE FUNCTION, etc.
    //Também é determinado por estes bits o tratamento da porta: 
    // PUSH PULL, OPEN DRAIN, ALTERNATE FUNCTION PUSH PULL E OPEN DRAIN
    //Nos bits do segundo grupo é determinado a velocidade 10MHz, 2MHz ou 50Mhz.
    //No caso queremos setar a porta 13 para output modo push pull a 2MHz.
    // CRH 0000 0000 0010 0000 0000 0000 0000 0000 = 0x200000
    GPIOC->CRH |= (uint32_t) 0x200000;

    //Desliga se estava ligado.
    GPIOC->BSRR = GPIO_Pin_13;


    //Inicializa o Clock para o ABP1 / TIM2
    RCC->APB1ENR |= RCC_APB1Periph_TIM2;

    TIM2->CR1 &= (uint16_t)(~((uint16_t)(TIM_CR1_DIR | TIM_CR1_CMS)));
    TIM2->CR1 |= (uint32_t) TIM_CounterMode_Up;
    TIM2->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CKD));
    TIM2->CR1 |= (uint32_t) TIM_CKD_DIV1;
    
    //O clock será dividido em 7200 partes (0..7199), sendo 72MHz, teremos 10MHz ou seja, 10000 ciclos por segundo.
    //Considerando que a ABP1 tem acesso ao clock cheio.
    //TIM2->PSC = SYSCLK_FREQ_72MHz/1000 - 1;
    //Não é possível utilizar um valor maior porque o registrador
    //PSC é um uint16_t, logo temos que fazer malabarismo
    //com as combinações de clock/prescalar/arr
    TIM2->PSC = 7199;
    //Recarrega o contador após 10000 iterações 0...9999
    //Usando os 10000 ciclos por segundo temos um update por segundo.
    TIM2->ARR = 9999;
    //Habilita as interrupções
    TIM2->DIER = TIM_DIER_UIE;

    //Como o clock se comporta ao concluir um ciclo?
    //Neste caso mandamos recarregar imediatamente.
    TIM2->EGR = TIM_PSCReloadMode_Immediate;  

    //Ligando o timer
    TIM2->CR1 |= TIM_CR1_CEN;           
    
    //Inicializa a interrupção do TIM2
    NVIC->ISER[0] = 0x10000000;
    
    //Loop infinito aguardando interrupção
    while(1) __WFI();
}

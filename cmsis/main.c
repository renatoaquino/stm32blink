#include <stm32f10x.h>
#include <stm32f10x_rcc.h> 
#include <stm32f10x_gpio.h> 
#include <stm32f10x_tim.h> 
#include <misc.h>

//Flag para tratamento do status do LED
uint8_t state = 0;

//Esta função é executada sempre que uma interrupção
//é lançada pelo timer TIM2.
void TIM2_IRQHandler(void) 
{
    //Caso a interrupção seja causada por um overflow
    //ou seja, contamos o valor esperado.
    if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET)
    {
        //Calcula o novo status para o LED
        state ^= 0x1;
        if(state)
        {
            //Liga o LED
            GPIO_ResetBits(GPIOC,GPIO_Pin_13);
        }else{
            //Desliga o LED
            GPIO_SetBits(GPIOC,GPIO_Pin_13);
        }
        //Como realizamos o trabalho necessário,
        //evitamos que esta interrupção siga sendo
        //avaliada por demais partes do CMSIS.
        //Para isso só precisamos limpar o BIT de interrupção
        //pendente de overflow.
        TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
    }
}

int main(void)
{
    //Inicializando o GPIO PC13 onde o LED da placa se encontra.
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);   

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //Prepara o NVIC, vetor de interrupções, para trabalhar
    //com o temporizador 2
    NVIC_EnableIRQ(TIM2_IRQn)
    
    //Configura e habilita o timer TIM2

    //Habilita o clock do TIM2 no ABP1 já que é onde ele se encontra.
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // Configure Timer
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
    //Configura o divisor do timer para 1000 ciclos por segundo.
    //O clock do sistema roda a 72Mhz, logo:
    //72000000 / 720000 = 1000 ciclos/seg
    TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock/100 - 1; 
    //Contamos até 1000 (0..999), assim temos 1000 contagens por segundo.
    //Com isso teremos um overflow por segundo.
    TIM_TimeBaseStructure.TIM_Period = 1000 - 1; // 0..999
    //Queremos que o contador conte de 0 até 999
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    //Não queremos nenhum divisor de clock, o calculo é com os 72Mhz.
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  
    //Zerando tudo.
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    //Passando para os registradores nossa configuração.
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    //Habilitando a atualização do bit de interrupção para o overflow.
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    //Ligando o timer
    TIM_Cmd(TIM2, ENABLE);


    //Loop infinito, entra em modo de espera por interrupções.
    //Wait For Interrupt == WFI
    while(1) __WFI(); 
}

//Requisito do CMSIS para tratamento de erros.
void assert_failed(uint8_t* file, uint32_t line) {
    /* Infinite loop */
    /* Use GDB to find out why we're here */ 
    while (1);
} 

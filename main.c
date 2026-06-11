/*
 ----------------- Sistema Automatico de Control de Iluminacion de un Ambiente  -----------------

Descripcion:
- ADC (canal 0, P0.23): digitaliza señal del sensor TEMT6000        		  Cable blanco
- GPDMA (canal 0): almacena automaticamente las conversiones ADC
- Timer0 (MAT0.1): genera el trigger periodico para el ADC
- Timer1 (MAT1.0 y MAT1.1): genera señal PWM hacia el gate del MOSFET
- DAC (P0.26): emite voltaje proporcional a los lux medidos en mV             Cable azul
- UART1 (P0.15 TX, P0.16 RX): recibe porcentaje deseado y reporta estado      Tx Cable rojo , Rx Cable verde
- P0.0: salida PWM hacia MOSFET                                               Cable morado
- EINT0: (P2.10) Detiene y arranca el sistema                                 Cable negro Eint0 boton

Logica del sistema:

1. El ADC muestrea el sensor de luz cada 1ms (1 kHz)
2. El DMA almacena BUFFER_TAM muestras en SRAM
3. Al completar el bloque, el ISR del DMA calcula el promedio - 64 ms
4. Se calcula el porcentaje de luz ambiente respecto al maximo calibrado
5. Se calcula el error = porcentaje deseado - porcentaje medido
6. El error se convierte en duty PWM (Timer1) y voltaje DAC
7. Por UART se configura el valor deseado y se reporta el estado

CCLK = 100 MHz, PCLK = 25 MHz
*/

// ===== INCLUDE =====

#include "LPC17xx.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_exti.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_uart.h"
#include <string.h>                 //para manejar cadenas de caracteres, ej strlen()
#include <stdint.h>                 // para variables de tipo entero con tam fijo 

// ===== Defines =====
#define BUFFER_ADC_ADDR 0x2007C000
#define BUFFER_TAM 64

#define UART_PC UART1
#define RX_BUFFER_SIZE 16

#define TIM1_PERIODO 4999	        // cada tick 1uS, = 5 mS de periodo PWM
#define ERROR_ZONA_MUERTA 2		    // para evitar que funcionen mal los leds cerca del valor deseado

#define LUX_MAXIMO 776              // lux correspondientes al 100% del foco
#define MV_MAXIMO 3133              // mV correspondientes al 100% 


// ===== Variables globales =====

volatile uint8_t  sistema_activo = 0;			// 0 = Stop, 1 = Activo
volatile uint16_t adc_promedio = 0;

volatile uint8_t  porcentaje_deseado = 50;
volatile uint8_t  porcentaje_ambiente = 0;
volatile int16_t  error_luz = 0;
volatile uint8_t  duty_pwm = 0;                 // duty aplicado en el mr1, se actualiza dentro del timer1 handler (el del pwm), en la int de mr0 para evitar cambiar el duty a mitad de ciclo
volatile uint8_t  duty_pwm_pendiente = 0;		// valor del duty q esta esperando a ser aplicado, no necesariamente esta activo, no actualiza el mr1

volatile uint16_t lux_ambiente = 0;
volatile uint16_t mv_ambiente = 0;
volatile uint16_t lux_deseado = 0;
volatile uint16_t mv_deseado = 0;

volatile uint8_t  enviar_estado = 0;
volatile uint8_t  aviso_arranque = 0;			// flag para el mensaje de UART
volatile uint32_t contador_2_segundos = 0;

volatile char rx_buffer[RX_BUFFER_SIZE];        // buffer para guardar los caracteres ingresados 
volatile uint8_t  rx_index = 0;                 // index para el buffer 
volatile uint8_t  comando_listo  = 0;           // flag que indica si ya esta el comando listo para procesar


// ==== Funciones ====
void confPin(void);
void configDAC(void);
void configADC(void);
void configDMA(void);
void configTIM0(void);
void confTim1(void);
void configUART1(void);
void sistema_start(void);
void sistema_stop(void);
void PWM_SetDuty(uint8_t duty);
void PWM_AplicarDuty(uint8_t duty);
void procesarBloqueDMA(void);

uint8_t mv_a_porcentaje(uint16_t mv);
uint16_t porcentaje_a_mv(uint8_t pct);

void UART_SendString(const char *str);
void UART_SendUInt(uint32_t num);
void UART_SendInt(int32_t num);
void UART_ProcesarComando(void);
void UART_EnviarEstado(void);
uint8_t convertirTextoAPorcentaje(volatile char *str);

void TIMER1_IRQHandler(void);
void DMA_IRQHandler(void);
void EINT0_IRQHandler(void);
void UART1_IRQHandler(void);


// ===== MAIN =====
int main(void) {
    SystemCoreClockUpdate();						// actualiza frecuencia real a la que esta el procesador (100MHz)
    SysTick_Config(SystemCoreClock / 1000);			// config Systick para 1mS (100M/1000=100k -> 1ms)

    confPin();
    configDAC();
    configADC();
    configDMA();
    configTIM0();
    confTim1();
    configUART1();

    // inicializar el valor deseado
    mv_deseado = porcentaje_a_mv(porcentaje_deseado); 								// se inicia con un porc deseado = 50%

    lux_deseado = (uint16_t)(((uint32_t)mv_deseado * LUX_MAXIMO) / MV_MAXIMO);		// regla de 3 simples para calcular el lux

    UART_SendString("=== Sistema de Control de Iluminacion ===\r\n");
    UART_SendString("Presione EINT0 para arrancar el sistema\r\n");
    UART_SendString("Ingrese porcentaje deseado (0-100) via UART:\r\n");

    while (1) {														// bucle que se encarga del envio del UART segun alguna flag
        if (comando_listo) {
            UART_ProcesarComando();
            continue;                                           //me salteo el resto 
        }   

        if (aviso_arranque == 1) {
            aviso_arranque = 0;
            UART_SendString("\r\n[SISTEMA ARRANCADO]\r\n");
        } else if (aviso_arranque == 2) {
            aviso_arranque = 0;
            UART_SendString("\r\n[SISTEMA DETENIDO]\r\n");
        }

        if (enviar_estado) {
            enviar_estado = 0;
            UART_EnviarEstado();
        }
    }
    return 0;
}

// Convierte los mV a porcentaje de luz                     % = %inicial + ((mV - mv_inicial) * dif%) / dif_mV
uint8_t mv_a_porcentaje(uint16_t mv) {
    if (mv == 0) return 0;

    // tramo 1: cualquier lectura real hasta 425 mV cuenta como 1%
    if (mv <= 425) return 1;

    // tramo 2: 1% a 20% (425 a 1087 mV) - dif: 662 mV, 19%
    if (mv <= 1087) return (uint8_t)(1 + (((mv - 425) * 19) / 662));

    // tramo 3: 20% a 30% (1087 a 1436 mV) - dif: 349 mV, 10%
    if (mv <= 1436) return (uint8_t)(20 + (((mv - 1087) * 10) / 349));

    // tramo 4: 30% a 50% (1436 a 2063 mV) - dif: 627 mV, 20%
    if (mv <= 2063) return (uint8_t)(30 + (((mv - 1436) * 20) / 627));

    // tramo 5: 50% a 75% (2063 a 2718 mV) - dif: 655 mV, 25%
    if (mv <= 2718) return (uint8_t)(50 + (((mv - 2063) * 25) / 655));

    // tramo 6: 75% a 100% (2718 a 3133 mV) - dif: 415 mV, 25%
    if (mv <= 3133) return (uint8_t)(75 + (((mv - 2718) * 25) / 415));

    return 100; 	// saturación absoluta
}

// Convierte porcentaje deseado a mV objetivo                       mv = mv_inicial + ((% - %inicial) * dif_mv) / dif%
uint16_t porcentaje_a_mv(uint8_t pct) {
    if (pct == 0) return 0;
    if (pct <= 1) return (uint16_t)((pct * 425) / 1);
    if (pct <= 20) return (uint16_t)(425 + (((pct - 1) * 662) / 19));
    if (pct <= 30) return (uint16_t)(1087 + (((pct - 20) * 349) / 10));
    if (pct <= 50) return (uint16_t)(1436 + (((pct - 30) * 627) / 20));
    if (pct <= 75) return (uint16_t)(2063 + (((pct - 50) * 655) / 25));
    if (pct <= 100) return (uint16_t)(2718 + (((pct - 75) * 415) / 25));
    return 3133;
}


// ==== temporizador de 2 segundos para UART1 ====

void SysTick_Handler(void) {
    contador_2_segundos++;
    if (contador_2_segundos >= 2000) {
        contador_2_segundos = 0;
        enviar_estado = 1;
    }
}



// ===== Config Pines y EINT0 =====

void confPin(void) {
    PINSEL_CFG_T confpin = {
        .port      = PORT_0,
        .pin       = PIN_0,
        .func      = 0,
        .mode      = 0,
        .openDrain = DISABLE
    };
    PINSEL_ConfigPin(&confpin);					//conf pin 0.0
    GPIO_SetDir(PORT_0, 1 << 0, GPIO_OUTPUT);
    GPIO_ClearPins(PORT_0, 1 << 0);

    EXTI_Init();
    EXTI_PinConfig(EXTI_EINT0, EXTI_PULLUP);	//conf EINT0
    EXTI_CFG_T conf_exti = {
        .line     = EXTI_EINT0,
        .mode     = EXTI_EDGE_SENSITIVE,
        .polarity = EXTI_FALLING_EDGE
    };
    EXTI_ConfigEnable(&conf_exti);
    NVIC_SetPriority(EINT0_IRQn, 2);
    NVIC_EnableIRQ(EINT0_IRQn);
}


// ===== Config DAC ======

void configDAC(void) {
    DAC_Init();
    DAC_UpdateValue(0);
    DAC_CONVERTER_CFG_T conf_dac = {
        .doubleBuffer = DISABLE,
        .dmaCounter   = DISABLE,
        .dmaRequest   = DISABLE
    };
    DAC_ConfigDAConverterControl(&conf_dac);
}


// ===== Config ADC =====

void configADC(void) {
    ADC_Init(200000);								//200 kHz
    ADC_PinConfig(ADC_CHANNEL_0);					// AD0.0
    ADC_BurstDisable();
    ADC_ChannelEnable(ADC_CHANNEL_0);
    ADC_EdgeStartConfig(ADC_START_ON_FALLING);
    ADC_StartCmd(ADC_START_ON_MAT01);
}


// ===== Config DMA =====

void configDMA(void) {
    static GPDMA_LLI_T lli_adc= {                       // config lli
        .srcAddr = (uint32_t)&LPC_ADC->ADDR0,
        .dstAddr = BUFFER_ADC_ADDR,
        .nextLLI = (uint32_t)&lli_adc,
        .control = (BUFFER_TAM & 0xFFF) | (2 << 18) | (2 << 21) | (0 << 26) | (1 << 27) | (1 << 31)
    };

    GPDMA_Channel_CFG_T dma_adc = {
        .channelNum = GPDMA_CH_0,
        .transferSize = BUFFER_TAM,
        .type = GPDMA_P2M,
        .srcMemAddr = (uint32_t)&LPC_ADC->ADDR0,
        .dstMemAddr = BUFFER_ADC_ADDR,
        .srcConn = GPDMA_ADC,
        .dstConn  = 0,
        .src = { 
            .width = GPDMA_WORD, 
            .burst = GPDMA_BSIZE_1, 
            .increment = DISABLE 
        },
        .dst = { 
            .width = GPDMA_WORD, 
            .burst = GPDMA_BSIZE_1, 
            .increment = ENABLE 
        },
        .intTC = ENABLE,                
        .intErr = DISABLE,
        .linkedList = (uint32_t)&lli_adc
    };

    GPDMA_Init();
    GPDMA_SetupChannel(&dma_adc);
    NVIC_SetPriority(DMA_IRQn, 1);
    NVIC_EnableIRQ(DMA_IRQn);
}


// ===== TIM0 =====

void configTIM0(void) {						//encargado de disparar al ADC
    TIM_TIMERCFG_T tim0 = {
        .prescaleOpt   = TIM_US,
        .prescaleValue = 1					//1uS
    };
    TIM_MATCHCFG_T mat01 = {
        .channel = TIM_MATCH_1,
        .intEn = DISABLE,
        .stopEn = DISABLE,
        .resetEn = ENABLE,                      
        .extOpt = TIM_TOGGLE,
        .matchValue = 500 - 1				//flanco cada 500uS, haciendo que haya un flanco de bajada cada 1mS
    };
    TIM_InitTimer(LPC_TIM0, &tim0);
    TIM_ConfigMatch(LPC_TIM0, &mat01);
    TIM_PinConfig(TIM_MAT0_1_P1_29);
}


// ===== TIM1 - PWM - Handler =====

void confTim1(void) {						//encargado del PWM
    TIM_TIMERCFG_T conf = {
        .prescaleOpt = TIM_US,
        .prescaleValue = 1
    };

    TIM_MATCHCFG_T match0 = {				//match Fijo
        .channel = TIM_MATCH_0,
        .intEn = ENABLE,
        .stopEn = DISABLE,
        .resetEn = ENABLE,
        .extOpt = TIM_NOTHING,
        .matchValue = TIM1_PERIODO			//periodo fijo de 5mS
    };

    TIM_MATCHCFG_T match1 = {				//match Variable
        .channel = TIM_MATCH_1,
        .intEn = ENABLE,
        .stopEn = DISABLE,
        .resetEn = DISABLE,
        .extOpt = TIM_NOTHING,
        .matchValue = 1						//periodo variable
    };

    TIM_InitTimer(LPC_TIM1, &conf);
    TIM_ConfigMatch(LPC_TIM1, &match0);
    TIM_ConfigMatch(LPC_TIM1, &match1);
    NVIC_SetPriority(TIMER1_IRQn, 3);
    NVIC_EnableIRQ(TIMER1_IRQn);
}

void TIMER1_IRQHandler(void) {
    if (TIM_GetIntStatus(LPC_TIM1, TIM_MR0_INT) == SET) {
        if (duty_pwm != duty_pwm_pendiente) {					//verifica si hay nuevo valor del duty calculado por el DMA
            duty_pwm = duty_pwm_pendiente;                      // se va a cambiar el mr1 al inicio del pwm (en mr0)
            PWM_AplicarDuty(duty_pwm);                          // actualiza el mr1 
        }

        if (sistema_activo && duty_pwm > 0)
            GPIO_SetPins(PORT_0, 1 << 0);						//pin ON si hay duty activo
        else
            GPIO_ClearPins(PORT_0, 1 << 0);						//pin OFF si no hay duty activo
        TIM_ClearIntPending(LPC_TIM1, TIM_MR0_INT);
    }

    if (TIM_GetIntStatus(LPC_TIM1, TIM_MR1_INT) == SET) {
        if (duty_pwm < 100)
            GPIO_ClearPins(PORT_0, 1 << 0);						//si el duty es menor al 100% baja el pin, sino se queda ON
        TIM_ClearIntPending(LPC_TIM1, TIM_MR1_INT);
    }
}


// ===== Handler DMA ===== 

void DMA_IRQHandler(void) {
    if (LPC_GPDMA->DMACIntTCStat & (1 << 0)) {					//flag de que el canal 0 termino de procesar
        LPC_GPDMA->DMACIntTCClear = (1 << 0);					//bajar Flag

        if (sistema_activo) {									//si esta ON el sistema se procesa el bloque
            procesarBloqueDMA();
        }
    }
}

void procesarBloqueDMA(void) {                                  //calcula nuevo valor de adc, actualiza los demas valores en base a eso, actualiza el dac
    volatile uint32_t *buf = (volatile uint32_t *)BUFFER_ADC_ADDR;		//puntero para recorrer y hacer el promedio
    uint32_t suma = 0;                  
    uint32_t i;
    uint32_t dac_valor;
    uint8_t nuevo_duty;                                                   // nuevo duty medido y calculado

    for (i=0; i< BUFFER_TAM; i++) {
        suma += (buf[i] >> 4) & 0xFFF;									// suma los datos del ADC para luego sacar el promedio
    }
    adc_promedio = (uint16_t)(suma / BUFFER_TAM);						//suma / 64

    // calculo directo de los mV leidos por el sensor
    mv_ambiente = (uint16_t)(((uint32_t)adc_promedio * 3300) / 4095);       // hace los calculos para los nuevos valores con el nuevo valor del adc

    // conversion lineal de mV: a Lux Físicos (Para el UART)
    lux_ambiente = (uint16_t)(((uint32_t)mv_ambiente * LUX_MAXIMO) / MV_MAXIMO);

    // calculo del porcentaje de luz ambiente
    porcentaje_ambiente = mv_a_porcentaje(mv_ambiente);

    // calculo del error para el control del PWM y para el UART
    error_luz = (int16_t)porcentaje_deseado - (int16_t)porcentaje_ambiente;

    if (error_luz <= 0) {											//si el error<=0, signfica mucha luz por lo tanto apago los leds
        nuevo_duty = 0;
    } else if ((porcentaje_deseado > ERROR_ZONA_MUERTA) &&			//Si el error<2% de error de la zoma muerta, no enciende los leds, y para no anular valores deseados bajos 
               (error_luz <= ERROR_ZONA_MUERTA)) {
        nuevo_duty = 0;
    } else if (error_luz >= 100) {									//si el error>= 100 significa poca luz por lo tanto enciendo
        nuevo_duty = 100;											//los leds al maximo
    } else {														// Duty = error calculado
        nuevo_duty = (uint8_t)error_luz;
    }

    if (nuevo_duty != duty_pwm_pendiente) {							// solo actualiza el duty si cambio pendiente, para evitar sobreescribir el match
        PWM_SetDuty(nuevo_duty);
    }

    // DAC: refleja directamente los lux medidos
    dac_valor = ((uint32_t)lux_ambiente * 1023) / 3300;
    if (dac_valor > 1023) dac_valor = 1023;
    DAC_UpdateValue((uint16_t)dac_valor);
}



// ===== Handler EINT0 =====

void EINT0_IRQHandler(void) {				//encargado del boton que activa o desactiva el sistema
    if (sistema_activo == 0)
        sistema_start();
    else
        sistema_stop();
    EXTI_ClearFlag(EXTI_EINT0);
}


// ===== Funciones encargadas de detener y activar el sistema ===== 

void sistema_start(void) {					//activa el sistema
    sistema_activo = 1;
    aviso_arranque = 1;

    adc_promedio = 0;
    porcentaje_ambiente = 0;
    error_luz = 0;
    duty_pwm = 0;
    duty_pwm_pendiente = 0;
    lux_ambiente = 0;
    mv_ambiente = 0;

    DAC_UpdateValue(0);
    PWM_AplicarDuty(0);
    GPIO_ClearPins(PORT_0, 1 << 0);
    configDMA();
    GPDMA_ChannelStart(GPDMA_CH_0);
    TIM_Enable(LPC_TIM1);
    TIM_Enable(LPC_TIM0);
}

void sistema_stop(void) {						//desactiva el sistema
    sistema_activo = 0;
    aviso_arranque = 2;

    TIM_Disable(LPC_TIM0);
    TIM_Disable(LPC_TIM1);
    GPDMA_ChannelStop(GPDMA_CH_0);

    GPIO_ClearPins(PORT_0, 1 << 0);
    DAC_UpdateValue(0);
    duty_pwm = 0;
    duty_pwm_pendiente = 0;
    PWM_AplicarDuty(0);
    lux_ambiente = 0;
    mv_ambiente = 0;
}


// ===== Funciones encargadas de cambiar el Duty =====

void PWM_SetDuty(uint8_t duty) {
    if (duty > 100) duty = 100;
    duty_pwm_pendiente = duty;					//solo escribe la variable sin tocar el timer
}

void PWM_AplicarDuty(uint8_t duty) {			//calcula tick y actualiza el mr1
    uint32_t ticks;
    if (duty == 0) {											//Min
        TIM_UpdateMatchValue(LPC_TIM1, TIM_MATCH_1, 1);
    } else if (duty >= 100) {									//Max
        TIM_UpdateMatchValue(LPC_TIM1, TIM_MATCH_1, TIM1_PERIODO + 1);          // queda fuera del periodo y el pin no baja
    } else {
        ticks = ((uint32_t)(TIM1_PERIODO + 1) * duty) / 100;
        if (ticks == 0)            ticks = 1;					//para no tener ticks = 0
        if (ticks > TIM1_PERIODO)  ticks = TIM1_PERIODO;		//para no tener ticks mayores al periodo fijo
        TIM_UpdateMatchValue(LPC_TIM1, TIM_MATCH_1, ticks);
    }
}


// ===== Config UART y Handler =====

void configUART1(void) {
    UART_CFG_T uartCfg = {
        .baudRate = 115200,								//velocidad de transmision, mas rapidos el envio de caracteres y evitar bloqueos
        .dataBits = UART_DBITS_8,
        .stopBits = UART_STOPBIT_1,
        .parity = UART_PARITY_NONE					//envio de 8 bits por paquete, con 1 bit de parada para setear el fin de cada caracter
    };													//sin bit de paridad para no comprobar si hay errores
    UART_FIFO_CFG_T fifoCfg = {
        .level = UART_FIFO_TRGLEV0,				    //trigger level 0: interrumpe cuando llega 1 caracter
        .resetRxBuf = ENABLE,                       // enable limpioa fifo de recepcion
        .resetTxBuf = ENABLE,                       //enable limpia fifo de transmision
        .dmaMode = DISABLE                          // no usamos dma
    };

    UART_PinConfig(UART_TX1_P0_15);
    UART_PinConfig(UART_RX1_P0_16);
    UART_Init(UART_PC, &uartCfg);                   // powerup, calcula divisores, configura y limpia estados viejos    
    UART_FIFOConfig(UART_PC, &fifoCfg);
    UART_TxEnable(UART_PC);
    UART_IntConfig(UART_PC, UART_INT_RBR, ENABLE);      //habilito int cuando hay dato recibido en la fifo rx
    UART_IntConfig(UART_PC, UART_INT_RLS, ENABLE);      //habilito int por error o estado de linea 
    NVIC_SetPriority(UART1_IRQn, 0);
    NVIC_EnableIRQ(UART1_IRQn);
}


// ==== Para Recepcion ====

void UART1_IRQHandler(void) {                           // para recepcion
    uint32_t intId = UART_GetIntId(UART_PC);			//Id de la interrupcion del uart, lee el IIR 
    uint32_t tipo_int;
    uint8_t estado;
    uint8_t dato;
    uint8_t leidos = 0;

    if (intId & UART_IIR_INTSTAT_PEND) return;			// = 1 no hay int pendiente, = 0 si hay int pendiente 

    tipo_int = intId & UART_IIR_INTID_MASK;             //se queda con los bits que le dicen la causa de la int     
    estado = UART_GetLineStatus(UART_PC);               //lee el LSR, contiene flags 

    if (tipo_int == UART_IIR_INTID_RLS) {				//si fue int de error de linea se limpia los buffer de rx si no habia comando para procesar
        if (!comando_listo) {							//y se limpia la FIFO en el while
            rx_index = 0;
            rx_buffer[0] = '\0';
        }

        while ((leidos < RX_BUFFER_SIZE) && (UART_Receive(UART_PC, &dato, 1, NONE_BLOCKING) == 1)) {	//se usa el NONE_BLOCKING para no bloquear
            leidos++;																		//mientras haya datos en el FIFO, los limpia leyendolos 
        }
        return;
    }

    if (tipo_int == UART_IIR_INTID_RDA || tipo_int == UART_IIR_INTID_CTI) {		//flag de int de dato recibido o dato atascado sin enviar
        if ((estado & (UART_LINESTAT_OE | UART_LINESTAT_PE | UART_LINESTAT_FE | UART_LINESTAT_BI | UART_LINESTAT_RXFE)) && !comando_listo) { //Diferentes tipos de errores, limpia el buffer de rx, mientras no se haya enviado un dato por teclado
            rx_index = 0;
            rx_buffer[0] = '\0';
        }

        while ((leidos < RX_BUFFER_SIZE) && (UART_Receive(UART_PC, &dato, 1, NONE_BLOCKING) == 1)) {		//limpia la FIFO si hubo error
            leidos++;

            if (comando_listo) {			
                continue;                               //para no agregar nuevos caracteres si ya hay un comanod listo para procesar 
            }

            if (dato == '\r' || dato == '\n') {		//detecta el enter del teclado y activa la flag de comando listo
                if (rx_index > 0) {                 //si tiene al menos un caracter cierra el string
                    rx_buffer[rx_index] = '\0';
                    rx_index = 0;
                    comando_listo = 1;
                }
            } else if (dato == 8 || dato == 127) {	//8 = espacio, 127 = borrar
                if (rx_index > 0) {					// si ya hay un caracter, lo borra y vuelve un espacio anterior, mientras no se haya enviado el enter
                    rx_index--;
                    rx_buffer[rx_index] = '\0';
                }
            } else {
                if (dato < '0' || dato > '9') {		//verifica si es un dato entre el 0 y 9, si es otro caracter lo descarta
                    continue;
                }

                if (rx_index < (RX_BUFFER_SIZE - 1)) {		//se encarga de guardar el dato en el buffer del rx, mientras no se pase del tamaño del buffer
                    rx_buffer[rx_index++] = dato;
                    rx_buffer[rx_index] = '\0';
                } else {
                    rx_index     = 0;
                    rx_buffer[0] = '\0';            //si se llena reinicio
                }
            }
        }
    }
}


// ===== Funciones para la transmision UART =====

void UART_SendString(const char *str) {				//encargado del envio del texto, Blocking = bloqueo hasta que se termine de enviar
    UART_Send(UART_PC, (uint8_t *)str, strlen(str), BLOCKING);      //str texto a enviar, strlen cuenta caracteres a enviar
}

void UART_SendUInt(uint32_t num) {					//encargado de transformar los caracteres en numero entero y enviarlos
    char buf[11];
    int8_t i = 0, j;
    if (num == 0) {
        UART_Send(UART_PC, (uint8_t *)"0", 1, BLOCKING);        //manda 0
        return;
    }
    while (num > 0 && i < 10) {
        buf[i++] = (char)((num % 10) + '0');                // guarda los digitos de a 1 
        num /= 10;
    }
    for (j=i-1; j>=0; j--)								//encargado de enviar en orden correcto, por que el while los obtiene en orden al reves
        UART_Send(UART_PC, (uint8_t *)&buf[j], 1, BLOCKING);
}

void UART_SendInt(int32_t num) {						//encargado de enviar el signo del numero, pq error puede ser negativo
    if (num < 0) {
        UART_Send(UART_PC, (uint8_t *)"-", 1, BLOCKING);
        UART_SendUInt((uint32_t)(-num));
    } else {
        UART_SendUInt((uint32_t)num);
    }
}


// ==== Funciones para recibir por UART ====

void UART_ProcesarComando(void) {                   //procesa comando recibido y responde ok y el valor o error 
    char comando_local[RX_BUFFER_SIZE];         //buffer local
    uint8_t nuevo_valor;
    uint8_t i;

    NVIC_DisableIRQ(UART1_IRQn);				//se deshabilita la int para copiar el buffer por si llega un caracter de por medio y evitar error
    for (i=0; i< RX_BUFFER_SIZE; i++) {
        comando_local[i] = rx_buffer[i];
        if (rx_buffer[i] == '\0') break;        //copia el comando 
    }
    comando_local[RX_BUFFER_SIZE - 1] = '\0';	//x seguridad 
    rx_buffer[0] = '\0';                        //limpio el buffer global
    rx_index = 0;
    comando_listo = 0;
    NVIC_EnableIRQ(UART1_IRQn);                 //habilito de nuevo para recibir 

    enviar_estado = 0;							//limpio flag del systick para q no se mande info mientras proceso 
    contador_2_segundos = 0;					//reseteo contador para q arranque de nuevo una vez cambiados los valores

    nuevo_valor = convertirTextoAPorcentaje(comando_local);		//convierte el codigo ASCII en un entero

    if (nuevo_valor <= 100) {						//se encarga de verificar si el valor ingresado esta entre 0 y 100
        porcentaje_deseado = nuevo_valor;
        mv_deseado = porcentaje_a_mv(porcentaje_deseado); 		// actualiza también el setpoint interno en mV y Lux para el reporte UART
        lux_deseado = (uint16_t)(((uint32_t)mv_deseado * LUX_MAXIMO) / MV_MAXIMO);

        UART_SendString("\r\nOK: ");
        UART_SendUInt(porcentaje_deseado);
        UART_SendString(" %\r\n");
    } else {
        UART_SendString("\r\nERROR: Ingrese un valor entre 0 y 100\r\n");
    }
}

uint8_t convertirTextoAPorcentaje(volatile char *str) {		//se encarga de convertir el texto ingresado a un valor entero, recorriendo el buffer de rx
    uint16_t valor = 0;
    uint8_t i = 0;
    if (str[0] == '\0') return 255;
    while (str[i] != '\0' && i < RX_BUFFER_SIZE) {
        if (str[i] < '0' || str[i] > '9') return 255;
        valor = valor * 10 +(uint16_t)(str[i] - '0');
        if (valor > 100) return 255;
        i++;
    }
    if (i >= RX_BUFFER_SIZE) return 255;
    return (uint8_t)valor;
}


/// ===== Envio de UART =====

void UART_EnviarEstado(void) {
    UART_SendString("\r\n======== ESTADO DEL SISTEMA ========\r\n");

    UART_SendString("Estado: ");
    UART_SendString(sistema_activo ? "ACTIVO\r\n": "DETENIDO\r\n");

    UART_SendString("Valor deseado en porcentaje: ");
    UART_SendUInt(porcentaje_deseado);
    UART_SendString("\r\n");

    UART_SendString("Valor deseado en Lux: ");
    UART_SendUInt(lux_deseado);
    UART_SendString("\r\n");

    UART_SendString("Luz ambiente medida  en porcentaje: ");
    UART_SendUInt(porcentaje_ambiente);
    UART_SendString("\r\n");

    UART_SendString("Luz ambiente medida en Lux: ");
    UART_SendUInt(lux_ambiente);
    UART_SendString("\r\n");

    UART_SendString("Error (des - amb): ");
    UART_SendInt(error_luz);
    UART_SendString("\r\n");

    UART_SendString("Valor de lux en mV: ");
    UART_SendUInt(mv_ambiente);
    UART_SendString("\r\n");
}

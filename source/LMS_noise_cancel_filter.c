/*
 * Author: Santiago Raimondi.
 * Fecha: 15/11/2021.
 *
 * Consigna:
 *	Utilizar un filtro FIR de 30 coeficientes para correlacionar la señal random (actuando como ruido)
 *	con la entrada de referencia del procesador adaptativo. Ingresar a la placa una senoidal y sumarle
 *	el ruido random. Demostrar que el procesador adaptativo converge al filtro óptimo que cancela el
 *	ruido de la señal+ruido. Mostrar la señal con un osciloscopio.
 *
 *	Notas:
 *		Con la variable de preprocesamiento TEST se activa la generacion de una sinusoide por software
 *		y no se utiliza la entrada del ADC como fuente de señal.
 *
 *
 *	NO ANDA POR EL MOMENTO EL FILTRADO
 */

/**
 * @file    LMS_noise_cancel_filter.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"

/* TODO: insert other include files here. */
#include "arm_math.h"

/* TODO: insert other definitions and declarations here. */
#define NUMTAPS   	(uint32_t)	30		/* Cantidad de coeficientes de los filtros FIR y LMS */
#define BLOCKSIZE 	(uint32_t)	1		/* Cantidad de muestras a procesar por llamado de los filtros */
#define POSTSHIFT 	(uint32_t)	0
//#define MU			(q15_t)		1000	/* Paso de aproximacion del filtro LMS */
#define CHANNEL_GROUP (uint32_t) 0		/* Para el ADC */
#define DAC_BUFFER_INDEX 		0U		/* Como no se utiliza el buffer, siempre es 0 este valor */
#define DC_OFFSET 				32767U	/* Offset de continua (1,65 [V]) */

/* Variable para hacer la onda senoidal por software en vez de adquirirla por ADC.
 * Si esta comentada, se espera que la señal provenga del ADC.
 * Si esta descomentada se genera la señal por software. */
#define TEST

#ifdef	TEST
	#define SIN_FRECUENCY_HZ 	300		/* Frecuencia del seno que se genera por software */
	#define SAMPLE_PERIOD_US	45U		/* Valor que coincide con el de la definicion de PIT_0_TICKS para que la
										   señal por software tenga la misma tasa de muestreo que la adquirida por ADC */
	uint32_t timerFilter_us = 0; 		/* Para simular el paso del tiempo entre dos muestras */
#endif

volatile q15_t input_value_fixed = 0;	/* Ultima muestra del ADC */
volatile bool adc_finished = false;		/* Flag que indica si hay una nueva muestra del ADC */

/* Valor con el que se shiftea la señal de ruido.
 * La amplitud del ruido no tiene que ser excesiva respecto a la señal.
 * Con 17 para arriba anda bien (si noise_shift = 17 se tomanlos 15
 * MSB que devuelve rand()).
 *
 * Se declaran globales para poder modificarlas durante debugg sin necesidad
 * de compilar el programa nuevamente.
 */
uint8_t noise_shift = 18;
q15_t MU = 1000;

/* La frecuencia de muestreo es 22 [k/s] */
void PIT_CHANNEL_0_IRQHANDLER(void) {
  uint32_t intStatus;
  /* Reading all interrupt flags of status register */
  intStatus = PIT_GetStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_0);
  PIT_ClearStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_0, intStatus);

  /* Place your code here */
  /* Se dispara la conversion */
  	ADC16_SetChannelConfig(ADC0_PERIPHERAL, CHANNEL_GROUP, &ADC0_channelsConfig[0]);	/* De la descripcion de la funcion vi que el CHANNEL_GROUP=0 para software trigger */

  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}

/* ADC0_IRQn interrupt handler */
void ADC0_IRQHANDLER(void) {
	/* Array of result values*/
	uint32_t result_values[2] = {0};
	/* Get flags for each group */
	for ( int i=0; i<2; i++){
	uint32_t status = ADC16_GetChannelStatusFlags(ADC0_PERIPHERAL, i);
	if ( status == kADC16_ChannelConversionDoneFlag){
		result_values[i] = ADC16_GetChannelConversionValue(ADC0_PERIPHERAL, i);
	}
	}

	/* Place your code here */
	adc_finished = true;
	input_value_fixed = (q15_t) (result_values[0] - DC_OFFSET);	/* Se le resta el offset de continua que trae la señal de entrada */

	/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
	 Store immediate overlapping exception return operation might vector to incorrect interrupt. */
	#if defined __CORTEX_M && (__CORTEX_M == 4U)
		__DSB();
	#endif
}

/*
 * @brief   Application entry point.
 */
int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

	/* Filtro FIR (planta) */
	arm_fir_instance_q15 fir_struct;
	q15_t fir_coeficients[NUMTAPS] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,32767,32767,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	q15_t fir_state[NUMTAPS + BLOCKSIZE - 1];
	arm_fir_init_q15(&fir_struct, NUMTAPS, fir_coeficients, fir_state, BLOCKSIZE);

	/* Filtro LMS */
	arm_lms_norm_instance_q15 lms_struct;
	q15_t lms_coeficients[NUMTAPS] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	q15_t lms_state[NUMTAPS + BLOCKSIZE - 1];
	arm_lms_norm_init_q15(&lms_struct, NUMTAPS, lms_coeficients, lms_state, MU, BLOCKSIZE, POSTSHIFT);

	/* Buffers auxiliares para computar algoritmo LMS */
	q15_t src[BLOCKSIZE];	/* Entrada del FIR (ruido) */
	q15_t fir_output[BLOCKSIZE]; /* Señal con la que se entrena al algoritmo LMS (ruido) */
	q15_t ref[BLOCKSIZE];	/* Señal de referencia (señal+ruido)*/
	q15_t out[BLOCKSIZE];	/* Salida del procesador adaptativo (filtro LMS) */
	q15_t err[BLOCKSIZE];	/* Error (ref-out) que se realimenta al procesador adaptivo */

	/* Variables auxiliares */
//	static float32_t ref_value, err_value;


    while(1) {

    	/* Primero se toman las 100 muestras */
    	if(adc_finished)
    	{
    		adc_finished = false;

			/* Se construye la señal con ruido. Se shiftea el valor devuelto
			 * para que sea una señal pequeña y no sature el calculo del filtro.
			 * Recordar que la amplitud de señal de entrada y mu tienen una
			 * relacion de compromiso para la velocidad de convergencia del
			 * algoritmo. Si mu es muy grande o la señal de entrada es muy
			 * grande, el algoritmo puede diverger.
			 */

			#ifdef TEST
				float32_t sin_value = arm_sin_f32(2*PI*SIN_FRECUENCY_HZ*timerFilter_us/1000000);
				timerFilter_us += SAMPLE_PERIOD_US;
				/* Se escala la señal a valores en el rango de 2^14 porque si uso 2^15
				 * la señal hace overflow cuando le sumo el ruido a los valores mas positivos del seno.*/
				input_value_fixed = (q15_t)(sin_value * 16384); // 16384 = 2^14
			#endif

			ref[0] = (q15_t)((rand()>>noise_shift) + input_value_fixed);
			src[0] = (q15_t)((rand()>>noise_shift) );

			// DAC_SetBufferValue(DAC0, DAC_BUFFER_INDEX, (ref[0]>>4)); /* Señal de referencia del filtro */
	    	DAC_SetBufferValue(DAC0, DAC_BUFFER_INDEX, ((err[0]>>4) + DC_OFFSET)); /* Señal filtrada */

			arm_fir_q15(&fir_struct, src, fir_output, BLOCKSIZE);
			arm_lms_norm_q15(&lms_struct, fir_output, ref, out, err, BLOCKSIZE);

			/* Para usar osciloscopio serial */
			// arm_q15_to_float(ref, &ref_value, BLOCKSIZE);
//			 arm_q15_to_float(err, &err_value, BLOCKSIZE);
//			PRINTF("%d, %d\r", ref[0], out[0]);
    	}
    }

    return 0 ;
}

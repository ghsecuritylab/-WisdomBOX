

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "gpio.h"
#include "spi.h"
#include "stm32f4xx_hal.h"
#include "DAC.h"
/*--------------------------------------------------*/
/*--------------------Local Variables---------------*/
/*--------------------------------------------------*/



// pointers to storage for the values to be written to MCP4822

/*--------------------------------------------------*/
/*---------------Public Functions-------------------*/
/*--------------------------------------------------*/

inline void spi1_dac_finalize(void) {
	// check (and if necessary, wait for) completion of SPI transfer.
//	while(SPI2->SR & SPI_SR_BSY) {
//		 // wait for BSY to clear
//		;
//	}
//	HAL_GPIO_WritePin(spi2_cs_GPIO_Port, spi2_cs_Pin, GPIO_PIN_SET);	// SPI SS -> H
}

inline void spi1_dac_write_cha(uint16_t word) {
	

//	 word &= 0x0fff ;	// strip any unwanted bits
	word |= 0x1000;
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET); // SS -> L
	HAL_SPI_Transmit(&hspi1,(uint8_t *)&word, 1, 100);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
}

inline void spi1_dac_write_chb(uint16_t word) {

//	 word &= 0x0fff ;	// strip any unwanted bits
	word |= 0x9000;
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);   // SS -> L
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&word, 1, 100);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	
}

//========================================================
//********************* IIR Filter *********************//
//========================================================
// second order IIR -- "Direct Form I Transposed"
//  a(0)*y(n) = b(0)*x(n) + b(1)*x(n-1) +  b(2)*x(n-2)
//                   - a(1)*y(n-1) -  a(2)*y(n-2)
// assumes a(0) = IIRSCALEFACTOR = 32

// Compute filter function coefficients
// http://en.wikipedia.org/wiki/Digital_biquad_filter
// https://www.hackster.io/bruceland/dsp-on-8-bit-microcontroller
// http://www.musicdsp.org/files/Audio-EQ-Cookbook.txt

void
setIIRFilterLPF(filter_t * const filter) // Low Pass Filter Setting
{
	if (!(filter->sample_rate))
		filter->sample_rate = SAMPLE_RATE;

	if (!(filter->cutoff))
		filter->cutoff = UINT16_MAX >> 1;  // 1/4 of sample rate = filter->sample_rate>>2

	if(!(filter->peak))
		filter->peak =  (uint16_t)(M_SQRT1_2 * UINT16_MAX / Q_MAXIMUM);  // 1/sqrt(2) effectively

	double frequency = ((double)filter->cutoff * (filter->sample_rate >> 1)) / UINT16_MAX;
	double q = (double)filter->peak * Q_MAXIMUM / UINT16_MAX;
	double w0 = (2.0 * M_PI * frequency) / filter->sample_rate;
	double sinW0 = sin(w0);
	double cosW0 = cos(w0);
	double alpha = sinW0 / (q * 2.0f);
	double scale = IIRSCALEFACTOR / (1 + alpha);  // a0 = 1 + alpha

	filter->b0	= \
	filter->b2	= float2int(((1.0 - cosW0) / 2.0) * scale);
	filter->b1	= float2int((1.0 - cosW0) * scale);

	filter->a1	= float2int((-2.0 * cosW0) * scale);
	filter->a2	= float2int((1.0 - alpha) * scale);
}

void
setIIRFilterHPF(filter_t * const filter) // High Pass Filter Setting
{
	if (!(filter->sample_rate))
		filter->sample_rate = SAMPLE_RATE;

	if (!(filter->cutoff))
		filter->cutoff = UINT16_MAX >> 1;  // 1/4 of sample rate = filter->sample_rate>>2

	if(!(filter->peak))
		filter->peak =  (uint16_t)(M_SQRT1_2 * UINT16_MAX / Q_MAXIMUM);  // 1/sqrt(2) effectively

	double frequency = ((double)filter->cutoff * (filter->sample_rate >> 1)) / UINT16_MAX;
	double q = (double)filter->peak * Q_MAXIMUM / UINT16_MAX;
	double w0 = (2.0 * M_PI * frequency) / filter->sample_rate;
	double sinW0 = sin(w0);
	double cosW0 = cos(w0);
	double alpha = sinW0 / (q * 2.0f);
	double scale = IIRSCALEFACTOR / (1 + alpha);  // a0 = 1 + alpha

	filter->b0	= float2int(((1.0 + cosW0) / 2.0) * scale);
	filter->b1	= float2int(-(1.0 + cosW0) * scale);
	filter->b2	= float2int(((1.0 + cosW0) / 2.0) * scale);

	filter->a1	= float2int((-2.0 * cosW0) * scale);
	filter->a2	= float2int((1.0 - alpha) * scale);
}

void
setIIRFilterBPF(filter_t * const filter) // Band Pass Filter Setting
{
	if (!(filter->sample_rate))
		filter->sample_rate = SAMPLE_RATE;

	if (!(filter->cutoff))
		filter->cutoff = UINT16_MAX >> 1;  // 1/4 of sample rate = filter->sample_rate>>2

	if(!(filter->peak))
		filter->peak =  (uint16_t)(M_SQRT1_2 * UINT16_MAX / Q_MAXIMUM);  // 1/sqrt(2) effectively

	double frequency = ((double)filter->cutoff * (filter->sample_rate >> 1)) / UINT16_MAX;
	double q = (double)filter->peak * Q_MAXIMUM / UINT16_MAX;
	double w0 = (2.0 * M_PI * frequency) / filter->sample_rate;
	double sinW0 = sin(w0);
	double cosW0 = cos(w0);
	double alpha = sinW0 / (q * 2.0f);
	double scale = IIRSCALEFACTOR / (1 + alpha);  // a0 = 1 + alpha

	filter->b0	= float2int(alpha * scale);
	filter->b1	= 0;
	filter->b2	= float2int(-alpha * scale);

	filter->a1	= float2int((-2.0 * cosW0) * scale);
	filter->a2	= float2int((1.0 - alpha) * scale);
}

// Coefficients in 8.8 format
// interim values in 24.8 format
// returns y(n) in place of x(n)
void
IIRFilter(filter_t * const filter, int16_t * const xn)
{
	int32_t yn; 			// current output
	int32_t  accum; 		// temporary accumulator

	// sum the 5 terms of the biquad IIR filter
	// and update the state variables
	// as soon as possible
    MultiS16X16to32(yn, filter->xn_2, filter->b2);
	filter->xn_2 = filter->xn_1;

	MultiS16X16to32(accum, filter->xn_1, filter->b1);
	yn += accum;
	filter->xn_1 = *xn;

	MultiS16X16to32(accum, *xn, filter->b0);
	yn += accum;

	MultiS16X16to32(accum, filter->yn_2, filter->a2);
	yn -= accum;
	filter->yn_2 = filter->yn_1;

	MultiS16X16to32(accum, filter->yn_1, filter->a1);
	yn -= accum;

	filter->yn_1 = yn >> (IIRSCALEFACTORSHIFT + 8);  // divide by a(0) = 32 & shift to 16.0 bit outcome from 24.8 interim steps

	*xn = filter->yn_1;  // being 16 bit yn, so that's what we return.
}



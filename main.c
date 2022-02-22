//by wuwbobo2021 <https://github.com/wuwbobo2021>, <wuwbobo@outlook.com>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

//Add preprocessor symbols: USE_STDPERIPH_DRIVER, DEBUG
#include <stm32f30x.h>

#define VRefInt (1510.0 / 4095.0 * 3.3) //actual voltage of VRefInt (for the unique device)
float ad_vrefint = 1.2 * 4095.0 / 3.3; //it will be updated

#define GPIO_ADC GPIOA
#define RCC_AHBPeriph_GPIO_ADC RCC_AHBPeriph_GPIOA
#define GPIO_Pin_ADC GPIO_Pin_1
#define ADC_SampleTime_Default ADC_SampleTime_7Cycles5
#define ADC_Diff_Tol 96 //for distinguishing of connected and floating states
#define ADC_Channel_Bat ADC_Channel_2 //PA1

#define VBatMin 0.8
#define VDecMin 0.05
#define AD_BatMin (4096.0 * VBatMin / 3.3)
#define AD_DecMin (4096.0 * VDecMin / 3.3)
#define Dec_Cnt_Min 4

#define ADC_Output_Cnt 128
#define ADC_Av_Cnt 32
#define ADC_Buffer_Size (ADC_Output_Cnt * ADC_Av_Cnt)

typedef enum {
	Measure_Disconnected = 0,
	Measure_Connected,
	Measure_Capturing,
	Measure_Later
} Measure_State;

volatile Measure_State st = Measure_Disconnected;

volatile uint32_t TickCount = 0, TimingDelay = 0; //SysTick

volatile uint16_t buf[2 * ADC_Buffer_Size];
volatile float av[ADC_Output_Cnt];

// Raw data will be broken by this function.
// Return 0 if the difference between the min and max values
// of the remaining data is still larger than diff_max.
static float get_average(uint16_t* raw_data, uint16_t cnt, float diff_max)
{
	if (cnt == 0) return 0;
	uint16_t bound = cnt / 4;
	
	uint32_t sum = 0;
	for (uint16_t i = 0; i < cnt; i++)
		sum += raw_data[i];
	
	uint16_t cnt_rem = cnt;
	uint16_t umin = 0, umax = 0;
	uint16_t tmin, tmax;
	for (uint16_t i = 0; i < bound + 1; i++) {
		umin = UINT16_MAX; umax = 0;
		
		for (uint16_t j = 0; j < cnt; j++) {
			if (raw_data[j] == UINT16_MAX) continue;
			if (raw_data[j] < umin) {
				tmin = j;
				umin = raw_data[j];
			}
			if (raw_data[j] > umax) {
				tmax = j;
				umax = raw_data[j];
			}
		}
		
		if (i == bound) break; //at the time tmin and tmax are of the remaining data
		sum -= raw_data[tmin]; sum -= raw_data[tmax];
		cnt_rem -= 2;
		
		raw_data[tmin] = raw_data[tmax] = UINT16_MAX;
	}
	
	if (umax - umin > diff_max) 
		return 0;
	return (float)sum / cnt_rem;
}

float get_diff_max(float* data, uint16_t cnt)
{
	if (cnt == 0) return 0;
	
	float max = -UINT16_MAX; float min = UINT16_MAX;
	for (uint16_t i = 0; i < cnt; i++) {
		if (data[i] > max) max = data[i];
		if (data[i] < min) min = data[i];
	}
	
	return (max - min) / 2;
}

float get_diff_average(float* data, uint16_t cnt)
{
	if (cnt == 0) return 0;
	
	float av = 0;
	for (uint16_t i = 0; i < cnt; i++)
		av += data[i];
	av /= cnt;
	
	float sum_diff = 0;
	for (uint16_t i = 0; i < cnt; i++)
		sum_diff += fabs(data[i] - av);
	
	return sum_diff / cnt;
}

void SysTick_Init()
{
	// Setup SysTick Timer for 1 msec interrupts.
	// Reload Value = SysTick Counter Clock (Hz) x  Desired Time base (s)
	// Reload Value should not exceed 0xFFFFFF

	SystemCoreClockUpdate(); //read system clock frequency
	if (SysTick_Config(SystemCoreClock / 1000))
		while (1); //Capture error 

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void Delay(volatile uint32_t nTime)
{ 
  TimingDelay = nTime;
  while (TimingDelay > 0);
}

void SysTick_Handler(void)
{
	TickCount++;
	if (TimingDelay > 0) TimingDelay--;
}

void adc_gpio_init(void)
{
	GPIO_DeInit(GPIO_ADC);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIO_ADC, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_ADC;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIO_ADC, &GPIO_InitStruct);
}

void adc_calibrate(ADC_TypeDef* ADCx)
{
	ADC_VoltageRegulatorCmd(ADCx, ENABLE);
	Delay(10);
	ADC_SelectCalibrationMode(ADCx, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADCx);
	while (ADC_GetCalibrationStatus(ADCx) != RESET);
}

static inline void adc_start(void)
{
	if ((ADC1->CR & ADC_CR_ADSTART) == RESET) {
		ADC_StartConversion(ADC1); //ADC2 starts simultaneously
		
		while ((ADC1->ISR & ADC_FLAG_EOC) == RESET || (ADC2->ISR & ADC_FLAG_EOC) == RESET);
		ADC1->DR; ADC2->DR; //discard first data
	}
}

static inline void adc_prepare_for_dma(void)
{
	ADC1->ISR = ADC_FLAG_OVR; ADC2->ISR = ADC_FLAG_OVR; //actually clears the flag (rc_w1)
}

static inline void adc_stop(void)
{
	if ((ADC1->CR & ADC_CR_ADSTART) != RESET) return;
	ADC_StopConversion(ADC1); while (ADC1->CR & ADC_CR_ADSTP); //ADC2 stops simultaneously
}

void adc_dma_capture(void) //time cost is about 7 ms under 12 MHz ADC Frequency, 7Cycles5 sample time
{
	adc_start();
	DMA_SetCurrDataCounter(DMA1_Channel1, ADC_Buffer_Size);
	DMA_Cmd(DMA1_Channel1, ENABLE);
	adc_prepare_for_dma();
	
	while (DMA_GetITStatus(DMA1_IT_TC1) == RESET); //wait for transfer complete event
	
	adc_stop();
	DMA_Cmd(DMA1_Channel1, DISABLE);
	DMA_ClearITPendingBit(DMA1_IT_TC1);
}

void adc_get_vrefint(void)
{
	adc_dma_capture();
	for (uint16_t i = 0; i < ADC_Buffer_Size; i++)
		buf[i] = buf[2*i + 1];
	ad_vrefint = get_average(buf, ADC_Buffer_Size, ADC_Diff_Tol);
}

static inline float adc_read(void) //when DMA is not running
{
	while ((ADC1->ISR & ADC_FLAG_EOC) == RESET || (ADC2->ISR & ADC_FLAG_EOC) == RESET);
	uint32_t rdual = ADC1_2->CDR;
	uint16_t rbat = rdual & 0xFFFF, rref = (rdual >> 16) & 0xFFFF;
	return (float)rbat * ad_vrefint / rref;
}

float adc_read_average(void)
{
	adc_dma_capture();
	
	for (uint16_t i = 0; i < ADC_Buffer_Size; i++)
		buf[i] = (uint16_t)((float)buf[2*i] * ad_vrefint / buf[2*i + 1]);
	
	float sum = 0;
	for (uint16_t i = 0; i < ADC_Output_Cnt; i++) {
		av[i] = get_average(buf + i*ADC_Av_Cnt, ADC_Av_Cnt, ADC_Diff_Tol);
		sum += av[i];
	}
	return sum / ADC_Output_Cnt;
}

static inline float adc_to_voltage(float ad_val)
{
	return VRefInt * ad_val/ad_vrefint;
}

float adc_get_freq(void) //cost 0.5 s
{
	uint32_t cnt = 0;
	TickCount = 0; //SysTick
	while (TickCount < 500) {
		adc_start();
		DMA_SetCurrDataCounter(DMA1_Channel1, ADC_Buffer_Size);
		DMA_Cmd(DMA1_Channel1, ENABLE);
		adc_prepare_for_dma();
		
		while (DMA_GetITStatus(DMA1_IT_TC1) == RESET)
			if (TickCount >= 500) break;
		
		adc_stop();
		cnt += ADC_Buffer_Size - DMA_GetCurrDataCounter(DMA1_Channel1);
		DMA_Cmd(DMA1_Channel1, DISABLE);
		DMA_ClearITPendingBit(DMA1_IT_TC1);
	}
	
	return (float)cnt / 500.0; //kHz
}

void adc_get_accuracy(uint8_t adc_sampletime, float* acc_worst, float* acc_av)
{
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Bat, 1, adc_sampletime);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_18, 1, adc_sampletime);
	adc_start(); Delay(10);
	adc_read_average();
	
	// fill in empty items corresponding with data discarded by get_average(), ADC_Diff_Tol.
	// these zero outputs have no influrence on accuracy, don't pass them to get_diff().
	float fill = 0; uint16_t i = 0;
	while (i < ADC_Av_Cnt) {
		if (fill == 0) {
			if (av[i] != 0) {
				fill = av[i]; i = 0; continue; //get first non-zero item
			}
		} else {
			if (av[i] == 0) av[i] = fill;
		}
		i++;
	}
	
	*acc_worst = 12.0 - log(get_diff_max(av, ADC_Output_Cnt)) / log(2);
	*acc_av = 12.0 - log(get_diff_average(av, ADC_Output_Cnt)) / log(2);
}

void adc_dma_init()
{
	adc_gpio_init();
	
	ADC_DeInit(ADC1); //Reset ADC12
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div6); //12MHz
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12 | RCC_AHBPeriph_DMA1, ENABLE);
	
	adc_calibrate(ADC1); adc_calibrate(ADC2);
	
	ADC_CommonInitTypeDef ADC_CommonInitStruct;
	ADC_CommonStructInit(&ADC_CommonInitStruct);
	ADC_CommonInitStruct.ADC_Mode = ADC_Mode_RegSimul; //ADC Dual Mode, regular simultaneous mode
	ADC_CommonInitStruct.ADC_Clock = ADC_Clock_AsynClkMode; //PLL->Prescaler->ADC, not from AHB HCLK
	ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStruct.ADC_TwoSamplingDelay = 0;
	ADC_CommonInit(ADC1, &ADC_CommonInitStruct); //ADC1_2
	
	ADC_InitTypeDef ADC_InitStruct;
	ADC_StructInit(&ADC_InitStruct);
	ADC_InitStruct.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable; //continuous mode
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_AutoInjMode = ADC_AutoInjec_Disable;
	ADC_InitStruct.ADC_OverrunMode = ADC_OverrunMode_Enable; //don't preserve the old data
	ADC_InitStruct.ADC_NbrOfRegChannel = 1;
	ADC_Init(ADC1, &ADC_InitStruct); ADC_Init(ADC2, &ADC_InitStruct);
	
	ADC_VrefintCmd(ADC2, ENABLE); //VRefInt:Channel18
	
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE); ADC_Cmd(ADC2, ENABLE);
	while (! (ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY) && ADC_GetFlagStatus(ADC2, ADC_FLAG_RDY)));
	
	DMA_DeInit(DMA1_Channel1);
	DMA_InitTypeDef DMA_InitStruct;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) &(ADC1_2->CDR);
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t) buf;
	DMA_InitStruct.DMA_BufferSize = ADC_Buffer_Size;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_Init(DMA1_Channel1, &DMA_InitStruct);
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Bat, 1, ADC_SampleTime_Default);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_18, 1, ADC_SampleTime_Default);
	adc_get_vrefint();
}

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SysTick_Init();
	adc_dma_init();
	
	float freq = adc_get_freq();
	printf("Current ADC Frequency: %d kHz.\n", (uint16_t) freq);
	printf("Capturing will start %.1f us after the actual drop,\n", Dec_Cnt_Min * 1000 / freq);
	printf("Interval (time cost) of each output data will be: %.1f us.\n", ADC_Av_Cnt * 1000.0 / freq);
	
	float ad_vbat = 0, vbat = 0;
	uint16_t ad_curr;
	float vbat_dec = 0;
	uint16_t dec_cnt = 0, dis_cnt = 0;
	
	while (true) {
		switch (st) {
			case Measure_Disconnected:
				ad_vbat = adc_read_average();
				if (ad_vbat >= AD_BatMin) {
					Delay(300);
					ad_vbat = adc_read_average();
					if (ad_vbat < AD_BatMin) break;
					
					st = Measure_Connected;
					vbat = adc_to_voltage(ad_vbat);
					printf("Connected. Battery Voltage: %.3f V.\n", vbat);
					
					float acc_av, acc_worst;
					adc_get_accuracy(ADC_SampleTime_Default, &acc_worst, &acc_av);
					printf("ADC Accuracy: %.1f ~ %.1f Bits.\n\n", acc_worst, acc_av);
					
					adc_start();
				}
				break;
			
			case Measure_Connected:
				ad_curr = adc_read();
				
				if (ad_curr < AD_BatMin) {
					dis_cnt++; dec_cnt = 0;
					if (dis_cnt >= 2 * ADC_Av_Cnt) {
						st = Measure_Disconnected; dis_cnt = 0;
						printf("Disconnected.\n\n");
					}
				} else if (ad_curr < ad_vbat - AD_DecMin) {
					dec_cnt++; dis_cnt = 0;
					if (dec_cnt >= Dec_Cnt_Min) { //start capturing
						st = Measure_Capturing; dec_cnt = 0;
					}
				} else
					dis_cnt = dec_cnt = 0;
				
				break;

			case Measure_Capturing:
				adc_read_average(); st = Measure_Later;
				
				vbat_dec = 0;
				for (uint16_t i = 1; i < ADC_Output_Cnt; i++) {
					if (av[i] > 0) {
						vbat_dec = adc_to_voltage(av[i]); break;
					}
				}
				if (vbat - vbat_dec < VDecMin) { //false trigger
					st = Measure_Connected; adc_start();
					break;
				} else if (vbat_dec < VBatMin) { //disconnect event
					if (av[ADC_Output_Cnt - 1] < VBatMin) {
						st = Measure_Disconnected;
						printf("Disconnected.\n\n");
					} else {
						st = Measure_Connected; adc_start();
					}
					break;
				}
				
				printf("Captured: %.3f V. Voltage Drop: %.3f V\n", vbat_dec, vbat - vbat_dec); Delay(5);
				for (uint16_t i = 0; i < ADC_Output_Cnt; i++) {
					printf("%.4f ", adc_to_voltage(av[i])); Delay(5);
				}
				printf("\n\n");
				break;
			
			case Measure_Later:
				ad_curr = adc_read_average();
				if (ad_vbat - ad_curr < AD_DecMin / 4) {
					st = Measure_Connected; dis_cnt = 0;
					Delay(2000);
					ad_vbat = adc_read_average();
					vbat = adc_to_voltage(ad_vbat);
					printf("Battery Voltage: %.3f V.\n\n", vbat);
					
					adc_start();
				} else if (ad_curr < AD_BatMin) {
					dis_cnt++;
					if (dis_cnt >= 3) {
						st = Measure_Disconnected; dis_cnt = 0;
						printf("Disconnected.\n\n");
					}
				} else dis_cnt = 0;
				break;
		}
	}
}

extern void assert_param(uint8_t expr)
{
#ifdef DEBUG
	if (! expr)	while (1);
#else
#endif
}

extern int fputc(int c, FILE* stream)
{
	ITM_SendChar(c);
	return -1;
}

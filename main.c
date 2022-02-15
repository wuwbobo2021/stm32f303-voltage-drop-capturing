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
#define ADC_Channel_Bat ADC_Channel_2
#define ADC_Channel_ADC1 ADC_Channel_Bat
#define ADC_Channel_ADC2 ADC_Channel_18

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

volatile uint32_t TickCount = 0; //SysTick
volatile uint32_t TimingDelay;

volatile uint16_t buf[ADC_Buffer_Size];
volatile uint16_t buf_ref[ADC_Buffer_Size];
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

void adc_dma_init(ADC_TypeDef* ADCx, uint16_t* dma_buf, uint16_t buf_size)
{
	DMA_Channel_TypeDef* DMA_Channel; uint32_t DMA_IT_TC1;
	if (ADCx == ADC1) {
		DMA_Channel = DMA1_Channel1; DMA_IT_TC1 = DMA1_IT_TC1;
	} else if (ADCx == ADC2) {
		DMA_Channel = DMA2_Channel1; DMA_IT_TC1 = DMA2_IT_TC1;
	} else return;
	
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div6); //12MHz
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12 | RCC_AHBPeriph_DMA1 | RCC_AHBPeriph_DMA2, ENABLE);
	
	ADC_VoltageRegulatorCmd(ADCx, ENABLE);
	Delay(10);
	ADC_SelectCalibrationMode(ADCx, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADCx);
	while (ADC_GetCalibrationStatus(ADCx) != RESET);
	ADC_GetCalibrationValue(ADCx);
	
	ADC_CommonInitTypeDef ADC_CommonInitStruct;
	ADC_CommonStructInit(&ADC_CommonInitStruct);
	ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStruct.ADC_Clock = ADC_Clock_AsynClkMode; //PLL->Prescaler->ADC, not from AHB HCLK
	ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStruct.ADC_TwoSamplingDelay = 0;
	ADC_CommonInit(ADCx, &ADC_CommonInitStruct);
	
	ADC_InitTypeDef ADC_InitStruct;
	ADC_StructInit(&ADC_InitStruct);
	ADC_InitStruct.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable; //continuous mode
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_AutoInjMode = ADC_AutoInjec_Disable;
	ADC_InitStruct.ADC_OverrunMode = ADC_OverrunMode_Enable;
	ADC_InitStruct.ADC_NbrOfRegChannel = 1;
	ADC_Init(ADCx, &ADC_InitStruct);
	
	if (ADCx == ADC2) ADC_VrefintCmd(ADCx, ENABLE); //18 VRefInt
	ADC_DMACmd(ADCx, ENABLE);
	ADC_Cmd(ADCx, ENABLE);
	while (! ADC_GetFlagStatus(ADCx, ADC_FLAG_RDY));
	
	DMA_DeInit(DMA_Channel);
	DMA_InitTypeDef DMA_InitStruct;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) &(ADCx->DR);
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t) dma_buf;
	DMA_InitStruct.DMA_BufferSize = buf_size;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_Init(DMA_Channel, &DMA_InitStruct);
}

static inline float adc_read(ADC_TypeDef* ADCx)
{
	while ((ADCx->ISR & ADC_FLAG_EOC) == RESET);
	return ADCx->DR;
}

static inline void adc_start(ADC_TypeDef* ADCx)
{
	ADC_StartConversion(ADCx);
	ADC_ClearFlag(ADCx, ADC_FLAG_OVR);
}

static inline void adc_stop(ADC_TypeDef* ADCx)
{
	ADC_StopConversion(ADCx); while (ADCx->CR & ADC_CR_ADSTP);
}

//cost at least 6.8 ms under 12 MHz ADC Frequency, 7Cycles5 sample time
float adc_read_average(ADC_TypeDef* ADCx)
{
	DMA_Channel_TypeDef* DMA_Channel; uint32_t DMA_IT_TC1;
	if (ADCx == ADC1) {
		DMA_Channel = DMA1_Channel1; DMA_IT_TC1 = DMA1_IT_TC1;
	} else if (ADCx == ADC2) {
		DMA_Channel = DMA2_Channel1; DMA_IT_TC1 = DMA2_IT_TC1;
	} else return 0;
	
	DMA_SetCurrDataCounter(DMA_Channel, ADC_Buffer_Size); //manual counter reload
	DMA_Cmd(DMA_Channel, ENABLE);
	adc_start(ADCx);
	
	while (DMA_GetITStatus(DMA_IT_TC1) == RESET); //wait for transfer complete event
	
	adc_stop(ADCx);
	DMA_Cmd(DMA_Channel, DISABLE);
	DMA_ClearITPendingBit(DMA_IT_TC1);
	return get_average((ADCx == ADC1)? buf : buf_ref, ADC_Buffer_Size, ADC_Diff_Tol);
}

void adc_overall_init(void)
{
	adc_gpio_init();
	ADC_DeInit(ADC1); //Reset ADC12
	adc_dma_init(ADC1, buf, ADC_Buffer_Size);
	adc_dma_init(ADC2, buf_ref, ADC_Buffer_Size);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_ADC1, 1, ADC_SampleTime_Default);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_ADC2, 1, ADC_SampleTime_Default);
	ad_vrefint = adc_read_average(ADC2);
}

void adc_overall_start(void)
{
	DMA_SetCurrDataCounter(DMA1_Channel1, ADC_Buffer_Size);
	DMA_SetCurrDataCounter(DMA2_Channel1, ADC_Buffer_Size);
	if ((ADC1->CR & ADC_CR_ADSTART) == RESET) ADC_StartConversion(ADC1);
	if ((ADC2->CR & ADC_CR_ADSTART) == RESET) ADC_StartConversion(ADC2);
	Delay(10);
	DMA1_Channel1->CCR |= DMA_CCR_EN; DMA2_Channel1->CCR |= DMA_CCR_EN;
	ADC1->ISR = ADC_FLAG_OVR; ADC2->ISR = ADC_FLAG_OVR; //actually clears the flag (rc_w1)
}

bool adc_overall_complete(void)
{
	return (DMA_GetITStatus(DMA1_IT_TC1) != RESET && DMA_GetITStatus(DMA2_IT_TC1) != RESET);
}

void adc_overall_stop(void)
{
	DMA_Cmd(DMA1_Channel1, DISABLE); DMA_Cmd(DMA2_Channel1, DISABLE);
	DMA_ClearITPendingBit(DMA1_IT_TC1); DMA_ClearITPendingBit(DMA2_IT_TC1);
	if (ADC1->CR & ADC_CR_ADSTART) adc_stop(ADC1);
	if (ADC2->CR & ADC_CR_ADSTART) adc_stop(ADC2);
}

static inline float adc_to_voltage(float ad_val)
{
	return VRefInt * ad_val/ad_vrefint;
}

float adc_get_freq(void)
{
	uint32_t cnt = 0;
	TickCount = 0; //SysTick
	while (TickCount < 500) {
		DMA_SetCurrDataCounter(DMA1_Channel1, ADC_Buffer_Size);
		DMA_Cmd(DMA1_Channel1, ENABLE);
		adc_start(ADC1);
		
		while (DMA_GetITStatus(DMA1_IT_TC1) == RESET)
			if (TickCount >= 500) break;
		
		adc_stop(ADC1);
		cnt += ADC_Buffer_Size - DMA_GetCurrDataCounter(DMA1_Channel1);
		DMA_Cmd(DMA1_Channel1, DISABLE); DMA_ClearITPendingBit(DMA1_IT_TC1);
	}
	
	return (float)cnt / 500.0; //kHz
}

void adc_get_accuracy(uint8_t adc_sampletime, float* acc_worst, float* acc_av)
{
	adc_overall_start(); while (! adc_overall_complete()); adc_overall_stop();
	
	for (uint16_t i = 0; i < ADC_Buffer_Size; i++)
		buf[i] = (uint16_t)((float)buf[i] * ad_vrefint / buf_ref[i]);
	
	for (uint16_t i = 0; i < ADC_Output_Cnt; i++)
		av[i] = get_average(buf + i * ADC_Av_Cnt, ADC_Av_Cnt, ADC_Diff_Tol);
	
	*acc_worst = 12.0 - log(get_diff_max(av, ADC_Output_Cnt)) / log(2);
	*acc_av = 12.0 - log(get_diff_average(av, ADC_Output_Cnt)) / log(2);
}

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SysTick_Init();
	adc_overall_init();
	
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
				ad_vbat = adc_read_average(ADC1);
				if (ad_vbat >= AD_BatMin) {
					Delay(300);
					ad_vbat = adc_read_average(ADC1);
					if (ad_vbat < AD_BatMin) break;
					
					st = Measure_Connected;
					vbat = adc_to_voltage(ad_vbat);
					printf("Connected. Battery Voltage: %.3f V.\n", vbat);
					
					float acc_av, acc_worst;
					adc_get_accuracy(ADC_SampleTime_Default, &acc_worst, &acc_av);
					printf("ADC Accuracy: %.1f ~ %.1f Bits.\n\n", acc_worst, acc_av);
					
					adc_start(ADC1); adc_read(ADC1);
				}
				break;
			
			case Measure_Connected:
				ad_curr = adc_read(ADC1);
				
				if (ad_curr < AD_BatMin) {
					dis_cnt++; dec_cnt = 0;
					if (dis_cnt >= 2 * ADC_Av_Cnt) {
						st = Measure_Disconnected; dis_cnt = 0;
						printf("Disconnected.\n\n");
					}
				} else if (ad_curr < ad_vbat - AD_DecMin) {
					dec_cnt++; dis_cnt = 0;
					if (dec_cnt >= Dec_Cnt_Min) { //start DMA for capturing
						st = Measure_Capturing; dec_cnt = 0;
						adc_overall_start();
					}
				} else
					dis_cnt = dec_cnt = 0;
				
				break;

			case Measure_Capturing:
				if (! adc_overall_complete()) break;
				adc_overall_stop(); st = Measure_Later;
				
				for (uint16_t i = 0; i < ADC_Buffer_Size; i++)
					buf[i] = (uint16_t)((float)buf[i] * ad_vrefint / buf_ref[i]);
				
				for (uint16_t i = 0; i < ADC_Output_Cnt; i++)
					av[i] = adc_to_voltage(get_average(buf + i * ADC_Av_Cnt, ADC_Av_Cnt, ADC_Diff_Tol));
				
				vbat_dec = 0;
				for (uint16_t i = 1; i < ADC_Output_Cnt; i++) {
					if (av[i] > 0) {
						vbat_dec = av[i]; break;
					}
				}
				if (vbat - vbat_dec < VDecMin) { //false trigger
					st = Measure_Connected; adc_start(ADC1); adc_read(ADC1);
					break;
				} else if (vbat_dec < VBatMin) { //disconnect event
					if (av[ADC_Output_Cnt - 1] < VBatMin) {
						st = Measure_Disconnected;
						printf("Disconnected.\n\n");
					} else {
						st = Measure_Connected; adc_start(ADC1); adc_read(ADC1);
					}
					break;
				}
				
				printf("Captured: %.3f V. Voltage Drop: %.3f V\n", vbat_dec, vbat - vbat_dec); Delay(5);
				for (uint16_t i = 0; i < ADC_Output_Cnt; i++) {
					printf("%.4f ", av[i]); Delay(5);
				}
				printf("\n\n");
				break;
			
			case Measure_Later:
				ad_curr = adc_read_average(ADC1);
				if (ad_vbat - ad_curr < AD_DecMin / 4) {
					st = Measure_Connected; dis_cnt = 0;
					Delay(1000);
					ad_vbat = adc_read_average(ADC1);
					vbat = adc_to_voltage(ad_vbat);
					printf("Battery Voltage: %.3f V.\n\n", vbat);
					
					adc_start(ADC1); adc_read(ADC1);
				} else if (vbat_dec < VBatMin) {
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

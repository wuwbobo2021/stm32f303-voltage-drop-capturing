#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

//Add preprocessor symbols: USE_STDPERIPH_DRIVER, DEBUG
#include <stm32f30x.h>

#define VRefInt (1510.0 / 4095.0 * 3.3)
float ad_vrefint = 1.2 * 4095.0 / 3.3; //it will be updated
#define adc_to_voltage(ad_val) (VRefInt * ad_val/ad_vrefint)

#define GPIO_ADC GPIOA
#define RCC_AHBPeriph_GPIO_ADC RCC_AHBPeriph_GPIOA
#define GPIO_Pin_ADC GPIO_Pin_1
#define ADC_Channel_Bat ADC_Channel_2
#define ADC_SampleTime ADC_SampleTime_61Cycles5
#define ADC_Diff_Tol 96 //for distinguishing of connected and floating states

#define VBatMin 0.8
#define VDecMin 0.05
#define AD_BatMin (4096.0 * VBatMin / 3.3)
#define AD_DecMin (4096.0 * VDecMin / 3.3)

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
volatile float av[ADC_Output_Cnt];

// cnt should not exceed 0x200.
// Returns 0 if the difference between the min and max values
// of the remaining data is still larger than diff_max.
static float get_average(uint16_t* raw_data, uint16_t cnt, float diff_max)
{
	if (cnt == 0) return 0;
	uint16_t bound = cnt / 4;
	
	static uint16_t data[0x200]; uint32_t sum = 0;
	
	for (uint16_t i = 0; i < cnt; i++) {
		data[i] = raw_data[i];
		sum += raw_data[i];
	}
	
	uint16_t cnt_rem = cnt;
	uint16_t umin = 0, umax = 0;
	uint16_t tmin, tmax;
	for (uint16_t i = 0; i < bound + 1; i++) {
		umin = UINT16_MAX; umax = 0;
		
		for (uint16_t j = 0; j < cnt; j++) {
			if (data[j] == UINT16_MAX) continue;
			if (data[j] < umin) {
				tmin = j;
				umin = raw_data[j];
			}
			if (data[j] > umax) {
				tmax = j;
				umax = raw_data[j];
			}
		}
		
		if (i == bound) break; //at the time tmin and tmax are of the remaining data
		sum -= data[tmin]; sum -= data[tmax];
		cnt_rem -= 2;
		
		data[tmin] = data[tmax] = UINT16_MAX;
	}
	
	if (umax - umin > diff_max) return 0;
	return (float)sum / cnt_rem;
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

void adc_init()
{
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIO_ADC | RCC_AHBPeriph_ADC12, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_ADC;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIO_ADC, &GPIO_InitStruct);
	
	ADC_VoltageRegulatorCmd(ADC1, ENABLE);
	Delay(10);
	ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1) != RESET);
	ADC_GetCalibrationValue(ADC1);
	
	ADC_CommonInitTypeDef ADC_CommonInitStruct;
	ADC_CommonStructInit(&ADC_CommonInitStruct);
	ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStruct.ADC_Clock = ADC_Clock_AsynClkMode; //PLL->Prescaler->ADC, not from AHB HCLK
	ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStruct.ADC_TwoSamplingDelay = 0;
	ADC_CommonInit(ADC1, &ADC_CommonInitStruct);
	
	ADC_InitTypeDef ADC_InitStruct;
	ADC_StructInit(&ADC_InitStruct);
	ADC_InitStruct.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable; //continuous mode
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_AutoInjMode = ADC_AutoInjec_Disable;
	ADC_InitStruct.ADC_OverrunMode = ADC_OverrunMode_Disable;
	ADC_InitStruct.ADC_NbrOfRegChannel = 1;
	ADC_Init(ADC1, &ADC_InitStruct);
	
	ADC_VrefintCmd(ADC1, ENABLE); //18 VRefInt
	
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = ADC1_2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	ADC_AnalogWatchdog1SingleChannelConfig(ADC1, ADC_Channel_Bat);
	ADC_ITConfig(ADC1, ADC_IT_AWD1, ENABLE);
	
	ADC_Cmd(ADC1, ENABLE);
	while (! ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));
}

static inline void adc_discard_one() //skip the first data that might be invalid
{
	while ((ADC1->ISR & ADC_FLAG_EOC) == RESET);
	ADC1->DR;
}

float adc_read_accurate(uint8_t channel)
{
	const uint16_t times = 0x80;
	static uint16_t raw_data[0x80];
	
	ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_181Cycles5);
	ADC1->ISR &= ~ADC_FLAG_EOC;
	ADC_StartConversion(ADC1); adc_discard_one();
	
	for (uint8_t i = 0; i < times; i++) {
		while ((ADC1->ISR & ADC_FLAG_EOC) == RESET);
		raw_data[i] = ADC1->DR; //reading DR register clears the EOC flag
	}
	
	ADC_StopConversion(ADC1); while (ADC1->CR & ADC_CR_ADSTP);
	return get_average(raw_data, times, ADC_Diff_Tol);
}

float adc_get_freq(uint8_t adc_sampletime)
{
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Bat, 1, adc_sampletime);
	ADC_StartConversion(ADC1);
	
	uint32_t cnt = 0;
	TickCount = 0; //SysTick
	while (TickCount < 300) {
		while ((ADC1->ISR & ADC_FLAG_EOC) == RESET);
		ADC1->DR; cnt++;
	}
	
	ADC_StopConversion(ADC1); while (ADC1->CR & ADC_CR_ADSTP);
	return (float)cnt / 300.0; //kHz
}

float adc_get_accuracy(uint8_t adc_sampletime, bool get_worst)
{
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Bat, 1, adc_sampletime);
	ADC_StartConversion(ADC1); adc_discard_one();
	
	for (uint16_t i = 0; i < ADC_Buffer_Size; i++) {
		while ((ADC1->ISR & ADC_FLAG_EOC) == RESET);
		buf[i] = ADC1->DR;
	}
	
	ADC_StopConversion(ADC1); while (ADC1->CR & ADC_CR_ADSTP);
	
	for (uint16_t i = 0; i < ADC_Output_Cnt; i++)
		av[i] = get_average(buf + i * ADC_Av_Cnt, ADC_Av_Cnt, ADC_Diff_Tol);
	
	float diff;
	if (get_worst)
		diff = get_diff_max(av, ADC_Output_Cnt);
	else
		diff = get_diff_average(av, ADC_Output_Cnt);
	
	return 12.0 - log(diff) / log(2);
}

void ADC1_2_IRQHandler(void)
{
	if (ADC_GetITStatus(ADC1, ADC_IT_AWD1) != RESET) {
		ADC_ClearITPendingBit(ADC1, ADC_IT_AWD1);
		if (st == Measure_Connected) {
			ADC_StopConversion(ADC1); while ((ADC1->CR & ADC_CR_ADSTP) != RESET);
			ADC_AnalogWatchdogCmd(ADC1, ADC_AnalogWatchdog_None);
			ADC_StartConversion(ADC1); adc_discard_one();
			st = Measure_Capturing;
		}
	}
}

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SysTick_Init();
	adc_init(); ad_vrefint = adc_read_accurate(ADC_Channel_18);
	
	float freq = adc_get_freq(ADC_SampleTime);
	printf("Current ADC Read Frequency: %.3f kHz.\nInterval of output data will be: %.3f us.\n",
	       freq, 1000.0 / (freq / ADC_Av_Cnt));
	
	float ad_vbat = 0, vbat = 0;
	float vbat_dec = 0;
	uint16_t dcnt = 0;
	
	while (true) {
		switch (st) {
			case Measure_Disconnected:
				ad_vbat = adc_read_accurate(ADC_Channel_Bat);
				if (ad_vbat >= AD_BatMin) {
					st = Measure_Connected;
					Delay(100);
					ad_vbat = adc_read_accurate(ADC_Channel_Bat);
					vbat = adc_to_voltage(ad_vbat);
					printf("Connected. Battery Voltage: %.3f V.\n", vbat);
					printf("ADC Accuracy: %.3f ~ %.3f Bits.\n\n",
					       adc_get_accuracy(ADC_SampleTime, true),
						   adc_get_accuracy(ADC_SampleTime, false));
					
					ADC_AnalogWatchdog1ThresholdsConfig(ADC1, 4095, ad_vbat - AD_DecMin);
					ADC_AnalogWatchdogCmd(ADC1, ADC_AnalogWatchdog_SingleRegEnable);
					
					ADC_RegularChannelConfig(ADC1, ADC_Channel_Bat, 1, ADC_SampleTime);
					ADC_StartConversion(ADC1); adc_discard_one();
				}
				break;
			
			case Measure_Connected: break; //rely on the watchdog

			case Measure_Capturing:
				for (uint16_t i = 0; i < ADC_Buffer_Size; i++) {
					while ((ADC1->ISR & ADC_FLAG_EOC) == RESET);
					buf[i] = ADC1->DR;
				}
				ADC_StopConversion(ADC1); while ((ADC1->CR & ADC_CR_ADSTP) != RESET);
				ADC_AnalogWatchdogCmd(ADC1, ADC_AnalogWatchdog_None);
				st = Measure_Later;
				
				for (uint16_t i = 0; i < ADC_Output_Cnt; i++)
					av[i] = adc_to_voltage(get_average(buf + i * ADC_Av_Cnt, ADC_Av_Cnt, ADC_Diff_Tol));
				
				vbat_dec = 0;
				for (uint16_t i = 1; i < ADC_Output_Cnt; i++) {
					if (av[i] > 0) {
						vbat_dec = av[i]; break;
					}
				}
				
				if (vbat - vbat_dec < VDecMin) { //false trigger
					ADC_AnalogWatchdogCmd(ADC1, ADC_AnalogWatchdog_SingleRegEnable);
					st = Measure_Connected; ADC_StartConversion(ADC1);
					break;
				} else if (vbat_dec < VBatMin) { //disconnect event
					if (av[ADC_Output_Cnt - 1] < VBatMin) {
						st = Measure_Disconnected;
						printf("Disconnected.\n\n");
					} else {
						ADC_AnalogWatchdogCmd(ADC1, ADC_AnalogWatchdog_SingleRegEnable);
						st = Measure_Connected; ADC_StartConversion(ADC1);
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
				vbat_dec = adc_to_voltage(adc_read_accurate(ADC_Channel_Bat));
				if (vbat - vbat_dec < VDecMin / 4) {
					st = Measure_Connected; dcnt = 0;
					Delay(100);
					ad_vbat = adc_read_accurate(ADC_Channel_Bat);
					vbat = adc_to_voltage(ad_vbat);
					printf("Battery Voltage: %.3f V.\n\n", vbat);
					
					ADC_AnalogWatchdog1ThresholdsConfig(ADC1, 4095, ad_vbat - AD_DecMin);
					ADC_AnalogWatchdogCmd(ADC1, ADC_AnalogWatchdog_SingleRegEnable);
					ADC_StartConversion(ADC1); adc_discard_one();
				} else if (vbat_dec < VBatMin) {
					dcnt++;
					if (dcnt >= 3) {
						st = Measure_Disconnected; dcnt = 0;
						printf("Disconnected.\n\n");
					}
				} else dcnt = 0;
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

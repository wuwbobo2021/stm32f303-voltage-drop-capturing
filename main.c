#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

//Add preprocessor symbols: USE_STDPERIPH_DRIVER, DEBUG
#include <stm32f30x.h>

#define VRefInt (1510.0 / 4095.0 * 3.3)
static float ad_vrefint = 1.2 * 4095.0 / 3.3; //it will be updated
#define adc_to_voltage(ad_val) (VRefInt * ad_val/ad_vrefint)

#define GPIO_ADC GPIOA
#define RCC_AHBPeriph_GPIO_ADC RCC_AHBPeriph_GPIOA
#define GPIO_Pin_ADC GPIO_Pin_1
#define ADC_Channel_Bat ADC_Channel_2

#define ADC_SampleTime_Bat ADC_SampleTime_7Cycles5

#define VBatMin 0.8
#define VDecMin 0.05
#define AD_BatMin (4096.0 * VBatMin / 3.3)
#define AD_DecMin (4096.0 * VDecMin / 3.3)

#define ADC_Cnt_Measure 32
#define Capture_Buffer_Size 60

static volatile uint32_t TickCount = 0; //SysTick
static volatile uint32_t TimingDelay;

static volatile float watch_vadc = 0;

static inline float prev_item(float* arr, uint16_t size, uint16_t dist, uint16_t cur)
{
	int16_t i = cur - dist;
	while (i < 0) i = size + i;
	return arr[i];
}

static inline float get_average(uint16_t* raw_data, uint16_t cnt)
{
	uint32_t sum = 0;
	for (uint16_t i = 0; i < cnt; i++)
		sum += raw_data[i];
	return (float)sum / cnt;
}

// Remove 2 * bound extreme values. cnt should not exceed 0x200.
// Returns 0 if the difference between the min and max values
// of the remaining data is still larger than diff_max.
static float get_stable_average(uint16_t* raw_data, uint16_t cnt, uint16_t bound, float diff_max)
{
	if (cnt == 0) return 0;
	if (bound * 2 >= cnt) bound = 0;
	
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
		
		if (i == bound) break; //after breaking, tmin and tmax are values of remaining data
		sum -= data[tmin]; sum -= data[tmax];
		cnt_rem -= 2;
		
		data[tmin] = data[tmax] = UINT16_MAX;
	}
	
	if (umax - umin > diff_max) return 0;
	return (float)sum / cnt_rem;
}

static void SysTick_Init()
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

static inline void Delay(volatile uint32_t nTime)
{ 
  TimingDelay = nTime;
  while (TimingDelay > 0);
}

void SysTick_Handler(void)
{
	TickCount++;
	
	if (TimingDelay > 0)
		TimingDelay--;
}

static void adc_init()
{
	GPIO_InitTypeDef GPIO_InitStruct;
	ADC_CommonInitTypeDef ADC_CommonInitStruct;
	ADC_InitTypeDef ADC_InitStruct;
	uint32_t calibration_value = 0;
	
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIO_ADC | RCC_AHBPeriph_ADC12, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_ADC;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIO_ADC, &GPIO_InitStruct);
	
	ADC_VoltageRegulatorCmd(ADC1, ENABLE);
	Delay(10);
	ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1) != RESET);
	calibration_value = ADC_GetCalibrationValue(ADC1);
	
	ADC_CommonStructInit(&ADC_CommonInitStruct);
	ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStruct.ADC_Clock = ADC_Clock_AsynClkMode; //PLL->Prescaler->ADC, not from AHB HCLK
	ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStruct.ADC_TwoSamplingDelay = 0;
	ADC_CommonInit(ADC1, &ADC_CommonInitStruct);
	
	ADC_StructInit(&ADC_InitStruct);
	ADC_InitStruct.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable; //continuous mode
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_AutoInjMode = ADC_AutoInjec_Disable;
	ADC_InitStruct.ADC_OverrunMode = ADC_OverrunMode_Disable;
	ADC_InitStruct.ADC_NbrOfRegChannel = 1;
	ADC_Init(ADC1, &ADC_InitStruct);
	
	ADC_TempSensorCmd(ADC1, ENABLE); //16 temperature sensor
	ADC_VrefintCmd(ADC1, ENABLE); //18 VRefInt
	ADC_VbatCmd(ADC1, ENABLE); //17 VBAT
	
	ADC_Cmd(ADC1, ENABLE);
	while (! ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));
}

static void adc_get_refint(void)
{
	const uint16_t times = 0x80;
	static uint16_t raw_data[0x80];
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_18, 1, ADC_SampleTime_181Cycles5);
	
	ADC1->ISR &= ~ADC_FLAG_EOC;
	ADC_StartConversion(ADC1);
	
	while ((ADC1->ISR & ADC_FLAG_EOC) == RESET);
	raw_data[0] = ADC1->DR; //skip the first data that might be invalid
	
	for (uint8_t i = 0; i < times; i++) {
		while ((ADC1->ISR & ADC_FLAG_EOC) == RESET);
		raw_data[i] = ADC1->DR; //reading DR register clears the EOC flag
	}
	
	ADC_StopConversion(ADC1);
	while (ADC1->CR & ADC_CR_ADSTP);
	
	ad_vrefint = 0;
	while (ad_vrefint == 0.0)
		ad_vrefint = get_stable_average(raw_data, times, times / 4, 8);
}

static inline void adc_discard_one() //skip the first data that might be invalid
{
	while ((ADC1->ISR & ADC_FLAG_EOC) == RESET);
	ADC1->DR;
}

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SysTick_Init();
	
	adc_init();
	adc_get_refint();
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Bat, 1, ADC_SampleTime_Bat);
	ADC_StartConversion(ADC1);
	adc_discard_one();
	
	static uint16_t buf[ADC_Cnt_Measure]; uint16_t ibuf = 0;
	static float cap[Capture_Buffer_Size]; uint16_t icap = 0;
	bool flag_refresh_vbat = true;
	bool flag_measure_read_freq = true; uint32_t cntf = 0;
	bool flag_load = false;
	float vbat = 0, vbat_load = 0;
	TickCount = 0; //SysTick
	
	while (true) {
		while ((ADC1->ISR & ADC_FLAG_EOC) == RESET);
		buf[ibuf] = ADC1->DR;
		
		if (buf[ibuf] == 0 || buf[ibuf] >= 4095) continue;
		
		ibuf++;
		if (ibuf < ADC_Cnt_Measure) continue;
		
		watch_vadc = adc_to_voltage((get_stable_average(buf, ADC_Cnt_Measure, ADC_Cnt_Measure / 4, 8)));
		if (watch_vadc <= VBatMin / 2.0 || watch_vadc >= 3.3) {
			ibuf = 0; continue;
		}

		cap[icap] = watch_vadc;
		if (flag_measure_read_freq) cntf++;
		
		if (! flag_load) {
			if (cap[icap] < vbat && vbat - cap[icap] >= VDecMin) {
				flag_load = true;
				float tmp0 = prev_item(cap, Capture_Buffer_Size, 2, icap),
					  tmp1 = prev_item(cap, Capture_Buffer_Size, 1, icap);
				cap[0] = tmp0; cap[1] = tmp1; icap = 2; //keep 2 previous data
			} else {
				if (flag_refresh_vbat && TickCount >= 500) {
					vbat = watch_vadc; flag_refresh_vbat = false;
					printf("vbat: %f\n\n", vbat);
				}
				icap++; if (icap == Capture_Buffer_Size) icap = 0;
			}
		} else {			
			if (icap < Capture_Buffer_Size)
				cap[icap++] = watch_vadc;
			
			if (icap == 4) //2nd data after the decline
				vbat_load = watch_vadc;
			
			if (watch_vadc > vbat || vbat - watch_vadc < VDecMin / 4.0) {
				printf("vbat_load: %f\n", vbat_load);
				for (uint16_t i = 0; i < icap; i++) {
					printf("%f ", cap[i]); Delay(5);
				}
				printf("\n\n");
				
				flag_load = false; icap = 0;
				flag_refresh_vbat = true; TickCount = 0; 
			}
		}
		
		if (flag_measure_read_freq) {
			cntf++;
			if (TickCount >= 1000) { //1s
				printf("Current ADC Accurate Reading Frequency: %f kHz\n", (float)cntf / 1000.0);
				printf("interval of output data will be %d us.\n\n",
				       (uint16_t)(1000.0 * 1000.0 / (float)cntf));
				flag_measure_read_freq = false;
			}
		}
		
		ibuf = 0;
	}
}

extern void assert_param(uint8_t expr)
{
#ifdef DEBUG
	if (! expr)	while (1);
#else
#endif
}
;
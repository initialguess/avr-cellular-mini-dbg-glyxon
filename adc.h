// This is a guard condition so that contents of this file are not included more than once.  
 #ifndef ADC_H
 #define	ADC_H

// function prototypes
void ADC_Init(VREF_REFSEL_t , uint8_t , uint8_t , ADC_CONVMODE_t , uint8_t , ADC_RESSEL_t , uint8_t , ADC_INITDLY_t , ADC_SAMPNUM_t , ADC_PRESC_t , ADC_SAMPDLY_t , uint8_t , ADC_MUXPOS_t , ADC_MUXNEG_t , ADC_WINCM_t , uint16_t , uint16_t);
void ADC_Enable(void);
void ADC_Disable(void);
void ADC_Start_Conversion(void);
void ADC_Stop_Conversion(void);
uint8_t ADC_Is_Conversion_Done(void);
uint8_t ADC_Is_Window_Satisfied(void);
uint16_t ADC_Get_Result(void);
void ADC_Start_Oversampled_Conversion(ADC_SAMPNUM_t);
uint16_t ADC_Get_Oversampled_Result(uint8_t);


 #endif	


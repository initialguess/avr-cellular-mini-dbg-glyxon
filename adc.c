#include <Arduino.h>
#include "adc.h"

uint8_t bitshift;

void ADC_Init(VREF_REFSEL_t vref, uint8_t vref_always_on, uint8_t runstndby, ADC_CONVMODE_t convmode, uint8_t leftadj,
        ADC_RESSEL_t ressel, uint8_t freerun, ADC_INITDLY_t initdly, ADC_SAMPNUM_t sampnum, ADC_PRESC_t presc,
        ADC_SAMPDLY_t sampdly, uint8_t samplen, ADC_MUXPOS_t muxpos, ADC_MUXNEG_t muxneg, ADC_WINCM_t wincm, uint16_t winlt, uint16_t winht) {
    
    VREF.ADC0REF = (vref << VREF_REFSEL_gp) | (vref_always_on << VREF_ALWAYSON_bp);
    
    ADC0.CTRLA = (runstndby << ADC_RUNSTBY_bp) | (convmode << ADC_CONVMODE_bp) | (leftadj << ADC_LEFTADJ_bp) 
            | (ressel << ADC_RESSEL_gp) | (freerun << ADC_FREERUN_bp);
    
    ADC0.CTRLB = (sampnum << ADC_SAMPNUM_gp);   
    ADC0.CTRLC = (presc << ADC_PRESC_gp);   
    ADC0.CTRLD = ((sampdly << ADC_SAMPDLY_gp) | (initdly << ADC_INITDLY_gp));   
    ADC0.SAMPCTRL = samplen << ADC_SAMPLEN_gp;
    if (convmode == ADC_CONVMODE_DIFF_gc)   {
        ADC0.MUXNEG = (muxneg << ADC_MUXNEG_gp);
    }
    ADC0.MUXPOS = (muxpos << ADC_MUXPOS_gp);  
    
    // if Window Cmp mode being used
    if (wincm != ADC_WINCM_NONE_gc) {
        // set Window Compare high and low thresholds
        ADC0.WINLT = winlt;
        ADC0.WINHT = winht;
        // set Window Compare mode 
        ADC0.CTRLE = (wincm << ADC_WINCM_gp); 
    }
}

void ADC_Enable(void)   {
    ADC0.CTRLA |= (1 << ADC_ENABLE_bp);
}

void ADC_Disable(void)  {
    ADC0.CTRLA &= ~(1 << ADC_ENABLE_bp);
}

void ADC_Start_Conversion(void) {
    ADC0.COMMAND |= ADC_STCONV_bm;
}

void ADC_Stop_Conversion(void)  {
    ADC0.COMMAND |= ADC_SPCONV_bm;
}

uint8_t ADC_Is_Conversion_Done(void)    {
    if(ADC0.INTFLAGS & ADC_RESRDY_bm)   {
        return 1;
    }
    return 0;
}

uint8_t ADC_Is_Window_Satisfied(void)   {
    if(ADC0.INTFLAGS & ADC_WCMP_bm) {
        return 1;
    }
    return 0;
}

uint16_t ADC_Get_Result(void)   {
    return ADC0.RES;
}

void ADC_Start_Oversampled_Conversion(ADC_SAMPNUM_t sampnum)    {
    ADC_Disable();
    
    ADC0.CTRLB = (sampnum << ADC_SAMPNUM_gp);
    
    ADC_Enable();
    
    ADC_Start_Conversion();
}    

uint16_t ADC_Get_Oversampled_Result(uint8_t enob)   {
    
    // get current resolution from ADC0 CTRLA register and calculate
    // diff between required enob and current ADC resolution setting
    
    // if ADC set to 12-bit result
    if(((ADC0.CTRLA & ADC_RESSEL_gm) >> 2) == 0x00) {
        bitshift = enob - 12;
    }
    // if ADC set to 10-bit resolution
    else if (((ADC0.CTRLA & ADC_RESSEL_gm) >> 2) == 0x01)   {
        bitshift = enob - 10;
    }
    // do bitshift to get required result
    return (ADC0.RES >> bitshift);
}

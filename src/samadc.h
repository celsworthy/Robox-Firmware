#ifndef SAMADC_H
#define SAMADC_H

void init_adc(void);
unsigned int adc_read_nozzle0(void);
unsigned int adc_read_nozzle1(void);
unsigned int adc_read_bed(void);
unsigned int adc_read_ambient(void);
void adc_start_conversion(void);
unsigned char are_adc_conversions_ready(void);

#endif

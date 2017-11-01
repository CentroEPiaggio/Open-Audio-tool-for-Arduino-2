#include <Streaming.h>
#include "Arduino_Due_IADC.h"

IADC ImADC;

// Override standard ADC_Handler with IADC version
void ADC_Handler(){
    ImADC.ADCInterrupt();
}

void setup() {
    // Serial
    SerialUSB.begin(0);

    ImADC.Init(200000);
    ImADC.Start();
}

uint16_t tmp, tmp2[IADC_BUFFER_SIZE];
void loop() {
    if(ImADC.DataAvailable()){
      tmp = ImADC.GetData(tmp2);
      //SerialUSB.println(millis());
      //SerialUSB.write((uint8_t*)tmp2,IADC_BUFFER_SIZE);
      for(int i = 0 ; i < IADC_BUFFER_SIZE ; i++)
        SerialUSB.println(tmp2[i]);
    }
}

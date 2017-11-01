#include "Arduino_Due_IADC.h"

IADC::IADC() : IADC(100000){
    // Set default sampling frequency at 100 ksps
}

IADC::IADC(uint32_t freq) :
    fs(freq),
    buffer_index(0),
    buffer_ready_index(IADC_BUFFERS),
    data_ready(false),
    adc_running(false),
    buffer_overflow(false) {

        // We cannot call Init here, because we cannot know when the constructor
        // will be called, and if the peripherials are ready to be initialized.

    return;
}

void IADC::InitADC(){
    // ADC
        // Reset
    ADC->ADC_CR = ADC_CR_SWRST;
    ADC->ADC_MR = 0;

        // Disable data transfer
    ADC->ADC_PTCR = ( ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS );

        // Timing options
    ADC->ADC_MR |= ADC_MR_PRESCAL(0);       // MCK/2 prescaler
    ADC->ADC_MR |= ADC_MR_STARTUP_SUT0;     // Startup cycles
    ADC->ADC_MR |= ADC_MR_TRACKTIM(15);     // Tracking cycles
    ADC->ADC_MR |= ADC_MR_TRANSFER(1);      // Transfer cycles
        // Trigger options
    ADC->ADC_MR |= ADC_MR_TRGEN_EN;         // Enable trigger
    ADC->ADC_MR |= ADC_MR_TRGSEL_ADC_TRIG1; // Select TIOA0 as trigger.

        // Set pin A7
    ADC->ADC_CHER = ADC_CHER_CH0;

        // Disable all ADC interrupt but ENDRX
    ADC->ADC_IDR = ~ADC_IDR_ENDRX;
    ADC->ADC_IER = ADC_IER_ENDRX;

    return;
}

void IADC::InitDMA(){
    // ADC DMA
        // Init indexes
    buffer_index = 0;
    buffer_ready_index = IADC_BUFFERS;
        // Init first Buffer
    ADC->ADC_RPR = (uint32_t) data_buffer[buffer_index];
    ADC->ADC_RCR = (uint16_t) IADC_BUFFER_SIZE;
        // Init second buffer
    SetNextBuffer();

    return;
}

void IADC::InitTimer(){
    // Timer Counter
        // rc represent the number of timer cycles referred to half Master clock
    uint32_t cycles = 42000000 / fs;        // ( MCK/(2*fs) )

        // TC enable
    pmc_enable_periph_clk(ID_TC0);

        // TC setup
    TC_Configure(TC0, 0, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_ACPA_CLEAR |
      TC_CMR_ACPC_SET | TC_CMR_ASWTRG_CLEAR | TC_CMR_TCCLKS_TIMER_CLOCK1);
    TC_SetRC( TC0, 0, cycles );   // Set sampling frequency (in MCK/2 cycles)
    TC_SetRA( TC0, 0, cycles/2 ); // 50% duty cycle

    return;
}

void IADC::StartTimer(){
    // Start timer
    TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN; // Clock enable
    TC0->TC_BCR = TC_BCR_SYNC;                // Channel sync, because why not?
    TC_Start(TC0, 0);                         // Start timer

    return;
}

void IADC::StopTimer(){
    // Stop timer
    TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKDIS;  // Clock disable
    TC_Stop(TC0, 0);                            // Stop timer

    return;
}

void IADC::Init(){
    if(adc_running)
        return;

    // Init stuff
    InitTimer();
    InitADC();

    return;
}

void IADC::Start(){
    // Start ADC if it is not already running
    if(adc_running)
        return;

    // Re-init DMA
    InitDMA();

    NVIC_EnableIRQ( ADC_IRQn );       // Enable interrupts
    ADC->ADC_PTCR = ADC_PTCR_RXTEN;   // Enable receiving data
    ADC->ADC_CR |= ADC_CR_START;      // Start converting

    // Start timer
    StartTimer();

    // Set flags
    data_ready = false;
    adc_running = true;
    buffer_overflow = false;

    return;
}

void IADC::Stop(){
    // Stop ADC if it is running
    if(!adc_running)
        return;

    StopTimer();

    NVIC_DisableIRQ( ADC_IRQn );
    ADC->ADC_PTCR = ( ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS );
    ADC->ADC_CR = ADC_CR_SWRST;

    // Set flags
    adc_running = false;

    return;
}

void IADC::ADCInterrupt(){
    uint32_t status = ADC->ADC_ISR;
    // ADC handler
    if( status & ADC_ISR_ENDRX ){ // Check what interrupt called it
        buffer_index = (buffer_index + 1) % IADC_BUFFERS;
        SetNextBuffer();
        data_ready = true;
        if(buffer_index == buffer_ready_index)
            buffer_overflow = true;
    }

    return;
}

void IADC::SetNextBuffer(){
    // Set next buffer
    buffer_index = (buffer_index + 1) % IADC_BUFFERS;
    ADC->ADC_RNPR = (uint32_t) data_buffer[buffer_index];
    ADC->ADC_RNCR = (uint16_t) IADC_BUFFER_SIZE;

    return;
}

bool IADC::ChangeSamplingFrequency(uint32_t samplingfreq){
    // Check if adc is not already running
    if(adc_running)
        return false;

    // Check if the new fs is inside the hard coded bounds
    if(samplingfreq > IADC_MAXFREQ || samplingfreq < IADC_MINFREQ )
        return false;   // Do nothing

    // Change fs
    fs = samplingfreq;
    InitTimer();

    return true;
}

int8_t IADC::GetData(uint16_t *data){
    // Return first available data

    if(!data_ready) // Check if there is available data
        return -1;  // -1 means "no new data"

    // buffer_ready_index is initialised outside buffer range
    buffer_ready_index = buffer_ready_index % IADC_BUFFERS;
    for ( uint32_t i = 0 ; i < IADC_BUFFER_SIZE ; i++ )
        data[i] = data_buffer[buffer_ready_index][i];

    // Increase next available buffer index
    buffer_ready_index = (buffer_ready_index + 1) % IADC_BUFFERS;

    // Number of unread buffers
    int8_t buff_n = (buffer_index - buffer_ready_index) % IADC_BUFFERS;

    // If there are no more buffer available, set data ready to false
    if(buff_n == 0)
        data_ready = false;

    return buff_n;  // Return the number of available buffers
}

// Flag check functions
bool IADC::DataAvailable(){
    return data_ready;
}

bool IADC::IsRunning(){
    return adc_running;
}

bool IADC::BufferOverflow(){
    return buffer_overflow;
}

/* [] END OF FILE */

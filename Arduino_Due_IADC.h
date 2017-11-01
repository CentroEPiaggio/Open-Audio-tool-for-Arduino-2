/********************************************************************************

Arduino Due Improved ADC

********************************************************************************/
//#if defined (_VARIANT_ARDUINO_DUE_X_)
#ifndef Arduino_Due_IADC
#define Arduino_Due_IADC
#include <Arduino.h>

// Buffer parameters
#define IADC_BUFFERS        4       // Number of buffers, cannot be set to
                                    // anything less than 2
#define IADC_BUFFER_SIZE    256     // Buffer size
#define IADC_BUFFER_MEMORY  2 * IADC_BUFFER_SIZE * IADC_BUFFERS
                                    // Total RAM used by buffers

// Frequency limits
#define IADC_MAXFREQ        1000000
#define IADC_MINFREQ        0.01

class IADC{
    public:
        // Constructors
        IADC();
        IADC(uint32_t freq);

        // Init function, it has to be called in order to start acquisitions
        void Init();

        // Start/stop acquisitions
        void Start();
        void Stop();

        // Change sampling frequency
        bool ChangeSamplingFrequency(uint32_t samplingfreq);

        // Get first available buffer, and return how many buffer are still
        // available to be read
        int8_t GetData(uint16_t *data);

        // Interrupt handle, required to override the standard arduino interrupt
        void ADCInterrupt();

        // Check if there is new data
        bool DataAvailable();
        // Check if ADC is running
        bool IsRunning();
        // Check if there was a buffer overflow
        bool BufferOverflow();

    private:
        // Sampling frequency
        uint32_t fs;
        // Buffer
        uint16_t data_buffer[IADC_BUFFERS][IADC_BUFFER_SIZE];
        // Current buffer index
        uint8_t buffer_index;
        // First available buffer index (used to get data)
        uint8_t buffer_ready_index;

        // Flags
        volatile bool data_ready;       // New data is ready
        bool adc_running;               // ADC is running
        bool buffer_overflow;           // The first available buffer has been
                                        // overwritten

        // Private functions.
        void InitADC();
        void InitTimer();
        void InitDMA();
        void SetNextBuffer();
        void StartTimer();
        void StopTimer();
};

#endif
//#endif
/* [] END OF FILE */

// Arduino standard library
#include <Arduino.h>

// Board preprocessor defintions
#ifdef ARDUINO_NRF52840_CIRCUITPLAY
#define ADAFRUIT_CIRCUITPLAYGROUND
#define NRF52_TIMER2
#define AUDIO_CPU
#else // ADAFRUIT_CIRCUITPLAYGROUND_BLUEFRUIT
#ifdef ADAFRUIT_CIRCUITPLAYGROUND_M0
#define ADAFRUIT_CIRCUITPLAYGROUND
#define SAMD_TC4
#define AUDIO_CPU
#else // Not Circuit Playground Express
#if !defined(ARDUINO_NRF52840_CIRCUITPLAY) and !defined(ADAFRUIT_CIRCUITPLAYGROUND_M0)
#define RTOS
#define AUDIO_I2S
#endif 
#endif // ADAFRUIT_CIRCUITPLAYGROUND_M0 
#endif // ADAFRUIT_CIRCUITPLAYGROUND_BLUEFRUIT
#ifdef ADAFRUIT_CIRCUITPLAYGROUND
#define TWO_BUTTONS
#define SPDT_SWITCH
#endif // ADAFRUIT_CIRCUITPLAYGROUND

// Firmware version
#define VERSION F("0.3")

// For USB
#ifdef ARDUINO_NRF52840_CIRCUITPLAY
// #define USB_DISABLE
#define USB_SUSPEND_KEEP_HFCLK
#endif // ARDUINO_NRF52840_CIRCUITPLAY

// For Clock
#ifdef ARDUINO_NRF52840_CIRCUITPLAY
#define ENABLE_HFCLK // Enables high frequency clock when metronome is ticking, required to keep high accuracy when powered by battery
// #define FORCE_HFCLK // Not optimal, but forces high frequency clock every loop to prevent USB suspend event from using lower accuracy internal RC oscillator
#define LFCLK_CALIBRATION_TIME 16 // Calibrate for 4 seconds (16 * 0.25 seconds)
#endif // ARDUINO_NRF52840_CIRCUITPLAY

// For Serial
#ifndef USB_DISABLE
// #define SERIAL_DEBUG
#define SERIAL_DEBUG_WAIT_INIT
#ifdef SERIAL_DEBUG
#define BAUD_RATE 115200
#ifdef ARDUINO_NRF52840_CIRCUITPLAY
#include <Adafruit_TinyUSB.h>
#endif // ARDUINO_NRF52840_CIRCUITPLAY
#endif // SERIAL_DEBUG
#endif // USB_DISABLE

// For Sound
#ifdef RTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif // RTOS
#ifdef AUDIO_I2S
#include <Audio.h>
#include <SPIFFS.h>
#endif // AUDIO_I2S

// For Display
#ifdef ADAFRUIT_CIRCUITPLAYGROUND
#include <Wire.h>
#else
#include <SPI.h>
#endif // ADAFRUIT_CIRCUITPLAYGROUND
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// Origin offset in pixels
#define ORIGIN_OFFSET_X 0
#define ORIGIN_OFFSET_Y 8
#define ORIGIN_OFFSET_DOT_X ORIGIN_OFFSET_X / 4
#define ORIGIN_OFFSET_DOT_Y ORIGIN_OFFSET_Y / 4
// Dot matrix coordinates
#define TEMPO_DOTS_X 8 + ORIGIN_OFFSET_DOT_X
#define TEMPO_DOTS_Y 0 + ORIGIN_OFFSET_DOT_Y
#define UPPER_TIME_SIGNATURE_X 0 + ORIGIN_OFFSET_DOT_X
#define UPPER_TIME_SIGNATURE_Y 0 + ORIGIN_OFFSET_DOT_Y
#define LOWER_TIME_SIGNATURE_X 0 + ORIGIN_OFFSET_DOT_X
#define LOWER_TIME_SIGNATURE_Y 7 + ORIGIN_OFFSET_DOT_Y
// Bitmaps
#define BPM_BITMAP_X 106 + ORIGIN_OFFSET_X 
#define BPM_BITMAP_Y 16 + ORIGIN_OFFSET_Y
#include "bpm_bitmap.c"
#define SWG_BITMAP_X 104 + ORIGIN_OFFSET_X
#define SWG_BITMAP_Y 16 + ORIGIN_OFFSET_Y
#include "swg_bitmap.c"
#define VOLUME_BITMAP_X 36 + ORIGIN_OFFSET_X
#define VOLUME_BITMAP_Y 43 + ORIGIN_OFFSET_Y
#define VOLUME_DOTS_LENGTH 15
#define VOLUME_DOTS_START_X 10 + ORIGIN_OFFSET_X / 4
#define VOLUME_DOTS_Y 11 + ORIGIN_OFFSET_Y / 4
#include "volume_bitmap.c"

// For NeoPixel
#define NEOPIXEL
#ifdef NEOPIXEL
#include <Adafruit_NeoPixel.h>
#endif //NEOPIXEL

// For Input

// Serial control commands
#ifdef SERIAL_DEBUG
#define SERIAL_CONTROL
#ifdef SERIAL_CONTROL
#define SERIAL_CONTROL_PRESS 'p'
#define SERIAL_CONTROL_LONG_PRESS 'l'
#define SERIAL_CONTROL_TURN_CW 't'
#define SERIAL_CONTROL_TURN_CCW 'r'
#define SERIAL_CONTROL_DEBUG 'd'
#define SERIAL_CONTROL_ETC 'e'
#define SERIAL_CONTROL_SPECIAL 's'
#endif // SERIAL_CONTROL
#endif // SERIAL_DEBUG

// Buttons and switch
#ifdef TWO_BUTTONS
//#define PRESSED_LED // Use builtin LED to indicate short press
#define BUTTON_A 5
#define BUTTON_B 4
#define BUTTON_READ_PERIOD 150UL
#define BUTTON_DOUBLE_PRESS_PERIOD 300UL // Must be multiple of read period for accuracy
#endif // TWO_BUTTONS
#ifdef SPDT_SWITCH
#define SWITCH_A 7
#endif // SPDT_SWITCH

// VCC and temperature debugging
//#define VCC_AND_TEMP_DEBUG

// Rotary encoder
// Pin A, Pin B, Button Pin
//#define ROTARY_ENCODER
#ifdef ROTARY_ENCODER
#include <SimpleRotary.h>
#define ROTARY_A 14
#define ROTARY_B 15
#define ROTARY_BUTTON 2
SimpleRotary rotary(ROTARY_A, ROTARY_B, ROTARY_BUTTON);
#endif // ROTARY_ENCODER

// Input
byte turn;
byte pressed;
byte long_pressed;
// Direction
#define CW 1
#define CCW 2

// LEDs
// #define SETUP_LED // Turns on LED_BUILTIN during initialization to indicate setup time
// #define HFCLK_SRC_LED // Sets LED_BUILTIN to the value of the SRC bit of HFCLKSTAT, indicating clock source
#ifdef NEOPIXEL
#define SEIZURE_PREVENTION
//#define SEIZURE_PREVENTION_TESTING A2
#ifdef ADAFRUIT_CIRCUITPLAYGROUND
#define NEOPIXEL_PIN 8
#define NEOPIXEL_COUNT 10
#define NEOPIXEL_ROTATE
#define NEOPIXEL_COLOR
#define NEOPIXEL_WHITE
#else // Not Circuit Playground board
#define NEOPIXEL_PIN 22
#define NEOPIXEL_COUNT 16
#endif // ADAFRUIT_CIRCUITPLAYGROUND
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
#if defined(NEOPIXEL_COLOR) && defined(NEOPIXEL_WHITE)
#define NEOPIXEL_COLOR_COUNT 6
uint8_t neopixel_color_offset = 0;
#endif // NEOPIXEL_COLOR && NEOPIXEL_WHITE
#endif // NEOPIXEL

// Display
#define OLED
#define LED_MATRIX_SIM
#ifdef OLED
#define SSD1306
#ifdef SSD1306
#define OLED_INIT_DELAY  100
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_ROTATION 0 // 90 degree intervals
#ifdef ADAFRUIT_CIRCUITPLAYGROUND
#define OLED_I2C
#ifdef OLED_I2C
#define OLED_ADDR 0x3C
#define OLED_RESET  -1
#define OLED_SCK    A4
#define OLED_SDA    A5
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
#endif // OLED_I2C
#else // ESP32
#define OLED_SPI
#ifdef OLED_SPI
#define OLED_MOSI   23
#define OLED_CLK    18
#define OLED_DC     17
#define OLED_CS     19
#define OLED_RESET  5
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT,
    OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
#endif // OLED_SPI
#endif // ADAFRUIT_CIRCUITPLAYGROUND
#endif // SSD1306
#ifdef LED_MATRIX_SIM
#include "LEDMatSim.cpp"
LEDMatSim ledmatsim(display, 32, 16);

// LED Matrix simulation dot configuration demo
typedef uint8_t bits_t;
bits_t new_dots_wh = 4;
bits_t new_dots[] = {
    0b00000000,
    0b00100000,
    0b00000000,
};

#endif // LED_MATRIX_SIM
#endif // OLED

// Audio constants
#if defined(AUDIO_DAC) || defined(AUDIO_CPU)
#define AUDIO_PIN A0
#define AUDIO_DUR 10
#define AUDIO_FREQ_A4 440.0
#define AUDIO_FREQ_E5 659.25
#define AUDIO_FREQ_A5 880.0
#ifdef ADAFRUIT_CIRCUITPLAYGROUND_M0
#define TONE_RESOLUTION 10
#else
#define TONE_RESOLUTION 8
#endif // ADAFRUIT_CIRCUITPLAYGROUND_M0
#define POWER_OF_TWO(x) (2 ^ x)
#ifdef AUDIO_DAC
#define AUDIO_ANALOG
#else
#define AUDIO_DIGITAL
#endif // AUDIO_DAC
//#define TEST_TONES
#else
#define AUDIO_I2S
#define I2S_LRC 26
#define I2S_DOUT 25
#define I2S_BCLK 27
#endif // AUDIO_DAC

// Audio variables and functions
volatile uint8_t counter = 0;
#if defined(AUDIO_DAC) || defined(AUDIO_CPU)
double audio_freq[] = {AUDIO_FREQ_A4, AUDIO_FREQ_E5, AUDIO_FREQ_A5};
unsigned long previous_tone_time = 0;
#endif // AUDIO_DAC or AUDIO_CPU
#ifdef AUDIO_I2S
Audio audio;
char *audio_files[] = {"/loo.mp3", "/loo_old.mp3" "/hii.mp3"};   
#endif // AUDIO_I2S
#ifdef RTOS
TaskHandle_t audio_task_handle = NULL;
TaskHandle_t audio_task_handle2 = NULL;
#endif // RTOS
#if defined(SAMD_TC4) || defined(NRF52_TIMER2)
volatile unsigned long interval = 1000000UL; // Default to 1 second
#endif // SAMD_TC4 or NRF52_TIMER2

// METRONOME SETTINGS
#define MAX_VOL 100
#define MIN_VOL 0

#define MAX_BEAT 8
#define MIN_BEAT 1

#define MAX_TEMPO 200
#define MIN_TEMPO 20
                
#define MAX_RHYTHM 8
#define MIN_RHYTHM 1

#define MAX_SWING 100
#define MIN_SWING 0

// Metronome settings
bool metronome_on = false;
uint8_t volume_val = 100;
uint8_t beat_val = 4;
uint8_t tempo_val = 120;
uint8_t rhy_val = 1;
uint8_t swing_val = 50;

// Menu constants
#define SELECT_BLINK_TIME 400UL
#define MODIFY_BLINK_TIME 100UL
enum selection // In menu order
{
#ifdef LED_MATRIX_SIM
#define SELECTION_COUNT 5
    SELECT_TEMPO,
    SELECT_SWING,
    SELECT_VOLUME,
    SELECT_RHYTHM,
    SELECT_BEAT
#else // !LED_MATRIX_SIM
#define SELECTION_COUNT 4
    SELECT_VOLUME,
    SELECT_BEAT,
    SELECT_TEMPO,
    SELECT_RHYTHM
#endif
};

// Menu variables
bool select_flag = false;
bool modify_flag = false;
uint8_t select_line = 0;
bool blink_flag = false;
unsigned long last_blink = 0;

#if defined(AUDIO_DAC) || defined(AUDIO_CPU)
// Manual tone generation to su
void volumeTone(uint32_t frequency, uint32_t duration, uint8_t volume)
{
    uint32_t halfPeriod = 1000000UL / frequency / 2;
    #ifdef AUDIO_ANALOG
    volume = map(volume, 0, 100, 0, POWER_OF_TWO(TONE_RESOLUTION));
    #endif // AUDIO_ANALOG

    for (uint32_t i = 0; i < frequency * duration / 1000; i++)
    {
        #ifdef AUDIO_ANALOG
        analogWrite(AUDIO_PIN, volume);
        #endif
        #ifdef AUDIO_DIGITAL
        digitalWrite(AUDIO_PIN, HIGH);
        #endif
        delayMicroseconds(halfPeriod);
        #ifdef AUDIO_ANALOG
        analogWrite(AUDIO_PIN, 0);
        #endif
        #ifdef AUDIO_DIGITAL
        digitalWrite(AUDIO_PIN, LOW);
        #endif
        delayMicroseconds(halfPeriod);
    }
}
#endif // AUDIO_DAC or AUDIO_CPU

#ifdef AUDIO_PWM
void volumeTone(uint32_t frequency, uint32_t duration, uint8_t volume)
{

}
#endif

void playAudio(int audio_index)
{
    #if defined(AUDIO_DAC) || defined(AUDIO_CPU)
    volumeTone(audio_freq[audio_index], AUDIO_DUR, volume_val);
    #endif // AUDIO_DAC or AUDIO_CPU
    previous_tone_time = micros();

    #ifdef AUDIO_I2S
    audio.connecttoFS(SPIFFS, audio_files[audio_index]);
    while (audio.isRunning()) 
    {
        audio.loop();
    }
    #endif // AUDIO_I2S
}

unsigned long calculateTiming()
{
    // Calculate period between metronome ticks based on tempo and rhythm division
    unsigned long period = tempo_val != 0 ? 
        60 * 1000000UL / (tempo_val * rhy_val) : 
        50UL;

    // Adjust period based on swing percentage if not 50%
    return swing_val == 50 ? 
        period : 
        period * (counter % 2 ?
            150 - swing_val : 
            swing_val + 50) 
            / 100.0;
}

void audioTask()
{
    #ifdef RTOS
    TickType_t xLastWakeTime, xFrequency;
    xLastWakeTime = xTaskGetTickCount();
    for (;;)
    #endif // RTOS
    {
        #ifdef RTOS
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        #endif // RTOS
        #if defined(SAMD_TC4) || defined(NRF52_TIMER2)
        unsigned long timing = calculateTiming();
        #endif // SAMD_TC4 or NRF52_TIMER2
        #ifdef SAMD_TC4
        if(timing != interval) setTC4(timing);
        #else
        #ifdef NRF52_TIMER2
        if(timing != interval) setTIMER2(timing);
        #endif // NRF52_TIMER2
        #endif // SAMD_TC4

        // Play audio
        #if !defined(NRF52_TIMER2) && !defined(SAMD_TC4)
        if(metronome_on)
        #endif // NRF52_TIMER2 && SAMD_TC4
        {
            #ifdef AUDIO_I2S
            audio.setVolume(volume_val);
            #endif

            #if defined(AUDIO_I2S) || defined(AUDIO_DAC) 
            if(
                #ifdef AUDIO_I2S
                audio.isRunning() &&
                #endif // AUDIO_I2S
                #ifdef AUDIO_DAC
                // Check to ensure DAC is done generating audio
                micros() - previous_tone_time >= AUDIO_DUR * 1000
                #endif // AUDIO_DAC
            ) 
            #endif // AUDIO_I2S || AUDIO_DAC
            {
                // Increment and wrap counter
                counter ++;
                counter %= beat_val * rhy_val;

                // Play audio if volume is not 0
                // First beat of measure is sound 2, other beats are sound 1, and subdivisions are sound 0
                if(volume_val)
                    playAudio(
                        counter == 0 ? 2
                        : counter % rhy_val == 0 ? 1
                        : 0);
            }
        }
    }
}

#ifdef SAMD_TC4
void configureTC4()
{
    // Enable shared generic clock for TC4 and TC5
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |
                        GCLK_CLKCTRL_GEN_GCLK0 |
                        GCLK_CLKCTRL_ID_TC4_TC5;
    while (GCLK->STATUS.bit.SYNCBUSY);
    
    // Reset timer/counter 4
    TC4->COUNT32.CTRLA.reg = TC_CTRLA_SWRST;
    // Wait for syncronization
    while (TC4->COUNT32.STATUS.bit.SYNCBUSY);
    while (TC4->COUNT32.CTRLA.bit.SWRST);

    // Set 32 bit counter and match frequency modes, disable prescaler division, and enables timer counter to start
    TC4->COUNT32.CTRLA.reg |= TC_CTRLA_MODE_COUNT32 |
                              TC_CTRLA_WAVEGEN_MFRQ | TC_CTRLA_PRESCALER_DIV1 | 
                              TC_CTRLA_ENABLE;
    while (TC4->COUNT32.STATUS.bit.SYNCBUSY);

    // Set timer compare capture register
    setTC4(interval);

    // Enable the TC4 interrupt request
    TC4->COUNT32.INTENSET.bit.MC0 = 1;
    // Enable TC4 interrupt
    NVIC_EnableIRQ(TC4_IRQn);
}

void setTC4(unsigned long new_interval) 
{
    interval = new_interval;
    TC4->COUNT32.CC[0].reg = 48UL * interval;
    while (TC4->COUNT32.STATUS.bit.SYNCBUSY);
}

inline void toggleTC4(bool enable)
{
    TC4->COUNT32.CTRLA.bit.ENABLE = enable;
    while(TC4->COUNT32.STATUS.bit.SYNCBUSY);
}

inline void triggerTC4()
{
    TC4->COUNT32.COUNT.reg = TC4->COUNT32.CC[0].reg; 
    while(TC4->COUNT32.STATUS.bit.SYNCBUSY);
}

void TC4_Handler()
{
    // Clear the interrupt flag
    TC4->COUNT32.INTFLAG.bit.MC0 = 1;  
    audioTask();
}
#endif // SAMD_TC4

#ifdef NRF52_TIMER2
void configureTIMER2()
{
    // Set bit size of TIMER2 to 32 bits
    NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    // Disable prescaler for full clock resolution
    NRF_TIMER2->PRESCALER = 0;
    setTIMER2(interval);
    // Enable interrupts on compare event 0
    NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
    // Clear counter
    NRF_TIMER2->SHORTS = 1UL;
    NVIC_EnableIRQ(TIMER2_IRQn);
    toggleTIMER2(metronome_on);
}

void setTIMER2(uint32_t new_interval)
{
    interval = new_interval;
    NRF_TIMER2->CC[0] = 16UL * new_interval;
}

inline void toggleTIMER2(bool enable)
{
    if(enable)
        NRF_TIMER2->TASKS_START = 1; 
    else
        NRF_TIMER2->TASKS_STOP = 1;
}

inline void triggerTIMER2()
{
    // Run counter capture task to capture compare register 1
    NRF_TIMER2->TASKS_CAPTURE[1] = 1;

    // Set capture compare register 0 to trigger compare event on next clock cycle
    NRF_TIMER2->CC[0] = NRF_TIMER2->CC[1] + 10;

    // Change interval to trigger setTIMER2 on interrupt
    interval = 0;
}

extern "C" void TIMER2_IRQHandler()
{
    if(NRF_TIMER2->EVENTS_COMPARE[0])
    {
        audioTask();
        NRF_TIMER2->EVENTS_COMPARE[0] = 0;
    }
}
#endif

#ifdef ENABLE_HFCLK
inline void calibrateLFCLK()
{
    NRF_CLOCK->CTIV = LFCLK_CALIBRATION_TIME;
    NRF_CLOCK->TASKS_CAL = 1;
    NRF_CLOCK->TASKS_CTSTART = 1;
    while(!NRF_CLOCK->EVENTS_DONE);
    NRF_CLOCK->EVENTS_DONE = 0;
}

inline void startHFCLK()
{
    if(!(NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_SRC_Msk))
    {
        NRF_CLOCK->TASKS_HFCLKSTART = 1;
        while(!NRF_CLOCK->HFCLKRUN);
    }
}

inline void stopHFCLK()
{
    if(NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_SRC_Msk)
        NRF_CLOCK->TASKS_HFCLKSTOP = 1;
}

inline void startLFCLK()
{
    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    while(!NRF_CLOCK->LFCLKRUN);
}
#endif // ENABLE_HFCLK

#ifdef USB_SUSPEND_KEEP_HFCLK
#define RESTART_TRIES 12 // Multiple tries needed for unstable VBUS voltage while unplugging USB causing multiple suspend events
volatile uint8_t hfclk_restart;
extern "C" void tud_suspend_cb(bool remote_wakeup_en)
{
    startHFCLK();
    if(metronome_on)
        hfclk_restart += RESTART_TRIES;
}
#endif // USB_SUSPEND_KEEP_HFCLK

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    #ifdef SETUP_LED
    digitalWrite(LED_BUILTIN, HIGH);
    #endif

    #ifdef SERIAL_DEBUG
    Serial.begin(BAUD_RATE);
    #ifdef SERIAL_DEBUG_WAIT_INIT
    while(!Serial);
    #endif // SERIAL_DEBUG_WAIT_INIT
    Serial.print(F("Soundial Firmware v"));
    Serial.println(VERSION);
    #endif // SERIAL_DEBUG

    #ifdef USB_DISABLE
    NRF_USBD->ENABLE = 0;
    #endif // USB_DISABLE

    #ifdef ENABLE_HFCLK
    startHFCLK();
    calibrateLFCLK();
    #endif // ENABLE_HFCLK

    #ifdef AUDIO_I2S
    // Start SPIFFS
    if(!SPIFFS.begin(true))
    {
        #ifdef SERIAL_DEBUG
        Serial.println(F("SPIFFS Mount Failed"));
        #endif // SERIAL_DEBUG
        for (;;);
    }
    #endif // AUDIO_I2S

    #ifdef OLED
    #ifdef SSD1306
    #ifdef OLED_SPI
    while (!display.begin(SSD1306_SWITCHCAPVCC)) 
    #endif // OLED_SPI
    #ifdef OLED_I2C
    while (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR))
    #endif // OLED_I2C
    {
        #ifdef SERIAL_DEBUG
        Serial.println(F("SSD1306 allocation failed"));
        delay(1000);
        #endif // SERIAL_DEBUG
    }
    #endif // SSD1306
    delay(OLED_INIT_DELAY);
    display.clearDisplay();
    display.print(F("Soundial"));
    display.setCursor(0, 8);
    display.print(F("Firmware v"));
    display.print(VERSION);
    display.setCursor(0, 16);
    display.print(F("Starting..."));
    display.display();
    #endif // OLED

    // NeoPixel
    #ifdef NEOPIXEL
    pixels.begin();
    pixels.fill();
    pixels.setBrightness(20);
    pixels.show();
    #ifdef SEIZURE_PREVENTION_TESTING
    pinMode(SEIZURE_PREVENTION_TESTING, OUTPUT);
    #endif // SEIZURE_PREVENTION_TESTING
    #endif // NEOPIXEL

    // Audio
    #if defined(AUDIO_DAC) || defined(AUDIO_CPU)
    #ifdef AUDIO_ANALOG
    analogWriteResolution(TONE_RESOLUTION);  // Set analog write resolution to 10 bits for DAC
    #endif // AUDIO_ANALOG
    pinMode(AUDIO_PIN, OUTPUT);
    #if defined(AUDIO_CPU)
    digitalWrite(AUDIO_PIN, LOW); // Prevent low hum
    #endif
    //REG_DAC_CTRLA |= DAC_CTRLA_ENABLE;  // Enable DAC
    //pinPeripheral(AUDIO_PIN, PIO_ANALOG) // Set A1 as DAC0 output
    #endif // AUDIO_DAC or AUDIO_CPU
    #ifdef AUDIO_I2S
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT, -1);
    audio.setVolumeSteps(170);
    audio.setVolume(volume_val);
    #endif // AUDIO_I2S

    // Test tones
    #ifdef TEST_TONES
    tone(AUDIO_PIN, AUDIO_FREQ_A4, 1000);
    delay(1100);
    volumeTone(AUDIO_FREQ_A4, 1000, volume_val);
    delay(1100);
    #endif // TEST_TONES
    
    #ifdef RTOS
    // Create an audio task on the second core
    xTaskCreatePinnedToCore(audioTask, "AudioTask", 5000, NULL, 1, &audio_task_handle, 1);
    #endif // RTOS

    #ifdef SAMD_TC4
    configureTC4();
    #endif // SAMD_TC4

    #ifdef NRF52_TIMER2
    configureTIMER2();
    #endif // NRF52_TIMER2

    #ifdef TWO_BUTTONS
    pinMode(BUTTON_A, INPUT_PULLDOWN);
    pinMode(BUTTON_B, INPUT_PULLDOWN);
    #endif // TWO_BUTTONS

    #ifdef SPDT_SWITCH
    pinMode(SWITCH_A, INPUT_PULLUP);
    #endif // SPDT_SWITCH

    #ifdef ROTARY_ENCODER
    rotary.setDebounceDelay(5);
    #endif // ROTARY_ENCODER

    #ifdef VCC_AND_TEMP_DEBUG
    initSAADC();
    initTEMP();
    #endif // VCC_AND_TEMP_DEBUG

    #ifdef SETUP_LED
    digitalWrite(LED_BUILTIN, LOW);
    #endif
}

// Adjusts counter for rhythm changes
void adjustCounter(uint8_t previous_rhythm)
{
    // Adjust the subdivision based on the ratio of new rhythm to old rhythm
    uint8_t adjusted_subdivision = counter % previous_rhythm * rhy_val / previous_rhythm;
    
    // Adjust the counter based on the current beat and adjusted subdivision
    counter = counter / previous_rhythm * rhy_val + adjusted_subdivision;
    
    // Wrap counter to valid range
    counter %= (beat_val * rhy_val);
}

#ifdef LED_MATRIX_SIM
void modifySetting(uint8_t& setting, uint8_t min, uint8_t max)
{
    switch(turn)
    {
        case CW:
            if(setting < max)
                setting ++;
            break;
        case CCW:
            if(setting > min)
                setting --;
            break;
    }
}

void modifySettings()
{
    switch(select_line)
    {
        case SELECT_VOLUME:
            modifySetting(volume_val, MIN_VOL, MAX_VOL);
            break;
        case SELECT_BEAT:
            modifySetting(beat_val, MIN_BEAT, MAX_BEAT);
            break;
        case SELECT_TEMPO:
            modifySetting(tempo_val, MIN_TEMPO, MAX_TEMPO);
            break;
        case SELECT_RHYTHM:
        {
            uint8_t previous_rhythm = rhy_val;
            modifySetting(rhy_val, MIN_RHYTHM, MAX_RHYTHM);
            if(previous_rhythm != rhy_val)
                adjustCounter(previous_rhythm);
            break;
        }
        case SELECT_SWING:
            modifySetting(swing_val, MIN_SWING, MAX_SWING);
            break;
    }
}

#else 

void displayModeMenu(uint8_t cursor, uint8_t rotary_val, byte p)
{
    switch (cursor)
    {
        case SELECT_VOLUME:
            switch (rotary_val)
            {
                case CW:
                    if(volume_val < MAX_VOL)
                        volume_val++;
                    break;
                case CCW:
                    if(volume_val > MIN_VOL)
                        volume_val--;
                    break;
            }
            display.clearDisplay();
            display.setTextSize(2);
            display.setCursor(28, 0);
            display.print(F("VOLUME"));
            display.drawPixel(64, 0, SSD1306_WHITE);
            display.setTextSize(6);
            if(volume_val < 100)
                display.setCursor(46, 20);
            else
                display.setCursor(10, 20);
            display.print(volume_val);
            display.display();
            break;
        case SELECT_BEAT:
            switch (rotary_val)
            {
                case CW:
                    if(beat_val < MAX_BEAT)
                        beat_val++;
                    break;
                case CCW:
                    if(beat_val > MIN_BEAT)
                        beat_val--;
                    break;
            }
            display.clearDisplay();
            display.setTextSize(2);
            display.setCursor(25, 0);
            display.print(F("BEATS"));
            display.setTextSize(6);
            display.setCursor(45, 20);
            display.print(beat_val);
            display.display();
            break;
        case SELECT_TEMPO:
            switch (rotary_val)
            {
                case CW:
                    if(tempo_val < MAX_TEMPO)
                        tempo_val++;
                    break;
                case CCW:
                    if(tempo_val > MIN_TEMPO)
                        tempo_val--;
                    break;
            }
            display.clearDisplay();
            display.setTextSize(2);
            display.setCursor(25, 0);
            display.print(F("SELECT_TEMPO"));
            display.setTextSize(6);
            display.setCursor(20, 20);
            display.print(tempo_val);
            display.display();
            break;
        case SELECT_RHYTHM:
            uint8_t previous_rhythm = rhy_val;
            switch (rotary_val)
            {
                case CW:
                    if(rhy_val < MAX_RHYTHM)
                        rhy_val++;
                    break;
                case CCW:
                    if(rhy_val > MIN_RHYTHM)
                        rhy_val--;
                    break;
            }
            if(previous_rhythm != rhy_val)
                adjustCounter(previous_rhythm);
            display.clearDisplay();
            display.setTextSize(2);
            display.setCursor(25, 0);
            display.print(F("RHYTHM"));
            display.setTextSize(6);
            display.setCursor(45, 20);
            display.print(rhy_val << 2);
            display.display();
            break;
    }
}
#endif // LED_MATRIX_SIM

void readInput()
{
    #ifdef TWO_BUTTONS
    turn = 0;
    pressed = false;
    long_pressed = false;
    static unsigned long last_press_times[2];
    static uint8_t press_counts;
    static uint8_t buttons_held;
    static uint8_t pending_single_presses;
    unsigned long now = millis();
    uint8_t button_state = digitalRead(BUTTON_A) << 1 | digitalRead(BUTTON_B);
    
    for(uint8_t i = 0; i < 2; i++)
    {
        uint8_t nibble = 4 * i; // Current nibble
        uint8_t mask1 = 1 << i; // Single bit mask for the current button
        uint8_t mask4 = 0xF << nibble; // Nibble mask for current button

        // Reset last press time if time overflowed
        if(last_press_times[i] > now)
            last_press_times[i] = now;

        if(button_state & mask1)
        {
            if(!(buttons_held & mask1))
            {
                buttons_held |= mask1;
                last_press_times[i] = now;
                press_counts = (press_counts & ~mask4) | ((press_counts & mask4) + (1 << nibble)); // Increment press count for current button
                pending_single_presses &= ~mask1; // Reset pending press
            }
            else if(now - last_press_times[i] > BUTTON_READ_PERIOD)
                turn = i + 1;
        }
        else
        {
            if(buttons_held & mask1)
            {
                buttons_held &= ~mask1;
                uint8_t press_count = (press_counts & mask4) >> nibble;
                if(press_count == 1)
                {
                    pending_single_presses |= mask1; // Set pending single press
                    last_press_times[i] = now;
                }
                else if(press_count == 2 && now - last_press_times[i] < BUTTON_DOUBLE_PRESS_PERIOD)
                {
                    if(!i)
                        pressed = true;
                    else
                        long_pressed = true;
                    press_counts &= ~mask4; // Set count to zero
                    pending_single_presses &= ~mask1; // Reset pending press
                }
                else
                {
                    press_counts &= ~mask4; // Set count to zero
                }
            }
        }

        // Handle pending single press
        if(pending_single_presses & mask1 && (now - last_press_times[i]) > BUTTON_READ_PERIOD && (now - last_press_times[i]) > BUTTON_DOUBLE_PRESS_PERIOD)
        {
            turn = i + 1;
            press_counts &= ~mask4; // Set count to zero
            pending_single_presses &= ~mask1; // Reset pending press
        }

        // Reset press counts if no press within double press period
        if(now - last_press_times[i] > BUTTON_DOUBLE_PRESS_PERIOD)
            press_counts &= ~mask4;
    }
    
    #ifdef PRESSED_LED
    digitalWrite(LED_BUILTIN, pressed);
    #endif // PRESSED_LED
    #endif // TWO_BUTTONS

    #ifdef SPDT_SWITCH
    #if defined(NEOPIXEL_COLOR) && defined(NEOPIXEL_WHITE)
    neopixel_color_offset = digitalRead(SWITCH_A) ? 0 : NEOPIXEL_COLOR_COUNT;
    #endif // NEOPIXEL_COLOR && NEOPIXEL_WHITE
    #endif // SPDT_SWITCH

    #ifdef SERIAL_DEBUG
    #ifdef SERIAL_CONTROL
    // Check for incoming serial data 
    char command = 0;
    if(Serial.available() > 0)
        // Read the last byte sent
        command = Serial.read(); 

    // Handle control commands
    switch(command)
    {
        case SERIAL_CONTROL_PRESS:
            pressed = true;
            break;

        case SERIAL_CONTROL_LONG_PRESS:
            long_pressed = true;
            break;

        case SERIAL_CONTROL_TURN_CW:
            turn = CW;
            break;

        case SERIAL_CONTROL_TURN_CCW:
            turn = CCW;
            break;

        case SERIAL_CONTROL_DEBUG:
            break;    

        case SERIAL_CONTROL_ETC:
            ledmatsim.configDotBitmap(new_dots, new_dots_wh, new_dots_wh);
            updateDisplay();
            break;

        case SERIAL_CONTROL_SPECIAL:
            ledmatsim.configDotBitmap(
                new_dots,
                4,
                4
           );
            break;

        default:
        #ifndef TWO_BUTTONS
            pressed = 0;
            long_pressed = 0;
            turn = 0;
        #endif
            break;
    }
    #endif // SERIAL_CONTROL
    #endif // SERIAL_DEBUG

    #ifdef ROTARY_ENCODER
    pressed = rotary.push();
    long_pressed = rotary.pushLong(1000);
    turn = rotary.rotate();
    #endif // ROTARY_ENCODER

    #ifdef LED_MATRIX_SIM
    if(pressed) 
    {
        if(!select_flag)
            select_flag = true;
        else if(!modify_flag)
            modify_flag = true;
        else
        {
            modify_flag = false;
            select_flag = false;
        }
    }
    #else
    if(pressed) 
    {
      select_flag = !select_flag;
    }
    #endif // LED_MATRIX_SIM
    if(long_pressed) 
    {
        metronome_on = !metronome_on;

        #ifdef ENABLE_HFCLK
        if(metronome_on)
            startHFCLK();
        else if(!NRF_USBD->ENABLE)
        {    
            stopHFCLK();
            startLFCLK();
            calibrateLFCLK();
        }
        #endif

        #ifdef NRF52_TIMER2
        toggleTIMER2(metronome_on);
        #endif // NRF52_TIMER2
        #ifdef SAMD_TC4
        toggleTC4(metronome_on);
        #endif // SAMD_TC4
        
        if(metronome_on)
        {
            counter = (beat_val * rhy_val) - 1; // Reset counter to last count to start at zero
            
            #ifdef NRF52_TIMER2
                triggerTIMER2();
            #endif // NRF52_TIMER
            #ifdef SAMD_TC4
                triggerTC4();
            #endif // SAMD_TC4
        }
    }
    
    #ifdef LED_MATRIX_SIM
    if(modify_flag)
        modifySettings();
    else
    #endif // LED_MATRIX_SIM
    if(select_flag)
    #ifndef LED_MATRIX_SIM
        displayModeMenu(select_line, turn, pressed);
    else
    #endif // !LED_MATRIX_SIM
    {
      switch (turn)
{
        case CCW:
            select_line = (select_line - 1 + SELECTION_COUNT) % SELECTION_COUNT;
            break;
        case CW:
            select_line = (select_line + 1) % SELECTION_COUNT;
            break;
      }
      #ifndef LED_MATRIX_SIM
      updateDisplay();
      #endif // !LED_MATRIX_SIM
    }
}

#ifdef NEOPIXEL
void updateNeoPixels()
{    
    static uint32_t neopixel_colors[] = {
    #ifdef NEOPIXEL_COLOR
        pixels.Color(128, 0, 0),
        pixels.Color(0, 128, 0),
        pixels.Color(0, 0, 128),
        pixels.Color(128, 0, 128),
        pixels.Color(128, 128, 0),
        pixels.Color(0, 128, 128),
    #endif // NEOPIXEL_COLOR
    #ifdef NEOPIXEL_WHITE
        pixels.Color(255, 255, 255),
        pixels.Color(128, 128, 128),
        pixels.Color(64, 64, 64),
        pixels.Color(32, 32, 32),
        pixels.Color(16, 16, 16),
        pixels.Color(0, 0, 0)
    #endif // NEOPIXEL_WHITE
    };

    // Just clear NeoPixels if metronome is not on
    if(!metronome_on)
    {
        pixels.clear();
        pixels.show();
        return;
    }

    uint8_t current_beat = (counter / rhy_val) % beat_val;
    uint8_t current_subdivision = counter % rhy_val;

    #ifdef SEIZURE_PREVENTION
    // Keeps subdivision changes to under 10 Hz
    // Ticks per second approximated with division by 64 in place of 60 for bitshift optimization
    if(((tempo_val * rhy_val >> 6) >= 10) & (current_subdivision & 1))
        return;
    #ifdef SEIZURE_PREVENTION_TESTING
    static uint8_t counter_buffer;
    digitalWrite(SEIZURE_PREVENTION_TESTING, counter_buffer != counter);
    counter_buffer = counter;
    #endif // SEIZURE_PREVENTION_TESTING
    #endif // SEIZURE_PREVENTION

    uint8_t end_pixel = ((current_beat + 1) * NEOPIXEL_COUNT + beat_val - 1) / beat_val;
    uint8_t current_color =
        #if defined(NEOPIXEL_COLOR) && defined(NEOPIXEL_WHITE)
        neopixel_color_offset +
        #endif // NEOPIXEL_COLOR && NEOPIXEL_WHITE
        (counter == 0 ? 0 
        : current_subdivision == 0 ? 1 
        : (current_subdivision & 1) == 1 ? 2
        : current_subdivision == 2 ? 3
        : (current_subdivision & 5) == 5 ? 4
        : 5);

    pixels.clear();
    for (uint8_t i = current_beat * NEOPIXEL_COUNT / beat_val; i < end_pixel; i++)
        pixels.setPixelColor(
        #ifdef NEOPIXEL_ROTATE
        i >= NEOPIXEL_COUNT / 2
            ? i - NEOPIXEL_COUNT / 2 
            : i + NEOPIXEL_COUNT / 2
        #else
        i
        #endif // NEOPIXEL_ROTATE
        , neopixel_colors[current_color]);
    pixels.show();
}
#endif // NEOPIXEL

void updateDisplay() 
{
    display.setRotation(OLED_ROTATION);
    #ifdef LED_MATRIX_SIM
    // Blink
    if(select_flag)
    {
        unsigned long now = millis();
        if(now - last_blink > (modify_flag ? MODIFY_BLINK_TIME : SELECT_BLINK_TIME))
        {
            blink_flag = !blink_flag;
            last_blink = now; 
        }
    }

    // Tempo and swing time percentage characters
    ledmatsim.drawRect(TEMPO_DOTS_X, TEMPO_DOTS_Y, DEFAULT_FONT_WIDTH * 3 + 2, DEFAULT_FONT_HEIGHT, DOT_OFF);
    uint8_t tempo_or_swing = tempo_val;
    if(select_line == SELECT_SWING)
        tempo_or_swing = swing_val;
    if(!select_flag || blink_flag && (select_line == SELECT_TEMPO || select_line == SELECT_SWING) || (select_line != SELECT_TEMPO && select_line != SELECT_SWING))
    {
        ledmatsim.configFont(ledmatsim.DEFAULT_FONT, DEFAULT_FONT_WIDTH, DEFAULT_FONT_HEIGHT, DEFAULT_FONT_LENGTH);
        if(tempo_or_swing > 99)
            ledmatsim.drawChar(tempo_or_swing / 100, TEMPO_DOTS_X, TEMPO_DOTS_Y);
        if(tempo_or_swing > 9)
            ledmatsim.drawChar(tempo_or_swing % 100 / 10, TEMPO_DOTS_X + DEFAULT_FONT_WIDTH, TEMPO_DOTS_Y);
        ledmatsim.drawChar(tempo_or_swing % 10, TEMPO_DOTS_X + DEFAULT_FONT_WIDTH * 2, TEMPO_DOTS_Y);
    }

    // Time signature characters
    ledmatsim.configFont(ledmatsim.SMALL_FONT, SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT, SMALL_FONT_LENGTH);
    ledmatsim.drawRect(UPPER_TIME_SIGNATURE_X, UPPER_TIME_SIGNATURE_Y, SMALL_FONT_WIDTH * 2, SMALL_FONT_HEIGHT * 2 + 1, DOT_OFF);
    if(!select_flag || select_line != SELECT_BEAT || blink_flag && select_line == SELECT_BEAT)
    {
        uint8_t bars = beat_val * rhy_val;
        if(bars < 10)
            ledmatsim.drawChar(bars, UPPER_TIME_SIGNATURE_X + SMALL_FONT_WIDTH / 2, UPPER_TIME_SIGNATURE_Y);
        else
        {
            ledmatsim.drawChar(bars / 10, UPPER_TIME_SIGNATURE_X, UPPER_TIME_SIGNATURE_Y);
            ledmatsim.drawChar(bars % 10, UPPER_TIME_SIGNATURE_X + SMALL_FONT_WIDTH, UPPER_TIME_SIGNATURE_Y);
        }
    }
    if(!select_flag || select_line != SELECT_RHYTHM || blink_flag && select_line == SELECT_RHYTHM)
    {
        if((rhy_val << 2) < 10)
            ledmatsim.drawChar(rhy_val << 2, LOWER_TIME_SIGNATURE_X + SMALL_FONT_WIDTH / 2, LOWER_TIME_SIGNATURE_Y);
        else
        {
            ledmatsim.drawChar((rhy_val << 2) / 10, LOWER_TIME_SIGNATURE_X, LOWER_TIME_SIGNATURE_Y);
            ledmatsim.drawChar((rhy_val << 2) % 10, LOWER_TIME_SIGNATURE_X + SMALL_FONT_WIDTH, LOWER_TIME_SIGNATURE_Y);
        }
    }
    
    // Volume indicator bar
    if(!select_flag || select_line != SELECT_VOLUME || blink_flag && select_line == SELECT_VOLUME)
    {
        for(uint8_t i = 0; i < VOLUME_DOTS_LENGTH; i++)
            ledmatsim.setPixel(VOLUME_DOTS_START_X + i, VOLUME_DOTS_Y,
                i < map(volume_val, MIN_VOL, MAX_VOL, 0, VOLUME_DOTS_LENGTH) || (i == 0 && volume_val));
    }
    else
    {
        ledmatsim.drawRect(VOLUME_DOTS_START_X, VOLUME_DOTS_Y, VOLUME_DOTS_LENGTH, 1, DOT_OFF);    
    }

    // Update LED matrix simulation buffer without updating display
    ledmatsim.update(false);
    
    // Overlay icon bitmaps
    if(select_line != SELECT_SWING)
        display.drawXBitmap(BPM_BITMAP_X, BPM_BITMAP_Y, bpm_bitmap_bits, bpm_bitmap_width, bpm_bitmap_height, WHITE);
    else
        display.drawXBitmap(SWG_BITMAP_X, SWG_BITMAP_Y, swg_bitmap_bits, swg_bitmap_width, swg_bitmap_height, WHITE);
    display.drawXBitmap(VOLUME_BITMAP_X, VOLUME_BITMAP_Y, volume_bitmap_bits, volume_bitmap_width, volume_bitmap_height, WHITE);
    display.display();
    
    #else // !LED_MATRIX_SIM
    
    display.clearDisplay();
  
    int lineStartY = select_line * 16;  // Assuming each line is 16 pixels high
      display.drawRect(0, lineStartY, OLED_WIDTH, 16, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);

    // Display the headings
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print(F("VOLUME"));
    display.setCursor(80, 0);
    display.print(volume_val);

    display.setCursor(0, 16);
    display.print(F("BEATS"));
    display.setCursor(80, 16);
    display.print(beat_val);

    display.setCursor(0, 32);
    display.print(F("SELECT_TEMPO"));
    display.setCursor(80, 32);
    display.print(tempo_val);

    display.setCursor(0, 48);
    display.print(F("RHYTHM"));
    display.setCursor(80, 48);
    display.print(F("1/"));
    display.print(rhy_val << 2);
    
    display.display();
    #endif // !LED_MATRIX_SIM
}

void loop()
{
    #ifdef USB_DISABLE
    NRF_USBD->ENABLE = 0;
    #endif // USB_DISABLE

    #ifdef ENABLE_HFCLK
    if(hfclk_restart)
    {
        startHFCLK();
        hfclk_restart --;
    }
    #endif

    #ifdef FORCE_HFCLK
    startHFCLK();
    #endif // FORCE_HFCLK

    #ifdef HFCLK_SRC_LED
    digitalWrite(LED_BUILTIN, NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_SRC_Msk);
    #endif // HFCLK_SRC_LED

    readInput();

    #ifdef NEOPIXEL
    updateNeoPixels();
    #endif // NEOPIXEL
    
    #ifdef LED_MATRIX_SIM
    updateDisplay();
    #endif // LED_MATRIX_SIM
}

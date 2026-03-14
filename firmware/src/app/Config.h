/*
    Store fixed compile-time constants here:
        - pin assignments 
        - voltage limits 
        - motor constants
        - shunt / amplifier constants 
        - SPI pins 
        - ADC reference voltage 
        - default gains 
*/
#define PIN_UL 4
#define PIN_UH 5
#define PIN_VL 6
#define PIN_VH 7
#define PIN_WL 15
#define PIN_WH 16
#define PIN_EN 17

// Current Sense Pins
#define PIN_I_A 8
#define PIN_I_B 9
#define PIN_I_C 10

// SPI Pins
#define PIN_SPI_MISO 42
#define PIN_SPI_MOSI -1 // not used
#define PIN_SPI_CLK 41

// Sensor Chip Selects
#define PIN_ENC_CS 40

// Setup values
constexpr float VOLTAGE_SUPPLY = 5.0f;
constexpr float VOLTAGE_LIMIT = 3.0f;
constexpr float CURRENT_LIMIT = 0.8f;

// Current Sense Constants
const float SHUNT_RESISTOR = 0.0107f;
const float AMP_GAIN = 50.0f;

constexpr int POLE_PAIRS = 7;

constexpr float R_FEEL = 5.0f;
constexpr float IQ_MAX = 2.0f;
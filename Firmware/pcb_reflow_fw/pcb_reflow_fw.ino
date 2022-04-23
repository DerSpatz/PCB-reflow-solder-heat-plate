/* Solder Reflow Plate Sketch
 *  H/W - Ver 2.4
 *  S/W - Ver 1.0
 *  by Chris Halsall and Nathan Heidt     */

/* To prepare
 * 1) Install MiniCore in additional boards; (copy into File->Preferences->Additional Boards Manager
 * URLs) https://mcudude.github.io/MiniCore/package_MCUdude_MiniCore_index.json 2) Then add MiniCore
 * by searching and installing (Tools->Board->Board Manager) 3) Install Adafruit_GFX and
 * Adafruit_SSD1306 libraries (Tools->Manage Libraries)
 */

/* To program
 *  1) Select the following settings under (Tools)
 *      Board->Minicore->Atmega328
 *      Clock->Internal 8MHz
 *      BOD->BOD 2.7V
 *      EEPROM->EEPROM retained
 *      Compiler LTO->LTO Disabled
 *      Variant->328P / 328PA
 *      Bootloader->No bootloader
 *  2) Set programmer of choice, e.g.'Arduino as ISP (MiniCore)', 'USB ASP', etc, and set correct
 * port. 3) Burn bootloader (to set fuses correctly) 4) Compile and upload
 *
 *
 * TODOS:
 * - add digital sensor setup routine if they are detected, but not setup
 * - figure out a method for how to use all the temperature sensors
 * - implement an observer/predictor for the temperature sensors.  Kalman filter time?!?
 */

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <SPI.h>

// Version Definitions
static const PROGMEM float hw = 2.4;
static const PROGMEM float sw = 2.0;

// Screen Definitions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define SCREEN_ADDRESS 0x3C                                       // I2C Address
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1); // Create Display

// Pin Definitions
#define MOSFET_PIN PIN_PC3
#define UPSW_PIN PIN_PF3
#define DNSW_PIN PIN_PD4
#define TEMP_PIN PIN_PF2 // A2
#define VCC_PIN PIN_PF4  // A0
#define LED_GREEN_PIN PIN_PC4
#define LED_RED_PIN PIN_PC5
#define ONE_WIRE_BUS PIN_PE1

#define MOSFET_PIN_OFF 255

enum menu_state_t { MENU_IDLE, MENU_SELECT_PROFILE, MENU_HEAT, MENU_INC_TEMP, MENU_DEC_TEMP };
enum buttons_state_t { BUTTONS_NO_PRESS, BUTTONS_BOTH_PRESS, BUTTONS_UP_PRESS, BUTTONS_DN_PRESS };
enum single_button_state_t { BUTTON_PRESSED, BUTTON_RELEASED, BUTTON_NO_ACTION };

// Button interrupt state
volatile single_button_state_t up_button_state = BUTTON_NO_ACTION;
volatile single_button_state_t dn_button_state = BUTTON_NO_ACTION;
volatile unsigned long up_state_change_time = 0;
volatile unsigned long down_state_change_time = 0;

// Temperature Info
byte max_temp_array[] = {140, 150, 160, 170, 180};
byte max_temp_index = 0;
#define MAX_RESISTANCE 10.0
float bed_resistance = 0.9;
#define MAX_AMPERAGE 5.0
#define PWM_VOLTAGE_SCALAR 1.0

// These values were derived using a regression from real world data.
// See the jupyter notebooks for more detail
#define ANALOG_APPROXIMATION_SCALAR 1.752
#define ANALOG_APPROXIMATION_OFFSET -20.517

// EEPROM storage locations
#define CRC_ADDR 0
#define FIRSTTIME_BOOT_ADDR 4
#define TEMP_INDEX_ADDR 5
#define RESISTANCE_INDEX_ADDR 6
#define DIGITAL_TEMP_ID_ADDR 10

// Voltage Measurement Info
#define VOLTAGE_REFERENCE 1.5


// Solder Reflow Plate Logo
static const uint8_t PROGMEM logo[] = {
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x1f, 0xc0, 0x00, 0x31, 0x80, 0x00, 0x00, 0x00, 0x00,
    0x1f, 0xe0, 0x03, 0x01, 0x80, 0x00, 0x00, 0x30, 0x70, 0x00, 0x21, 0x80, 0x00, 0x00, 0x00, 0x00,
    0x10, 0x20, 0x03, 0x00, 0xc7, 0x80, 0x00, 0x20, 0x18, 0xf0, 0x61, 0x80, 0x00, 0x00, 0x00, 0x00,
    0x18, 0x00, 0x03, 0x3e, 0xcc, 0xc0, 0xc0, 0x04, 0x19, 0x98, 0x61, 0x80, 0x00, 0x00, 0x00, 0x00,
    0x1c, 0x01, 0xf3, 0x77, 0xd8, 0xc7, 0xe0, 0x06, 0x33, 0x18, 0x61, 0x8f, 0x88, 0x00, 0x00, 0x00,
    0x06, 0x03, 0x3b, 0x61, 0xd0, 0xc6, 0x00, 0x07, 0xe2, 0x18, 0x61, 0x98, 0xd8, 0x04, 0x00, 0x00,
    0x01, 0xc6, 0x0b, 0x60, 0xd9, 0x86, 0x00, 0x06, 0x03, 0x30, 0xff, 0xb0, 0x78, 0x66, 0x00, 0x00,
    0x40, 0xe4, 0x0f, 0x60, 0xdf, 0x06, 0x00, 0x07, 0x03, 0xe0, 0x31, 0xe0, 0x78, 0x62, 0x00, 0x00,
    0x40, 0x3c, 0x0f, 0x61, 0xd8, 0x06, 0x00, 0x07, 0x83, 0x00, 0x31, 0xe0, 0x78, 0x63, 0x00, 0x00,
    0x60, 0x36, 0x1b, 0x63, 0xc8, 0x02, 0x00, 0x02, 0xc1, 0x00, 0x18, 0xb0, 0xcc, 0xe2, 0x00, 0x00,
    0x30, 0x33, 0x3b, 0x36, 0x4e, 0x03, 0x00, 0x02, 0x61, 0xc0, 0x0c, 0x99, 0xcd, 0xfe, 0x00, 0x00,
    0x0f, 0xe1, 0xe1, 0x3c, 0x03, 0xf3, 0x00, 0x02, 0x38, 0x7e, 0x0c, 0x8f, 0x07, 0x9c, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x7f, 0x84, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xc0, 0xe4, 0x00, 0x18, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x04, 0x3c, 0x3c, 0x18, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x04, 0x1e, 0x06, 0x7f, 0xc6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x04, 0x3e, 0x03, 0x18, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x04, 0x36, 0x7f, 0x19, 0x8c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x07, 0xe6, 0xc7, 0x19, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x06, 0x07, 0x83, 0x18, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x06, 0x07, 0x81, 0x18, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x06, 0x06, 0xc3, 0x98, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x02, 0x04, 0x7e, 0x08, 0x3f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t logo_width = 128;
static const uint8_t logo_height = 27;

// Heating Animation
static const uint8_t PROGMEM heat_animate[] = {
    0b00000001, 0b00000000, 0b00000001, 0b10000000, 0b00000001, 0b10000000, 0b00000001, 0b01000000,
    0b00000010, 0b01000000, 0b00100010, 0b01000100, 0b00100100, 0b00100100, 0b01010101, 0b00100110,
    0b01001001, 0b10010110, 0b10000010, 0b10001001, 0b10100100, 0b01000001, 0b10011000, 0b01010010,
    0b01000100, 0b01100010, 0b00100011, 0b10000100, 0b00011000, 0b00011000, 0b00000111, 0b11100000};
static const uint8_t heat_animate_width = 16;
static const uint8_t heat_animate_height = 16;

// Tick
static const uint8_t PROGMEM tick[] = {
    0b00000000, 0b00000100, 0b00000000, 0b00001010, 0b00000000, 0b00010101, 0b00000000, 0b00101010,
    0b00000000, 0b01010100, 0b00000000, 0b10101000, 0b00000001, 0b01010000, 0b00100010, 0b10100000,
    0b01010101, 0b01000000, 0b10101010, 0b10000000, 0b01010101, 0b00000000, 0b00101010, 0b00000000,
    0b00010100, 0b00000000, 0b00001000, 0b00000000, 0b01111111, 0b11100000};
static const uint8_t tick_width = 16;
static const uint8_t tick_height = 15;

// This needs to be specified or the compiler will fail as you can't initialize a
// flexible array member in a nested context
#define MAX_PROFILE_LENGTH 8

// TODO(HEIDT) may need to switch away from floats for speed/sizeA
struct solder_profile_t {
    uint8_t points;
    float seconds[MAX_PROFILE_LENGTH];
    float fraction[MAX_PROFILE_LENGTH];
};

// TODO(HEIDT) how to adjust for environments where the board starts hot or cold?
// profiles pulled from here: https://www.7pcb.com/blog/lead-free-reflow-profile.php
#define NUM_PROFILES 2
const static solder_profile_t profiles[NUM_PROFILES] = {
    {.points = 4, .seconds = {25, 105, 115, 155}, .fraction = {.65, .78, .81, 1.00}},
    {.points = 2, .seconds = {162.0, 202.0}, .fraction = {.95, 1.00}}};

// temperature must be within this range to move on to next step
#define TARGET_TEMP_THRESHOLD 5.0

// PID values
float kI = 0.2;
float kD = 0.25;
float kP = 8.0;
float I_clip = 150;
float error_I = 0;

// Optional temperature sensor
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
int sensor_count = 0;
DeviceAddress temp_addresses[3];

#define DEBUG

#ifdef DEBUG
#define debugprint(x) Serial.print(x);
#define debugprintln(x) Serial.println(x);
#else
#define debugprint(x)
#define debugprintln(x)
#endif

// -------------------- Function prototypes -----------------------------------
void inline heatAnimate(int &x, int &y, float v, float t, float target_temp);

// -------------------- Function definitions ----------------------------------

void dnsw_change_isr() {
    dn_button_state = BUTTON_PRESSED;
    down_state_change_time = millis();
}

void upsw_change_isr() {
    up_button_state = BUTTON_PRESSED;
    up_state_change_time = millis();
}

void setup() {

    // Pin Direction control
    pinMode(MOSFET_PIN, OUTPUT);
    pinMode(UPSW_PIN, INPUT);
    pinMode(DNSW_PIN, INPUT);
    pinMode(TEMP_PIN, INPUT);
    pinMode(VCC_PIN, INPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);

    digitalWrite(LED_GREEN_PIN, HIGH);
    analogWrite(MOSFET_PIN, 255); // VERY IMPORTANT, DONT CHANGE!

    attachInterrupt(DNSW_PIN, dnsw_change_isr, FALLING);
    attachInterrupt(UPSW_PIN, upsw_change_isr, FALLING);

    Serial.begin(9600);

    // Enable Fast PWM with no prescaler
    setFastPwm();
    setVREF();

    // Start-up Diplay
    debugprintln("Showing startup");
    showLogo();

    debugprintln("Checking sensors");
    // check onewire TEMP_PIN sensors
    setupSensors();

    debugprintln("Checking first boot");
    if (isFirstBoot() || !validateCRC()) {
        doSetup();
    }

    // Pull saved values from EEPROM
    max_temp_index = getMaxTempIndex();
    bed_resistance = getResistance();

    debugprintln("Entering main menu");
    // Go to main menu
    mainMenu();
}

void updateCRC() {
    uint32_t new_crc = eepromCRC();
    setCRC(new_crc);
}

bool validateCRC() {
    uint32_t stored_crc;
    EEPROM.get(CRC_ADDR, stored_crc);
    uint32_t calculated_crc = eepromCRC();
    debugprint("got CRCs, stored: ");
    debugprint(stored_crc);
    debugprint(", calculated: ");
    debugprintln(calculated_crc);
    return stored_crc == calculated_crc;
}

void setCRC(uint32_t new_crc) { EEPROM.put(CRC_ADDR, new_crc); }

uint32_t eepromCRC(void) {
    static const uint32_t crc_table[16] = {0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
                                           0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
                                           0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
                                           0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c};
    uint32_t crc = ~0L;
    // Skip first 4 bytes of EEPROM as thats where we store the CRC
    for (int index = 4; index < EEPROM.length(); ++index) {
        crc = crc_table[(crc ^ EEPROM[index]) & 0x0f] ^ (crc >> 4);
        crc = crc_table[(crc ^ (EEPROM[index] >> 4)) & 0x0f] ^ (crc >> 4);
        crc = ~crc;
    }

    return crc;
}

inline void setupSensors() {
    sensors.begin();
    sensor_count = sensors.getDeviceCount();
    debugprint("Looking for sensors, found: ");
    debugprintln(sensor_count);
    for (int i = 0; i < min(sensor_count, sizeof(temp_addresses)); i++) {
        sensors.getAddress(temp_addresses[i], i);
    }
}

inline void setFastPwm() { analogWriteFrequency(64); }

inline void setVREF() { analogReference(INTERNAL1V5); }

inline bool isFirstBoot() {
    uint8_t first_boot = EEPROM.read(FIRSTTIME_BOOT_ADDR);
    debugprint("Got first boot flag: ");
    debugprintln(first_boot);
    return first_boot != 1;
}

inline void setFirstBoot() {
    EEPROM.write(FIRSTTIME_BOOT_ADDR, 1);
    updateCRC();
}

inline float getResistance() {
    float f;
    return EEPROM.get(RESISTANCE_INDEX_ADDR, f);
    return f;
}

inline void setResistance(float resistance) {
    EEPROM.put(RESISTANCE_INDEX_ADDR, resistance);
    updateCRC();
}

inline void setMaxTempIndex(int index) {
    EEPROM.update(TEMP_INDEX_ADDR, index);
    updateCRC();
}

inline int getMaxTempIndex(void) { return EEPROM.read(TEMP_INDEX_ADDR) % sizeof(max_temp_array); }

void showLogo() {
    unsigned long start_time = millis();
    display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
    while (start_time + 2000 > millis()) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.drawBitmap(0, 0, logo, logo_width, logo_height, SSD1306_WHITE);
        display.setCursor(80, 16);
        display.print(F("S/W V"));
        display.print(sw, 1);
        display.setCursor(80, 24);
        display.print(F("H/W V"));
        display.print(hw, 1);
        display.display();
        buttons_state_t cur_button = getButtonsState();
        // If we press both buttons during boot, we'll enter the setup process
        if (cur_button == BUTTONS_BOTH_PRESS) {
            doSetup();
            return;
        }
    }
}

inline void doSetup() {
    debugprintln("Performing setup");
    // TODO(HEIDT) show an info screen if we're doing firstime setup or if memory is corrupted

    getResistanceFromUser();
    // TODO(HEIDT) do a temperature module setup here

    setFirstBoot();
}

inline void getResistanceFromUser() {
    float resistance = 0.9;
    while (1) {
        clearMainMenu();
        display.setCursor(3, 4);
        display.print(F("Resistance"));
        display.drawLine(3, 12, 79, 12, SSD1306_WHITE);
        display.setCursor(3, 14);
        display.print(F("UP/DN: change"));
        display.setCursor(3, 22);
        display.print(F("BOTH: choose"));
        buttons_state_t button = getButtonsState();
        if (button == BUTTONS_UP_PRESS) {
            resistance += 0.1;
        } else if (button == BUTTONS_DN_PRESS) {
            resistance -= 0.1;
        } else if (button == BUTTONS_BOTH_PRESS) {
            setResistance(resistance);
            return;
        }
        resistance = constrain(resistance, 0, MAX_RESISTANCE);

        display.setCursor(90, 12);
        display.print(resistance);
        display.display();
    }
}

inline void mainMenu() {
    // Debounce
    menu_state_t cur_state = MENU_IDLE;

    int x = 0;   // Display change counter
    int y = 200; // Display change max (modulused below)
    uint8_t profile_index = 0;

    while (1) {
        switch (cur_state) {
        case MENU_IDLE: {
            clearMainMenu();
            buttons_state_t cur_button = getButtonsState();

            if (cur_button == BUTTONS_BOTH_PRESS) {
                cur_state = MENU_SELECT_PROFILE;
            } else if (cur_button == BUTTONS_UP_PRESS) {
                cur_state = MENU_INC_TEMP;
            } else if (cur_button == BUTTONS_DN_PRESS) {
                cur_state = MENU_DEC_TEMP;
            }
        } break;
        case MENU_SELECT_PROFILE: {
            debugprintln("getting thermal profile");
            profile_index = getProfile();
            cur_state = MENU_HEAT;
        } break;
        case MENU_HEAT: {
            if (!heat(max_temp_array[max_temp_index], profile_index)) {
                cancelledPB();
                coolDown();
            } else {
                coolDown();
                completed();
            }
            cur_state = MENU_IDLE;
        } break;
        case MENU_INC_TEMP: {
            if (max_temp_index < sizeof(max_temp_array) - 1) {
                max_temp_index++;
                debugprintln("incrementing max temp");
                setMaxTempIndex(max_temp_index);
            }
            cur_state = MENU_IDLE;
        } break;
        case MENU_DEC_TEMP: {
            if (max_temp_index > 0) {
                max_temp_index--;
                debugprintln("decrementing max temp");
                setMaxTempIndex(max_temp_index);
            }
            cur_state = MENU_IDLE;
        } break;
        }

        // Change Display (left-side)
        showMainMenuLeft(x, y);

        // Update Display (right-side)
        showMainMenuRight();
    }
}

#define BUTTON_PRESS_TIME 50
buttons_state_t getButtonsState() {
    single_button_state_t button_dn;
    single_button_state_t button_up;
    unsigned long button_dn_time;
    unsigned long button_up_time;

    noInterrupts();
    button_dn = dn_button_state;
    button_up = up_button_state;
    button_dn_time = down_state_change_time;
    button_up_time = up_state_change_time;
    interrupts();

    unsigned long cur_time = millis();
    buttons_state_t state = BUTTONS_NO_PRESS;

    if (button_dn == BUTTON_PRESSED && button_up == BUTTON_PRESSED &&
        abs(button_dn_time - button_up_time) < BUTTON_PRESS_TIME) {
        if (cur_time - button_dn_time > BUTTON_PRESS_TIME &&
            cur_time - button_up_time > BUTTON_PRESS_TIME) {
            state = BUTTONS_BOTH_PRESS;
            noInterrupts();
            dn_button_state = BUTTON_NO_ACTION;
            up_button_state = BUTTON_NO_ACTION;
            interrupts();
        }
    } else if (button_up == BUTTON_PRESSED && cur_time - button_up_time > BUTTON_PRESS_TIME) {
        state = BUTTONS_UP_PRESS;
        noInterrupts();
        up_button_state = BUTTON_NO_ACTION;
        interrupts();
    } else if (button_dn == BUTTON_PRESSED && cur_time - button_dn_time > BUTTON_PRESS_TIME) {
        state = BUTTONS_DN_PRESS;
        noInterrupts();
        dn_button_state = BUTTON_NO_ACTION;
        interrupts();
    }

    return state;
}

inline uint8_t getProfile() {
    uint8_t cur_profile = 0;
    while (1) {
        clearMainMenu();
        display.setCursor(3, 4);
        display.print(F("Pick profile"));
        display.drawLine(3, 12, 79, 12, SSD1306_WHITE);
        display.setCursor(3, 14);
        display.print(F(" UP/DN: cycle"));
        display.setCursor(3, 22);
        display.print(F(" BOTH: choose"));
        buttons_state_t cur_button = getButtonsState();
        if (cur_button == BUTTONS_BOTH_PRESS) {
            clearMainMenu();
            return cur_profile;
        } else if (cur_button == BUTTONS_DN_PRESS) {
            cur_profile--;
        } else if (cur_button == BUTTONS_UP_PRESS) {
            cur_profile++;
        }
        cur_profile %= NUM_PROFILES;
        displayProfileRight(cur_profile);
        display.display();
    }
}

inline void displayProfileRight(int8_t cur_profile) {
    int cur_x = 90;
    int cur_y = 30;
    // start at x=90, go to SCREEN_WIDTH-8, save 6 pixels for cooldown
    float x_dist = SCREEN_WIDTH - 90 - 8;
    display.setCursor(cur_x, cur_y);
    float total_seconds = (int)profiles[cur_profile].seconds[profiles[cur_profile].points - 1];

    for (int i = 0; i < profiles[cur_profile].points; i++) {
        int x_next = (int)((profiles[cur_profile].seconds[i] / total_seconds) * x_dist) + 90;
        int y_next = 30 - (int)(profiles[cur_profile].fraction[i] * 28.0);
        display.drawLine(cur_x, cur_y, x_next, y_next, SSD1306_WHITE);
        cur_x = x_next;
        cur_y = y_next;
    }
    // draw down to finish TEMP_PIN
    display.drawLine(cur_x, cur_y, SCREEN_WIDTH - 2, 30, SSD1306_WHITE);
}

inline void clearMainMenu() {
    display.clearDisplay();
    display.setTextSize(1);
    display.drawRoundRect(0, 0, 83, 32, 2, SSD1306_WHITE);
}

inline void showMainMenuLeft(int &x, int &y) {
    if (x < (y * 0.5)) {
        display.setCursor(3, 4);
        display.print(F("PRESS BUTTONS"));
        display.drawLine(3, 12, 79, 12, SSD1306_WHITE);
        display.setCursor(3, 14);
        display.print(F(" Change  MAX"));
        display.setCursor(3, 22);
        display.print(F(" Temperature"));
    } else {
        display.setCursor(3, 4);
        display.print(F("HOLD  BUTTONS"));
        display.drawLine(3, 12, 79, 12, SSD1306_WHITE);
        display.setCursor(3, 18);
        display.print(F("Begin Heating"));
    }
    x = (x + 1) % y; // Display change increment and modulus
}

inline void showMainMenuRight() {
    display.setCursor(95, 6);
    display.print(F("TEMP"));
    display.setCursor(95, 18);
    display.print(max_temp_array[max_temp_index]);
    display.print(F("C"));
    display.display();
}

inline void showHeatMenu(byte max_temp) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(22, 4);
    display.print(F("HEATING"));
    display.setTextSize(1);
    display.setCursor(52, 24);
    display.print(max_temp);
    display.print(F("C"));
    display.display();
}

bool heat(byte max_temp, int profile_index) {
    // Heating Display
    showHeatMenu(max_temp);
    delay(3000);

    float t; // Used to store current temperature
    float v; // Used to store current voltage

    unsigned long profile_max_time = millis() / 1000 + (8 * 60);
    unsigned long step_start_time = (millis() / 1000);
    int current_step = 0;

    // Other control variables
    int x = 0;  // Heat Animate Counter
    int y = 80; // Heat Animate max (modulused below)

    float start_temp = getTemp();
    float goal_temp = profiles[profile_index].fraction[0] * max_temp;
    float step_runtime = profiles[profile_index].seconds[0];
    float last_time = 0;
    float last_temp = getTemp();
    error_I = 0;

    while (1) {
        // Cancel heat, don't even wait for uppress so we don't risk missing it during the loop
        if (getButtonsState() != BUTTONS_NO_PRESS) {
            analogWrite(MOSFET_PIN, MOSFET_PIN_OFF);
            debugprintln("cancelled");
            return 0;
        }

        // Check Heating not taken more than 8 minutes
        if (millis() / 1000 > profile_max_time) {
            analogWrite(MOSFET_PIN, MOSFET_PIN_OFF);
            debugprintln("exceeded time");
            cancelledTimer();
            return 0;
        }

        // Measure Values
        // TODO(HEIDT) getting the temperature from the digital sensors is by far the slowest part
        // of this loop. figure out an approach that allows control faster than sensing
        t = getTemp();
        v = getVolts();
        float max_possible_amperage = v / bed_resistance;
        // TODO(HEIDT) approximate true resistance based on cold resistance and temperature
        float vmax = (MAX_AMPERAGE * bed_resistance) * PWM_VOLTAGE_SCALAR;
        int min_PWM = 255 - ((vmax * 255.0) / v);
        min_PWM = constrain(min_PWM, 0, 255);
        debugprint("Min PWM: ");
        debugprintln(min_PWM);
        debugprintln(bed_resistance);

        // Determine what target temp is and PID to it
        float time_into_step = ((float)millis() / 1000.0) - (float)step_start_time;
        float target_temp = min(
            ((goal_temp - start_temp) * (time_into_step / step_runtime)) + start_temp, goal_temp);

        // TODO(HEIDT) PID for a ramp will always lag, other options may be better
        stepPID(target_temp, t, last_temp, time_into_step - last_time, min_PWM);
        last_time = time_into_step;

        // if we finish the step timewise
        if (time_into_step >= step_runtime) {
            // and if we're within the goal temperature of the step
            if (abs(t - goal_temp) < TARGET_TEMP_THRESHOLD) {
                // move onto the next step in the profile
                current_step++;
                // if that was the last step, we're done!
                if (current_step == profiles[profile_index].points) {
                    analogWrite(MOSFET_PIN, MOSFET_PIN_OFF);
                    return 1;
                }
                // otherwise, get the next goal temperature and runtime, and do the process again
                last_time = 0.0;
                start_temp = t;
                goal_temp = profiles[profile_index].fraction[current_step] * max_temp;
                step_runtime = profiles[profile_index].fraction[current_step] -
                               profiles[profile_index].fraction[current_step - 1];
                step_start_time = millis() / 1000.0;
            }
        }

        heatAnimate(x, y, v, t, target_temp);
    }
}

void evaluate_heat() {
    debugprintln("Starting thermal evaluation");
    uint8_t duties[] = {255, 225, 200, 150, 100, 50, 0};
    unsigned long runtime = 60*5;  // run each for 5 minutes

    for(int i = 0; i < sizeof(duties); i++) {
        debugprint("Running to duty of: ");
        debugprintln(duties[i]);
        unsigned long start_time = millis();
        analogWrite(MOSFET_PIN, duties[i]);
        float elapsed_time = (millis() - start_time)/1000.0;
        while(elapsed_time < runtime) {
            debugprint("elapsed time: ");
            debugprintln(elapsed_time);
            debugprint("runtime: ");
            debugprintln(runtime);
            elapsed_time = (millis() - start_time)/1000.0;
            float v = getVolts();
            float t = getTemp();
            delay(500);
        }
    }
    
    analogWrite(MOSFET_PIN, MOSFET_PIN_OFF);
}

void stepPID(float target_temp, float current_temp, float last_temp, float dt, int min_pwm) {
    float error = target_temp - current_temp;
    float D = (current_temp - last_temp) / dt;

    error_I += error * dt * kI;
    error_I = constrain(error_I, -I_clip, I_clip);

    // PWM is inverted so 0 duty is 100% power
    float PWM = 255.0 - (error * kP + D * kD + error_I);
    PWM = constrain(PWM, min_pwm, 255);

    debugprintln("PID");
    debugprintln(dt);
    debugprintln(error);
    debugprintln(PWM);
    debugprintln(error_I);
    analogWrite(MOSFET_PIN, (int)PWM);
}

void inline heatAnimate(int &x, int &y, float v, float t, float target) {
    // Heat Animate Control
    display.clearDisplay();
    display.drawBitmap(0, 3, heat_animate, heat_animate_width, heat_animate_height, SSD1306_WHITE);
    display.drawBitmap(112, 3, heat_animate, heat_animate_width, heat_animate_height,
                       SSD1306_WHITE);
    display.fillRect(0, 3, heat_animate_width, heat_animate_height * (y - x) / y, SSD1306_BLACK);
    display.fillRect(112, 3, heat_animate_width, heat_animate_height * (y - x) / y, SSD1306_BLACK);
    x = (x + 1) % y; // Heat animate increment and modulus

    // Update display
    display.setTextSize(2);
    display.setCursor(22, 4);
    display.print(F("HEATING"));
    display.setTextSize(1);
    display.setCursor(20, 24);
    display.print(F("~"));
    display.print(v, 1);
    display.print(F("V"));
    if (t >= 100) {
        display.setCursor(63, 24);
    } else if (t >= 10) {
        display.setCursor(66, 24);
    } else {
        display.setCursor(69, 24);
    }
    display.print(F("~"));
    display.print(t, 0);
    display.print(F("C"));
    display.print(F("/"));
    display.print(target, 0);
    display.print(F("C"));
    display.display();
}

void cancelledPB() { // Cancelled via push button
    // Update Display
    display.clearDisplay();
    display.drawRoundRect(22, 0, 84, 32, 2, SSD1306_WHITE);
    display.setCursor(25, 4);
    display.print(F("  CANCELLED"));
    display.display();
    delay(2000);
}

void cancelledTimer() { // Cancelled via 5 minute Time Limit
    // Initiate Swap Display
    int x = 0;   // Display change counter
    int y = 150; // Display change max (modulused below)

    // Wait to return on any button press
    while (getButtonsState() == BUTTONS_NO_PRESS) {
        // Update Display
        display.clearDisplay();
        display.drawRoundRect(22, 0, 84, 32, 2, SSD1306_WHITE);
        display.setCursor(25, 4);
        display.print(F("  TIMED OUT"));
        display.drawLine(25, 12, 103, 12, SSD1306_WHITE);

        // Swap Main Text
        if (x < (y * 0.3)) {
            display.setCursor(25, 14);
            display.println(" Took longer");
            display.setCursor(25, 22);
            display.println(" than 5 mins");
        } else if (x < (y * 0.6)) {
            display.setCursor(28, 14);
            display.println("Try a higher");
            display.setCursor(25, 22);
            display.println(" current PSU");
        } else {
            display.setCursor(25, 14);
            display.println(" Push button");
            display.setCursor(25, 22);
            display.println("  to return");
        }
        x = (x + 1) % y; // Display change increment and modulus

        display.setTextSize(3);
        display.setCursor(5, 4);
        display.print(F("!"));
        display.setTextSize(3);
        display.setCursor(108, 4);
        display.print(F("!"));
        display.setTextSize(1);
        display.display();
        delay(50);
    }
}

void coolDown() {
    float t = getTemp(); // Used to store current temperature

    // Wait to return on any button press, or TEMP_PIN below threshold
    while (getButtonsState() == BUTTONS_NO_PRESS && t > 45.00) {
        display.clearDisplay();
        display.drawRoundRect(22, 0, 84, 32, 2, SSD1306_WHITE);
        display.setCursor(25, 4);
        display.print(F("  COOL DOWN"));
        display.drawLine(25, 12, 103, 12, SSD1306_WHITE);
        display.setCursor(25, 14);
        display.println("  Still Hot");
        t = getTemp();
        if (t >= 100) {
            display.setCursor(49, 22);
        } else {
            display.setCursor(52, 22);
        }
        display.print(F("~"));
        display.print(t, 0);
        display.print(F("C"));
        display.setTextSize(3);
        display.setCursor(5, 4);
        display.print(F("!"));
        display.setTextSize(3);
        display.setCursor(108, 4);
        display.print(F("!"));
        display.setTextSize(1);
        display.display();
    }
}

void completed() {
    // Update Display
    display.clearDisplay();
    display.drawRoundRect(22, 0, 84, 32, 2, SSD1306_WHITE);
    display.setCursor(25, 4);
    display.print(F("  COMPLETED  "));
    display.drawLine(25, 12, 103, 12, SSD1306_WHITE);
    display.setCursor(25, 14);
    display.println(" Push button");
    display.setCursor(25, 22);
    display.println("  to return");
    display.drawBitmap(0, 9, tick, tick_width, tick_height, SSD1306_WHITE);
    display.drawBitmap(112, 9, tick, tick_width, tick_height, SSD1306_WHITE);
    display.display();

    // Wait to return on any button press
    while (getButtonsState() == BUTTONS_NO_PRESS) {
    }
}

float getTemp() {
    debugprint("Temps: ");
    float t = 0;
    for (byte i = 0; i < 100; i++) { // Poll TEMP_PIN reading 100 times
        t = t + analogRead(TEMP_PIN);
    }
    t /= 100.0;        // average
    t *= VOLTAGE_REFERENCE / 1024.0; // voltage
    // conversion to temp, consult datasheet:
    // https://www.ti.com/document-viewer/LMT85/datasheet/detailed-description#snis1681040
    // this is optimized for 25C to 150C
    // TODO(HEIDT) this is linearized and innacurate, could probably use the nonlinear
    // functions without much overhead.
    t = (t - 1.365) / ((.301 - 1.365) / (150.0 - 25.0)) + 25.0;

    // The analog sensor is too far from the bed for an accurate reading
    // this simple function estimates the true bed temperature based off the thermal
    // gradient 
    float estimated_temp = t*ANALOG_APPROXIMATION_SCALAR + ANALOG_APPROXIMATION_OFFSET;
    debugprint(estimated_temp);
    debugprint(" ");

    sensors.requestTemperatures();
    for (int i = 0; i < sensor_count; i++) {
        float temp_in = sensors.getTempC(temp_addresses[i]);
        debugprint(temp_in);
        debugprint(" ");
    }
    debugprintln();


    return max(t, estimated_temp);
}

float getVolts() {
    float v = 0;
    for (byte i = 0; i < 20; i++) { // Poll Voltage reading 20 times
        v = v + analogRead(VCC_PIN);
    }
    v /= 20;

    float vin = (v / 1023.0) * 1.5;
    debugprint("voltage at term: ");
    debugprintln(vin);
    vin = (vin / 0.090981) + 0.3;
    return vin;
}

void loop() {
    // Not used
}

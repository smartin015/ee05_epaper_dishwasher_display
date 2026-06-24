#include <Arduino.h>
#include "TFT_eSPI.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "esp_mac.h"           // esp_base_mac_addr_set
#include "nvs_flash.h"

// ---------------------------------------------------------------------------
// BLE UUIDs — ESPHome ble_client writes to this characteristic
// ---------------------------------------------------------------------------
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// ---------------------------------------------------------------------------
// Power management — deep sleep
// ---------------------------------------------------------------------------
#define DEEP_SLEEP_SEC      60      // Wake interval when state is stable
#define BLE_LISTEN_MS       15000   // How long to advertise/listen for BLE writes
#define DEBUG_STAY_MS       60000   // Stay awake after button press (for debug)

// Button GPIOs (also used as deep-sleep wake sources)
#define BUTTON_KEY0  2
#define BUTTON_KEY1  3
#define BUTTON_KEY2  5
#define BUTTON_MASK  ((1ULL << BUTTON_KEY0) | (1ULL << BUTTON_KEY1) | (1ULL << BUTTON_KEY2))

// Battery ADC
#define BATTERY_ADC  A0
#define ADC_EN       6
#define VOLTAGE_DIVIDER_RATIO  2.0f

// ---------------------------------------------------------------------------
// Display state
// ---------------------------------------------------------------------------
enum DisplayState {
    STATE_CLEAN,
    STATE_DIRTY,
    STATE_RUNNING,
    STATE_UNKNOWN
};

// ---- RTC memory: survives deep sleep --------------------------------------
RTC_DATA_ATTR DisplayState rtcState       = STATE_UNKNOWN;
RTC_DATA_ATTR uint32_t      rtcBootCount  = 0;
RTC_DATA_ATTR uint32_t      rtcStateChanges = 0;

// ---- volatile globals (lost across deep sleep) ----------------------------
volatile DisplayState pendingState  = STATE_UNKNOWN;
volatile bool          stateChanged = false;
static bool            bleInitialised = false;

#ifdef EPAPER_ENABLE
EPaper epaper;
#endif

// ---------------------------------------------------------------------------
// BLE callback
// ---------------------------------------------------------------------------
class StateCallback : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        String val = pCharacteristic->getValue();
        val.trim();
        Serial.printf("BLE write: \"%s\"\n", val.c_str());

        DisplayState next = STATE_UNKNOWN;
        if (val == "dirty" || val == "DIRTY")
            next = STATE_DIRTY;
        else if (val == "clean" || val == "CLEAN")
            next = STATE_CLEAN;
        else if (val == "running" || val == "RUNNING")
            next = STATE_RUNNING;
        else {
            Serial.println("  -> unknown state, ignoring");
            return;
        }
        pendingState  = next;
        stateChanged  = true;
    }
};

// ---------------------------------------------------------------------------
// Initialise BLE peripheral (called fresh on every wake)
// ---------------------------------------------------------------------------
static void bleInit() {
    if (bleInitialised) return;

    // Force the public (factory) MAC so the address is stable across
    // deep-sleep reboots.  Without this, the BLE stack may use a random
    // address that changes every wake — breaking ble_client auto_connect.
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    esp_base_mac_addr_set(mac);

    BLEDevice::init("EE05-Status");
    BLEServer *pServer = BLEDevice::createServer();

    BLEService *pService = pServer->createService(SERVICE_UUID);
    BLECharacteristic *pChar = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE);

    pChar->setCallbacks(new StateCallback());
    pChar->setValue("clean");
    pService->start();

    BLEAdvertising *pAdv = BLEDevice::getAdvertising();
    pAdv->addServiceUUID(SERVICE_UUID);
    pAdv->setScanResponse(true);
    BLEDevice::startAdvertising();

    bleInitialised = true;

    // Print both the base MAC we set AND the BLE stack's actual address.
    // They can differ — the BLE address is what the bridge sees on air.
    Serial.printf("BLE advertising as 'EE05-Status'\n");
    Serial.printf("  Base MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.printf("  BLE addr: %s\n", BLEDevice::getAddress().toString().c_str());
}

// ---------------------------------------------------------------------------
// Draw one of the three status screens
// ---------------------------------------------------------------------------
static void drawState(DisplayState state) {
    const char *text;
    uint32_t    color;

    switch (state) {
    case STATE_DIRTY:   text = "DIRTY";   color = TFT_RED;    break;
    case STATE_CLEAN:   text = "CLEAN";   color = TFT_BLACK;  break;
    case STATE_RUNNING: text = "RUNNING"; color = TFT_YELLOW; break;
    default:            text = "...";      color = TFT_BLACK;  break;
    }

    int32_t w = epaper.width();
    int32_t h = epaper.height();

    epaper.fillScreen(TFT_WHITE);
    epaper.fillRect(0, 0, w, 10, color);
    epaper.fillRect(w - 8, 10, 8, h - 10, color);

    epaper.setTextColor(color, TFT_WHITE);
    epaper.setTextFont(4);
    epaper.setTextDatum(MC_DATUM);
    epaper.setTextPadding(w);
    epaper.drawString(text, w / 2, h / 2 - 8);

    epaper.fillCircle(w / 2, h - 26, 9, color);
    epaper.fillCircle(w / 2, h - 26, 5, TFT_WHITE);

    epaper.setTextFont(2);
    epaper.setTextColor(TFT_BLACK, TFT_WHITE);
    epaper.setTextDatum(BC_DATUM);
    epaper.drawString("EE05  ·  BLE", w / 2, h - 4);

    Serial.printf("Drawing '%s' ...\n", text);
    epaper.update();
    Serial.println("  display updated.");
}

// ---------------------------------------------------------------------------
// Read battery voltage
// ---------------------------------------------------------------------------
static float readBattery() {
    int sum = 0;
    pinMode(ADC_EN, OUTPUT);
    digitalWrite(ADC_EN, HIGH);
    delay(5);
    for (int i = 0; i < 10; i++) {
        sum += analogRead(BATTERY_ADC);
        delay(2);
    }
    digitalWrite(ADC_EN, LOW);
    return (sum / 10.0f / 4095.0f) * 3.3f * VOLTAGE_DIVIDER_RATIO;
}

// ---------------------------------------------------------------------------
// Enter deep sleep
// ---------------------------------------------------------------------------
static void enterDeepSleep(uint64_t sleepSec, bool buttonWake) {
    Serial.printf("→ Deep sleep for %llu s", sleepSec);
    if (buttonWake) Serial.print(" (buttons enabled)");
    Serial.println("...");
    Serial.flush();

    // Put ePaper panel to sleep
#ifdef EPAPER_ENABLE
    epaper.sleep();
#endif

    delay(50);

    // Timer wake-up
    esp_sleep_enable_timer_wakeup(sleepSec * 1000000ULL);

    // Button wake-up (EXT1: wake if ANY button is pressed)
    if (buttonWake) {
        // Configure buttons as RTC GPIOs with internal pull-ups
        rtc_gpio_pulldown_dis((gpio_num_t)BUTTON_KEY0);
        rtc_gpio_pullup_en((gpio_num_t)BUTTON_KEY0);
        rtc_gpio_pulldown_dis((gpio_num_t)BUTTON_KEY1);
        rtc_gpio_pullup_en((gpio_num_t)BUTTON_KEY1);
        rtc_gpio_pulldown_dis((gpio_num_t)BUTTON_KEY2);
        rtc_gpio_pullup_en((gpio_num_t)BUTTON_KEY2);

        // EXT1: wake when ANY button goes LOW (active-low buttons)
        esp_sleep_enable_ext1_wakeup(BUTTON_MASK, ESP_EXT1_WAKEUP_ANY_LOW);
    }

    esp_deep_sleep_start();
    // ── never reaches here ──
}

// ---------------------------------------------------------------------------
// Setup — runs on cold boot AND after every deep-sleep wake
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(300);

    rtcBootCount++;
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    bool isColdBoot = (cause == ESP_SLEEP_WAKEUP_UNDEFINED);

    Serial.println();
    Serial.println("═══════════════════════════════════════════");
    Serial.println("  EE05 4-Color ePaper Status Display");
    Serial.printf ("  Boot #%u  |  State changes: %u\n",
                   rtcBootCount, rtcStateChanges);
    Serial.print  ("  Wake cause: ");
    switch (cause) {
    case ESP_SLEEP_WAKEUP_UNDEFINED: Serial.println("cold boot / reset");   break;
    case ESP_SLEEP_WAKEUP_TIMER:     Serial.println("timer");               break;
    case ESP_SLEEP_WAKEUP_EXT1:      Serial.println("button");              break;
    default:                         Serial.printf("other (%d)\n", cause);   break;
    }
    Serial.println("═══════════════════════════════════════════");

    // ---- cold boot: initialise display, draw default state ----------------
#ifdef EPAPER_ENABLE
    epaper.begin();
    epaper.setRotation(1);
    Serial.printf("Display: %d x %d\n", epaper.width(), epaper.height());

    if (isColdBoot) {
        drawState(STATE_CLEAN);
        rtcState = STATE_CLEAN;
        Serial.printf("Battery: %.2f V\n", readBattery());
    }
#else
    Serial.println("EPAPER_ENABLE not defined — display disabled.");
#endif

    // ---- button wake: show debug screen, stay awake longer -----------------
    if (cause == ESP_SLEEP_WAKEUP_EXT1) {
        float vbatt = readBattery();
        Serial.printf("Button wake — battery: %.2f V, boot #%u, changes: %u\n",
                      vbatt, rtcBootCount, rtcStateChanges);

#ifdef EPAPER_ENABLE
        // Draw a compact debug overlay
        epaper.fillScreen(TFT_WHITE);
        epaper.setTextColor(TFT_BLACK, TFT_WHITE);
        epaper.setTextFont(2);
        epaper.setTextDatum(TL_DATUM);
        epaper.drawString("DEBUG", 10, 10);

        char buf[64];
        snprintf(buf, sizeof(buf), "Boot: #%u  Changes: %u", rtcBootCount, rtcStateChanges);
        epaper.drawString(buf, 10, 35);
        snprintf(buf, sizeof(buf), "Battery: %.2f V", vbatt);
        epaper.drawString(buf, 10, 55);
        epaper.update();
#endif

        // Init BLE and stay awake for DEBUG_STAY_MS
        bleInit();
        unsigned long debugEnd = millis() + DEBUG_STAY_MS;
        while (millis() < debugEnd) {
            if (stateChanged) {
                stateChanged = false;
                if (pendingState != rtcState) {
                    rtcState = pendingState;
                    rtcStateChanges++;
#ifdef EPAPER_ENABLE
                    drawState(rtcState);
#endif
                }
            }
            delay(100);
        }
        enterDeepSleep(DEEP_SLEEP_SEC, true);
    }

    // ---- timer wake (or cold boot): BLE listen window ---------------------
    bleInit();

    unsigned long listenEnd = millis() + BLE_LISTEN_MS;
    Serial.printf("Listening for BLE writes (%d s)...\n", BLE_LISTEN_MS / 1000);

    while (millis() < listenEnd) {
        if (stateChanged) {
            stateChanged = false;
            if (pendingState != rtcState) {
                rtcState = pendingState;
                rtcStateChanges++;
#ifdef EPAPER_ENABLE
                // Display init may already be done; begin() is idempotent
                if (!isColdBoot) {
                    epaper.begin();
                    epaper.setRotation(1);
                }
                drawState(rtcState);
#endif
            } else {
                Serial.println("State unchanged — skipping refresh.");
            }
            // After a state change, extend the listen window a bit
            // to catch follow-up writes, then sleep.
            listenEnd = millis() + 2000;
        }
        delay(100);
    }

    Serial.println("No state change during listen window.");
    enterDeepSleep(DEEP_SLEEP_SEC, true);
}

// ---------------------------------------------------------------------------
// Loop — never reached; deep sleep reboots on wake
// ---------------------------------------------------------------------------
void loop() {
    // After deep sleep, ESP32 restarts at setup().
    // This is here only to satisfy the Arduino framework.
    delay(1000);
}

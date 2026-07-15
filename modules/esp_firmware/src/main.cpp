#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include "Steering.h"
#include "Throttle.h"
#include "Brake.h"

// ============================================================
// Donanım Pinleri (esp32_pinout.txt'den)
// ============================================================
constexpr uint8_t PIN_STEER_ENA = 15;
constexpr uint8_t PIN_STEER_PUL = 16;
constexpr uint8_t PIN_STEER_DIR = 17;

constexpr uint8_t PIN_THROTTLE_SDA = 8;
constexpr uint8_t PIN_THROTTLE_SCL = 18;
constexpr uint8_t PIN_RELAY_BRAKE = 0;
constexpr uint8_t PIN_RELAY_LOW = 36;
constexpr uint8_t PIN_RELAY_REVERSE = 35;

constexpr uint8_t PIN_BRAKE_DIR = 12;
constexpr uint8_t PIN_BRAKE_PWM = 11;

// ============================================================
// Modüller
// ============================================================
Steering steering(PIN_STEER_ENA, PIN_STEER_PUL, PIN_STEER_DIR);
Throttle throttle(PIN_THROTTLE_SDA, PIN_THROTTLE_SCL, PIN_RELAY_BRAKE, PIN_RELAY_LOW, PIN_RELAY_REVERSE);
Brake brake(PIN_BRAKE_DIR, PIN_BRAKE_PWM);

// Xbox Bluetooth Modülü
XboxSeriesXControllerESP32_asukiaaa::Core xboxController;

// ============================================================
// Durum Değişkenleri
// ============================================================
bool isArmed = false;
bool isDeadmanActive = false;
bool wasConnected = false;
enum Gear { NEUTRAL, DRIVE, REVERSE };
Gear currentGear = NEUTRAL;

uint32_t armStartedAt = 0;
constexpr uint32_t ARM_HOLD_MS = 1500;

constexpr float MAX_STEER_ANGLE = 450.0f;
constexpr float STEER_DEADZONE = 0.10f;
constexpr float TRIGGER_DEADZONE = 0.02f;

float applyDeadzone(float value, float deadzone) {
    if (abs(value) <= deadzone) return 0.0f;
    float sign = (value > 0.0f) ? 1.0f : -1.0f;
    return sign * ((abs(value) - deadzone) / max(1.0f - deadzone, 1e-6f));
}

// ============================================================
// Kurulum
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=========================================");
    Serial.println("  ALAZ BLUETOOTH XBOX KONTROLCÜ BASLATILIYOR  ");
    Serial.println("=========================================");

    steering.begin();
    throttle.begin();
    brake.begin();

    // Bluetooth Xbox Başlat
    xboxController.begin();
    Serial.println("BLUETOOTH AKTIF! Lutfen Xbox kolunu 'Eslestirme' moduna (Hizli Yanip Sonme) alin.");
}

// ============================================================
// Ana Döngü
// ============================================================
void loop() {
    xboxController.onLoop();

    // Bağlantı durumu değiştiğinde mesaj ver
    if (xboxController.isConnected() && !wasConnected) {
        Serial.println("\n>>> HARIKA! KABLOSUZ XBOX BAGLANDI! <<<");
        wasConnected = true;
    } else if (!xboxController.isConnected() && wasConnected) {
        Serial.println("\n>>> ACIL DURUM: KABLOSUZ XBOX BAGLANTISI KOPTU!");
        wasConnected = false;
        isArmed = false;
        throttle.setVoltage(Throttle::IDLE_V);
        brake.setMotor(0, 0);
        steering.stop();
    }

    if (xboxController.isConnected() && !xboxController.isWaitingForFirstNotification()) {
        
        bool btnA      = xboxController.xboxNotif.btnA;
        bool btnSelect = xboxController.xboxNotif.btnSelect;
        bool btnRB     = xboxController.xboxNotif.btnRB;
        
        bool btnB      = xboxController.xboxNotif.btnB;
        bool btnY      = xboxController.xboxNotif.btnY;
        bool btnX      = xboxController.xboxNotif.btnX;

        // 1. Arming Logic (A tuşuna 1.5 sn basılı tutma)
        if (btnSelect) {
            isArmed = false;
            armStartedAt = 0;
            Serial.println("Sistem: DISARMED (Select'e basildi)");
        } else if (btnA) {
            if (armStartedAt == 0) armStartedAt = millis();
            else if (!isArmed && (millis() - armStartedAt) >= ARM_HOLD_MS) {
                isArmed = true;
                steering.enable(true);
                Serial.println("Sistem: ARMED (A'ya basili tutuldu)");
            }
        } else {
            armStartedAt = 0;
        }

        // 2. Vites Kontrolü
        static bool lastBtnX = false, lastBtnB = false, lastBtnY = false;
        if (btnX && !lastBtnX) { currentGear = NEUTRAL; throttle.setDirection(false); Serial.println("Vites: BOS"); }
        else if (btnB && !lastBtnB) { currentGear = DRIVE; throttle.setDirection(false); Serial.println("Vites: ILERI"); }
        else if (btnY && !lastBtnY) { currentGear = REVERSE; throttle.setDirection(true); Serial.println("Vites: GERI"); }
        lastBtnX = btnX; lastBtnB = btnB; lastBtnY = btnY;

        // 3. Güvenlik (Deadman - RB tuşu)
        isDeadmanActive = btnRB;

        if (!isArmed || !isDeadmanActive) {
            throttle.setVoltage(Throttle::IDLE_V);
            brake.setMotor(0, 0);
        } else {
            // 4. Joystick ve Tetik Okumaları
            // joyLHori: 0 (Sol) - 32768 (Orta) - 65535 (Sağ)
            float joyX = max(-1.0f, min(1.0f, (xboxController.xboxNotif.joyLHori - 32768) / 32768.0f));
            
            // trigs: 0 (Basılmamış) - 1023 (Tam Basılı)
            float lt = max(0.0f, min(1.0f, xboxController.xboxNotif.trigLT / 1023.0f));
            float rt = max(0.0f, min(1.0f, xboxController.xboxNotif.trigRT / 1023.0f));

            joyX = applyDeadzone(joyX, STEER_DEADZONE);
            lt = applyDeadzone(lt, TRIGGER_DEADZONE);
            rt = applyDeadzone(rt, TRIGGER_DEADZONE);

            // 5. Sisteme Gönderme
            steering.setTargetAngle(joyX * MAX_STEER_ANGLE);

            if (lt > 0.0f) {
                throttle.setVoltage(Throttle::IDLE_V);
                brake.setMotor(0, static_cast<int>(lt * 255.0f)); 
            } else {
                brake.setMotor(1, 255); 
                
                float max_v = (currentGear == REVERSE) ? Throttle::MAX_REVERSE_V : Throttle::MAX_FORWARD_V;
                float targetVolts = Throttle::IDLE_V + (rt * (max_v - Throttle::IDLE_V));
                
                if (currentGear == NEUTRAL) {
                    throttle.setVoltage(Throttle::IDLE_V);
                } else {
                    throttle.setVoltage(targetVolts);
                }
            }

            // Ekranda Analog Degerleri Gorebilmek Icin Yazdir (Saniyede 4 kez)
            static uint32_t lastPrint = 0;
            if (millis() - lastPrint > 250) {
                Serial.printf("Direksiyon (Sol Joystick): %5.2f  |  Gaz (RT): %5.2f  |  Fren (LT): %5.2f\n", joyX, rt, lt);
                lastPrint = millis();
            }
        }
    }

    // Step motoru sur
    steering.update();
}

// LWIP IPv6 linker hatasını çözmek için boş fonksiyon
extern "C" int lwip_hook_ip6_input(void *pbuf, void *inp) {
    return 0;
}

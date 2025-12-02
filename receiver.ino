/* Receiver (RX)
   - Receives PRESS/RELEASE messages via ESP-NOW
   - Controls 3 continuous MG995 servos by ramping PWM pulse width
*/

#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ---------------------- CONFIG -----------------------
#define CHANNEL 1

// Servo GPIOs (servo0, servo1, servo2)
int SERVO_PIN[3] = {1, 2, 4};

// Per-servo adjustable max speed (PWM offset range)
int maxSpeed[3] = {
    40,   // servo 0 max speed
    6,   // servo 1 max speed
    20    // servo 2 max speed
};

// Per-servo acceleration rate
int accelRate[3] = {
    2,    // servo 0 acceleration
    1,    // servo 1 acceleration
    3     // servo 2 acceleration
};

// Servo objects
Servo servos[3];

// Servo center and working PWM ranges
const int SERVO_CENTER = 90;       // Neutral stop
const int SERVO_MIN_PWM = 45;      // Full reverse limit
const int SERVO_MAX_PWM = 135;     // Full forward limit

// Current + target speeds
int currentSpeed[3] = {0, 0, 0};    // -maxSpeed..+maxSpeed
int targetSpeed[3]  = {0, 0, 0};

// Heartbeat
unsigned long lastPacket = 0;
const unsigned long TIMEOUT_MS = 8000;
bool lostConnection = false;

// Packet from transmitter
struct CommandPacket {
    uint8_t servo;
    int8_t direction;  // -1, 0, +1
};

// ----------------- ESP-NOW CALLBACK ------------------
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    lastPacket = millis();
    lostConnection = false;

    if (len != sizeof(CommandPacket)) return;

    CommandPacket p;
    memcpy(&p, data, sizeof(p));

    if (p.servo == 255) {
        // Heartbeat
        return;
    }

    if (p.servo < 3) {
        // direction is -1, 0, +1
        targetSpeed[p.servo] = p.direction * maxSpeed[p.servo];

        Serial.printf("Servo %d -> Dir %d -> target %d\n",
                      p.servo, p.direction, targetSpeed[p.servo]);
    }
}

// ---------------------- SETUP ------------------------
void setup()
{
    Serial.begin(115200);
    delay(500);

    // Setup servos
    for (int i = 0; i < 3; i++) {
        servos[i].setPeriodHertz(50);
        servos[i].attach(SERVO_PIN[i], 500, 2500); // works best for MG995 continuous
        servos[i].write(SERVO_CENTER);
    }

    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        return;
    }

    esp_now_register_recv_cb(onReceive);

    lastPacket = millis();
    Serial.println("Receiver ready");
}

// ---------------------- LOOP -------------------------
void loop()
{
    unsigned long now = millis();

    // Connection timeout
    if (!lostConnection && now - lastPacket > TIMEOUT_MS) {
        lostConnection = true;
        Serial.println("Connection lost, stopping servos");
        for (int i = 0; i < 3; i++) targetSpeed[i] = 0;
    }

    // Smooth acceleration loop
    for (int s = 0; s < 3; s++) {

        if (currentSpeed[s] < targetSpeed[s]) {
            currentSpeed[s] += accelRate[s];
            if (currentSpeed[s] > targetSpeed[s]) currentSpeed[s] = targetSpeed[s];
        }
        else if (currentSpeed[s] > targetSpeed[s]) {
            currentSpeed[s] -= accelRate[s];
            if (currentSpeed[s] < targetSpeed[s]) currentSpeed[s] = targetSpeed[s];
        }

        // Convert speed (-max..max) → servo angle
        int pwm = SERVO_CENTER + currentSpeed[s];

        // Safety clamp
        pwm = constrain(pwm, SERVO_MIN_PWM, SERVO_MAX_PWM);

        servos[s].write(pwm);
    }

    delay(10);
}
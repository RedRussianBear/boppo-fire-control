#include <Arduino.h>

#define RC_IN       3
#define FIRE_OUT    4
#define CHARGE_OUT  5
#define SENSE_IN    A0

#define PULSE_MIN   500         // All unit microseconds
#define PULSE_MAX   2500
#define GAP_MIN     3000
#define GAP_MAX     100000

#define PULSE_LOW   1000
#define PULSE_HIGH  1800
#define PULSE_THR   1700

#define FIRE_LENGTH     15000
#define FIRE_PAUSE_MIN  140000
#define FIRE_PAUSE_MAX  1000000

#define FIRE_V_THR      1.7
#define SAFE_V_THR      0.1

#define STATE_STARTUP   1
#define STATE_IDLE      2
#define STATE_CHARGING  3
#define STATE_FIRE      4


void measurePWM();

unsigned long gap, duration;
unsigned long pulse_start = 0;
unsigned long pulse_end = 0;
int state = STATE_STARTUP;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(FIRE_OUT, OUTPUT);
    pinMode(CHARGE_OUT, OUTPUT);
    pinMode(RC_IN, INPUT);
    pinMode(SENSE_IN, INPUT);

    attachInterrupt(digitalPinToInterrupt(RC_IN), measurePWM, CHANGE);
    state = STATE_IDLE;
    Serial.begin(9600);
    gap = 0;
    duration = 0;
}

void loop() {
    unsigned long last_activity = micros() - max(pulse_start, pulse_end);
    double cap_v = 5.0 * analogRead(SENSE_IN) / 1024.0;
    Serial.println(cap_v, 3);

    switch (state) {
        case STATE_IDLE:
            if (gap < GAP_MIN || gap > GAP_MAX || duration < PULSE_MIN || duration > PULSE_MAX ||
                last_activity > GAP_MAX) {
                return;
            }

            if (duration > PULSE_THR) {
                state = STATE_CHARGING;
                digitalWrite(CHARGE_OUT, HIGH);
                digitalWrite(LED_BUILTIN, HIGH);
            }

            break;

        case STATE_CHARGING:
            if (cap_v > FIRE_V_THR) {
                digitalWrite(CHARGE_OUT, LOW);
                delay(10);
                state = STATE_FIRE;
                digitalWrite(FIRE_OUT, HIGH);
            }

            break;

        case STATE_FIRE:
            if (cap_v < SAFE_V_THR) {
                digitalWrite(LED_BUILTIN, LOW);
                digitalWrite(FIRE_OUT, LOW);
                state = STATE_IDLE;
                delay(10);
            }

            break;

        default:
            return;
    }

//    unsigned long pause = map((long) duration, PULSE_THR, PULSE_HIGH, FIRE_PAUSE_MAX, FIRE_PAUSE_MIN);
//    pause = constrain(pause, FIRE_PAUSE_MIN, FIRE_PAUSE_MAX);
//    delay(pause / 1000);
}

void measurePWM() {
    static uint8_t old_state = digitalRead(RC_IN);

    uint8_t pin_state = digitalRead(RC_IN);

    if (old_state == LOW && pin_state == HIGH) {        // Rising edge
        pulse_start = micros();
        gap = pulse_start - pulse_end;
    } else if (old_state == HIGH && pin_state == LOW) { // Falling edge
        pulse_end = micros();
        duration = pulse_end - pulse_start;
    }

//    digitalWrite(LED_BUILTIN, HIGH);
    old_state = pin_state;
}
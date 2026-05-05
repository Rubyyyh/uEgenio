#include "mbed.h"
#include "HCSR04.h"
#include <cmath>
#include <cstring>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>
#include <chrono>

using namespace std::chrono;

// ─────────────────────────────────────────────────────────────────────────────
// PIN NUCLEO-F401RE
// ─────────────────────────────────────────────────────────────────────────────
#define TRIG_PIN        D7
#define ECHO_PIN        D8
#define HEAD_SERVO_PIN  D6
#define BUZZER_PIN      D4
#define OLED_SDA        PB_9
#define OLED_SCL        PB_8

// ─────────────────────────────────────────────────────────────────────────────
// SERVO TESTA POSIZIONALE 0°-180°
// ─────────────────────────────────────────────────────────────────────────────
// 0 = servo posizionale classico 0°-180°
// IMPORTANTE: visto che il test 0→180→0 era perfetto, lasciamo 0.
#define CONTINUOUS_SERVO 0

// Stessi valori del test: 500us = 0°, 2500us = 180° su periodo 20ms.
#define HEAD_SERVO_MIN_DUTY    0.025f
#define HEAD_SERVO_MAX_DUTY    0.125f
#define HEAD_SERVO_STEP_DEG    1.0f
#define HEAD_SERVO_STEP_MS     20
#define HEAD_SERVO_CENTER_DEG  90.0f

// Range patrol uguale al test: 0° → 180° → 0°.
#define PATROL_LEFT_ANGLE      0.0f
#define PATROL_RIGHT_ANGLE     180.0f

// Valori vecchi per servo 360°. Con CONTINUOUS_SERVO 0 non vengono usati.
#define HEAD_SERVO_STOP_DUTY   0.075f
#define HEAD_SERVO_CW_DUTY     0.083f
#define HEAD_SERVO_CCW_DUTY    0.067f

// ─────────────────────────────────────────────────────────────────────────────
// BUZZER
// ─────────────────────────────────────────────────────────────────────────────
#define BUZZER_TOGGLE_MS 150

// ─────────────────────────────────────────────────────────────────────────────
// BRACCIO
// ─────────────────────────────────────────────────────────────────────────────
#define ARM_SERVO1_PIN D9
#define ARM_SERVO2_PIN D10
#define ARM_SERVO3_PIN D5
#define ARM_SERVO4_PIN D3
#define ARM_SERVO5_PIN D11

#define ARM_STEP_GRADI       5
#define ARM_RITARDO_MS       45
#define ARM_RITARDO_SLOW_MS  45
#define ARM_PAUSA_MS         100
#define ARM_LOOP_MAX_CYCLES  2  

#define ARM_SEQUENCE_STEP_GRADI      1
#define ARM_SEQUENCE_RITARDO_MS      20

// ─────────────────────────────────────────────────────────────────────────────
// HCSR04
// ─────────────────────────────────────────────────────────────────────────────
#define OBSTACLE_DIST_CM 40.0f
#define HYSTERESIS_CM    5.0f
#define SENSOR_READ_MS   100

// ─────────────────────────────────────────────────────────────────────────────
// OLED SSD1306 128x64
// ─────────────────────────────────────────────────────────────────────────────
#define OLED_ADDR      (0x3C << 1)
#define OLED_W         128
#define OLED_H         64
#define OLED_FRAME_MS  25

// ─────────────────────────────────────────────────────────────────────────────
// SERIALE
// ─────────────────────────────────────────────────────────────────────────────
#define PRINT_MS 500

// ─────────────────────────────────────────────────────────────────────────────
// OGGETTI HARDWARE
// ─────────────────────────────────────────────────────────────────────────────
HCSR04 sensor(TRIG_PIN, ECHO_PIN);

PwmOut headServo(HEAD_SERVO_PIN);
PwmOut armServo1(ARM_SERVO1_PIN);
PwmOut armServo2(ARM_SERVO2_PIN);
PwmOut armServo3(ARM_SERVO3_PIN);
PwmOut armServo4(ARM_SERVO4_PIN);
PwmOut armServo5(ARM_SERVO5_PIN);

DigitalOut led(LED1);
DigitalOut buzzer(BUZZER_PIN);
BufferedSerial pc(USBTX, USBRX, 115200);
I2C i2c(OLED_SDA, OLED_SCL);
Timer mainTimer;

static uint8_t fb[1024];

// ─────────────────────────────────────────────────────────────────────────────
// STATI
// ─────────────────────────────────────────────────────────────────────────────
enum RobotMode {
    MODE_IDLE,
    MODE_PATROL,
    MODE_ARM_LOOP,
    MODE_ARM_SEQUENCE
};

enum OledState {
    OLED_HAPPY,
    OLED_SAD,
    OLED_KISS,
    OLED_WARNING
};

// ─────────────────────────────────────────────────────────────────────────────
// VARIABILI BRACCIO
// ─────────────────────────────────────────────────────────────────────────────
static int p1 = 90;
static int p2 = 90;
static int p3 = 90;
static int p4 = 90;
static int p5 = 30;

// Movimento vecchio in loop, comando 5.
static int armT1 = 90;
static int armT2 = 120;
static int armT3 = 90;
static int armT4 = 90;
static int armT5 = 30;

static bool armTargetHigh = true;
static bool armPauseActive = false;
static uint32_t armPauseStartMs = 0;
static uint32_t lastArmStepMs = 0;
static long armInitialDistance = 1;
static int armLoopCompletedCycles = 0;

// Sequenza nuova, comando 7.
struct ArmStep {
    int t1;
    int t2;
    int t3;
    int t4;
    int t5;
    uint32_t pausaDopoMs;
};

static const ArmStep armSequence[] = {
    {90, 45, 170, 90, 30, 700},
    {90, 45, 170, 90, 60, 400},
    {90, 45, 130, 90, 60, 400},
    {30, 45, 130, 90, 60, 400},
    {30, 45, 170, 90, 60, 400},
    {30, 45, 170, 90, 30, 400},
    {90, 90, 90, 90, 30, 1200}
};

static const int ARM_SEQUENCE_LEN = sizeof(armSequence) / sizeof(armSequence[0]);
static int armSequenceIndex = 0;
static bool armSequenceWaiting = false;
static uint32_t armSequenceWaitStartMs = 0;
static uint32_t armSequenceWaitDurationMs = 0;

// ─────────────────────────────────────────────────────────────────────────────
// SERIALE
// ─────────────────────────────────────────────────────────────────────────────
static void serial_newline() {
    const char nl[2] = {13, 10};
    pc.write(nl, 2);
}

static void serial_println(const char *text) {
    pc.write(text, strlen(text));
    serial_newline();
}

static void serial_printf(const char *fmt, ...) {
    char buf[220];

    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len > 0) {
        if (len >= (int)sizeof(buf)) len = sizeof(buf) - 1;
        pc.write(buf, len);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// SERVO TESTA
// ─────────────────────────────────────────────────────────────────────────────
static float gradi_a_duty(float gradi) {
    if (gradi < 0.0f) gradi = 0.0f;
    if (gradi > 180.0f) gradi = 180.0f;

    return HEAD_SERVO_MIN_DUTY +
           (gradi / 180.0f) * (HEAD_SERVO_MAX_DUTY - HEAD_SERVO_MIN_DUTY);
}

static void head_servo_stop() {
#if CONTINUOUS_SERVO
    headServo.write(HEAD_SERVO_STOP_DUTY);
#else
    // Servo posizionale: per fermarsi basta non cambiare più duty.
#endif
}

static void head_servo_center() {
#if CONTINUOUS_SERVO
    headServo.write(HEAD_SERVO_STOP_DUTY);
#else
    headServo.write(gradi_a_duty(HEAD_SERVO_CENTER_DEG));
#endif
}

static void head_servo_move_positional(float angle) {
#if !CONTINUOUS_SERVO
    headServo.write(gradi_a_duty(angle));
#endif
}

static void head_servo_move_continuous(bool direction) {
#if CONTINUOUS_SERVO
    headServo.write(direction ? HEAD_SERVO_CW_DUTY : HEAD_SERVO_CCW_DUTY);
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
// BUZZER
// ─────────────────────────────────────────────────────────────────────────────
static void buzzer_off(bool &buzzer_state) {
    buzzer_state = false;
    buzzer = 0;
}

static void buzzer_update(uint32_t now, bool buzzer_alarm_active, bool &buzzer_state, uint32_t &lastBuzzerToggleMs) {
    if (!buzzer_alarm_active) {
        buzzer_off(buzzer_state);
        return;
    }

    if (now - lastBuzzerToggleMs >= BUZZER_TOGGLE_MS) {
        lastBuzzerToggleMs = now;
        buzzer_state = !buzzer_state;
        buzzer = buzzer_state ? 1 : 0;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// BRACCIO BASE
// ─────────────────────────────────────────────────────────────────────────────
static void writeArmServo(PwmOut &servo, int gradi) {
    if (gradi < 0) gradi = 0;
    if (gradi > 180) gradi = 180;

    int pulse_width = 500 + (int)((float)gradi * 2000.0f / 180.0f);
    servo.pulsewidth_us(pulse_width);
}

static void arm_write_current() {
    writeArmServo(armServo1, p1);
    writeArmServo(armServo2, p2);
    writeArmServo(armServo3, p3);
    writeArmServo(armServo4, p4);
    writeArmServo(armServo5, p5);
}

static void arm_init() {
    armServo1.period_ms(20);
    armServo2.period_ms(20);
    armServo3.period_ms(20);
    armServo4.period_ms(20);
    armServo5.period_ms(20);

    p1 = 90;
    p2 = 90;
    p3 = 90;
    p4 = 90;
    p5 = 30;

    arm_write_current();
}

static void arm_reset_home() {
    p1 = 90;
    p2 = 90;
    p3 = 90;
    p4 = 90;
    p5 = 30;

    arm_write_current();

    armTargetHigh = true;
    armPauseActive = false;
    armPauseStartMs = 0;
    lastArmStepMs = 0;
    armInitialDistance = 1;
    armLoopCompletedCycles = 0;

    armSequenceIndex = 0;
    armSequenceWaiting = false;
    armSequenceWaitStartMs = 0;
    armSequenceWaitDurationMs = 0;
}

static void arm_step_one_servo(int &pos, int target) {
    if (pos < target) {
        pos += ARM_STEP_GRADI;
        if (pos > target) pos = target;
    } else if (pos > target) {
        pos -= ARM_STEP_GRADI;
        if (pos < target) pos = target;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// BRACCIO LOOP, COMANDO 5
// ─────────────────────────────────────────────────────────────────────────────
static long arm_loop_distance_to_target() {
    return std::abs(armT1 - p1) +
           std::abs(armT2 - p2) +
           std::abs(armT3 - p3) +
           std::abs(armT4 - p4) +
           std::abs(armT5 - p5);
}

static void arm_loop_set_target(int t1, int t2, int t3, int t4, int t5) {
    armT1 = t1;
    armT2 = t2;
    armT3 = t3;
    armT4 = t4;
    armT5 = t5;

    armInitialDistance = arm_loop_distance_to_target();
    if (armInitialDistance <= 0) armInitialDistance = 1;
}

static bool arm_loop_at_target() {
    return p1 == armT1 &&
           p2 == armT2 &&
           p3 == armT3 &&
           p4 == armT4 &&
           p5 == armT5;
}

static void arm_loop_start(uint32_t now) {
    armTargetHigh = true;
    armPauseActive = false;
    armPauseStartMs = 0;
    lastArmStepMs = now;
    armLoopCompletedCycles = 0;
    arm_loop_set_target(90, 120, 90, 90, 30);
}

static void arm_loop_step_towards_target(uint32_t now) {
    long remaining = arm_loop_distance_to_target();
    uint32_t ritardo = ARM_RITARDO_MS;

    if (remaining <= (armInitialDistance * 10L) / 100L) {
        ritardo = ARM_RITARDO_SLOW_MS;
    }

    if (now - lastArmStepMs < ritardo) return;
    lastArmStepMs = now;

    arm_step_one_servo(p1, armT1);
    arm_step_one_servo(p2, armT2);
    arm_step_one_servo(p3, armT3);
    arm_step_one_servo(p4, armT4);
    arm_step_one_servo(p5, armT5);

    arm_write_current();
}

static bool arm_loop_update(uint32_t now) {
    if (armLoopCompletedCycles >= ARM_LOOP_MAX_CYCLES) {
        return true;
    }

    if (armPauseActive) {
        if (now - armPauseStartMs >= ARM_PAUSA_MS) {
            armPauseActive = false;

            if (armTargetHigh) {
                arm_loop_set_target(90, 60, 90, 90, 30);
                armTargetHigh = false;
            } else {
                armLoopCompletedCycles++;

                if (armLoopCompletedCycles >= ARM_LOOP_MAX_CYCLES) {
                    return true;
                }

                arm_loop_set_target(90, 120, 90, 90, 30);
                armTargetHigh = true;
            }
        }
        return false;
    }

    if (arm_loop_at_target()) {
        armPauseActive = true;
        armPauseStartMs = now;
        return false;
    }

    arm_loop_step_towards_target(now);
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// BRACCIO SEQUENZA, COMANDO 7
// ─────────────────────────────────────────────────────────────────────────────
static bool arm_sequence_at_target(const ArmStep &s) {
    return p1 == s.t1 &&
           p2 == s.t2 &&
           p3 == s.t3 &&
           p4 == s.t4 &&
           p5 == s.t5;
}

static void arm_sequence_start(uint32_t now) {
    armSequenceIndex = 0;
    armSequenceWaiting = false;
    armSequenceWaitStartMs = 0;
    armSequenceWaitDurationMs = 0;
    lastArmStepMs = now;
}

static void arm_sequence_step_one_servo(int &pos, int target) {
    if (pos < target) {
        pos += ARM_SEQUENCE_STEP_GRADI;
        if (pos > target) pos = target;
    } else if (pos > target) {
        pos -= ARM_SEQUENCE_STEP_GRADI;
        if (pos < target) pos = target;
    }
}

static void arm_sequence_step_towards_target(uint32_t now, const ArmStep &s) {
    if (now - lastArmStepMs < ARM_SEQUENCE_RITARDO_MS) return;
    lastArmStepMs = now;

    arm_sequence_step_one_servo(p1, s.t1);
    arm_sequence_step_one_servo(p2, s.t2);
    arm_sequence_step_one_servo(p3, s.t3);
    arm_sequence_step_one_servo(p4, s.t4);
    arm_sequence_step_one_servo(p5, s.t5);

    arm_write_current();
}

static bool arm_sequence_update(uint32_t now) {
    if (armSequenceIndex >= ARM_SEQUENCE_LEN) return true;

    if (armSequenceWaiting) {
        if (now - armSequenceWaitStartMs >= armSequenceWaitDurationMs) {
            armSequenceWaiting = false;
            armSequenceIndex++;
            lastArmStepMs = now;

            if (armSequenceIndex >= ARM_SEQUENCE_LEN) return true;
        }
        return false;
    }

    const ArmStep &s = armSequence[armSequenceIndex];

    if (arm_sequence_at_target(s)) {
        armSequenceWaiting = true;
        armSequenceWaitStartMs = now;
        armSequenceWaitDurationMs = s.pausaDopoMs;
        return false;
    }

    arm_sequence_step_towards_target(now, s);
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// OLED BASE
// ─────────────────────────────────────────────────────────────────────────────
static void oled_cmd(uint8_t cmd) {
    char d[2] = {0x00, (char)cmd};
    i2c.write(OLED_ADDR, d, 2);
}

static void oled_init() {
    uint8_t init[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
        0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,
        0xAF
    };

    for (uint8_t c : init) oled_cmd(c);
}

static void oled_flush() {
    oled_cmd(0x21);
    oled_cmd(0);
    oled_cmd(127);

    oled_cmd(0x22);
    oled_cmd(0);
    oled_cmd(7);

    char chunk[17];
    chunk[0] = 0x40;

    for (int i = 0; i < 1024; i += 16) {
        memcpy(&chunk[1], &fb[i], 16);
        i2c.write(OLED_ADDR, chunk, 17);
    }
}

static void px(int x, int y, bool on = true) {
    if (x < 0 || x >= OLED_W || y < 0 || y >= OLED_H) return;

    if (on) fb[x + (y / 8) * OLED_W] |= (1 << (y & 7));
    else fb[x + (y / 8) * OLED_W] &= ~(1 << (y & 7));
}

static void line(int x0, int y0, int x1, int y1) {
    int dx = std::abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (true) {
        px(x0, y0);
        if (x0 == x1 && y0 == y1) break;

        int e2 = 2 * err;

        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }

        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

static void fillRect(int x, int y, int w, int h) {
    for (int yy = y; yy < y + h; yy++) {
        for (int xx = x; xx < x + w; xx++) {
            px(xx, yy);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// DISEGNI OLED
// ─────────────────────────────────────────────────────────────────────────────
static void drawHappy() {
    for (int x = 40; x <= 88; x++) {
        int y = 50 - (int)(powf((float)(x - 64), 2.0f) / 50.0f);
        px(x, y);
    }
}

static void drawSad() {
    for (int x = 40; x <= 88; x++) {
        int y = 35 + (int)(powf((float)(x - 64), 2.0f) / 50.0f);
        px(x, y);
    }
}

static void drawKiss(int cx, int cy, float scala = 2.0f) {
    int raggio = (int)(4 * scala);
    int offset = (int)(3 * scala);

    for (float a = 1.5f; a < 4.7f; a += 0.15f) {
        px(cx - (int)(raggio * cosf(a)), cy - offset + (int)(raggio * sinf(a)));
        px(cx - (int)(raggio * cosf(a)), cy + offset + (int)(raggio * sinf(a)));
    }
}

static void drawHeart(int cx, int cy, int size) {
    for (int x = -size; x <= size; x++) {
        for (int y = -size; y <= size; y++) {
            float xf = (float)x / (size * 0.85f);
            float yf = (float)y / (size * 0.85f);
            float v = xf * xf + yf * yf - 1.0f;

            if (v * v * v - xf * xf * yf * yf * yf <= 0.0f) {
                px(cx + x, cy - y);
            }
        }
    }

    if (size > 3) px(cx + 1, cy - 1, false);
}

static void drawWarningTriangle() {
    int topX = 64;
    int topY = 10;
    int leftX = 34;
    int leftY = 54;
    int rightX = 94;
    int rightY = 54;

    for (int i = 0; i < 3; i++) {
        line(topX, topY + i, leftX + i, leftY);
        line(topX, topY + i, rightX - i, rightY);
        line(leftX + i, leftY - i, rightX - i, rightY - i);
    }

    fillRect(61, 26, 6, 16);
    fillRect(61, 46, 6, 6);
}

// ─────────────────────────────────────────────────────────────────────────────
// MAIN
// ─────────────────────────────────────────────────────────────────────────────
int main() {
    headServo.period_ms(20);
    head_servo_center();

    arm_init();
    buzzer = 0;

    i2c.frequency(400000);
    pc.set_blocking(false);

    mainTimer.start();
    oled_init();

    RobotMode mode = MODE_IDLE;
    OledState oled_state = OLED_HAPPY;

    ThisThread::sleep_for(700ms);

    serial_newline();
    serial_println("=== ROBOT OLED + HCSR04 + TESTA + BRACCIO ===");
    serial_println("IDLE:   1=Happy  2=Sad  3=Kiss+Heart  4=Patrol  5=Braccio loop  7=Sequenza braccio");
    serial_println("PATROL: servo testa posizionale 0 -> 180 -> 0");
    serial_println("ARM:    solo 6=Stop braccio e ritorno in IDLE");
    serial_println("BUZZER: D4, suona se ostacolo entro 150 cm mentre guarda a sinistra");
    serial_printf("HEAD_STEP=%.1f  HEAD_MS=%d  ARM_STEP=%d  ARM_DELAY=%d", HEAD_SERVO_STEP_DEG, HEAD_SERVO_STEP_MS, ARM_STEP_GRADI, ARM_RITARDO_MS);
    serial_newline();
    serial_newline();

    float current_angle = HEAD_SERVO_CENTER_DEG;
    float patrol_angle_est = HEAD_SERVO_CENTER_DEG;
    float direction_pos = 1.0f;
    bool direction_cont = true;
    bool obstacle_active = false;
    bool obstacle_seen_on_right = false;
    bool buzzer_alarm_active = false;
    bool buzzer_state = false;
    float last_dist = -1.0f;
    int invalid_sensor_count = 0;

    int near_count = 0;
    int far_count = 0;

    float heartX = 75.0f;
    float heartY = 25.0f;

    uint32_t lastHeadServoMs = 0;
    uint32_t lastSensorMs = 0;
    uint32_t lastOledMs = 0;
    uint32_t lastPrintMs = 0;
    uint32_t lastBuzzerToggleMs = 0;

    while (true) {
        uint32_t now = (uint32_t)(mainTimer.elapsed_time().count() / 1000);

        // ─────────────────────────────────────────────────────────────────────
        // SERIALE
        // ─────────────────────────────────────────────────────────────────────
        if (pc.readable()) {
            char cmd;

            if (pc.read(&cmd, 1) == 1) {
                if (mode == MODE_IDLE) {
                    if (cmd == '1') {
                        oled_state = OLED_HAPPY;
                        serial_println(">> Happy");
                    } else if (cmd == '2') {
                        oled_state = OLED_SAD;
                        serial_println(">> Sad");
                    } else if (cmd == '3') {
                        oled_state = OLED_KISS;
                        heartX = 75.0f;
                        heartY = 25.0f;
                        serial_println(">> Kiss+Heart");
                    } else if (cmd == '4') {
                        mode = MODE_PATROL;
                        oled_state = OLED_WARNING;

                        obstacle_active = false;
                        obstacle_seen_on_right = false;
                        buzzer_alarm_active = false;
                        buzzer_off(buzzer_state);

                        near_count = 0;
                        far_count = 0;

                        current_angle = PATROL_LEFT_ANGLE;
                        patrol_angle_est = PATROL_LEFT_ANGLE;
                        direction_pos = 1.0f;
                        direction_cont = true;
                        head_servo_move_positional(current_angle);

                        serial_println(">> PATROL MODE ATTIVA");
                    } else if (cmd == '5') {
                        mode = MODE_ARM_LOOP;
                        oled_state = OLED_HAPPY;

                        buzzer_alarm_active = false;
                        buzzer_off(buzzer_state);

                        head_servo_center();
                        arm_loop_start(now);

                        serial_println(">> ARM LOOP MODE ATTIVA");
                    } else if (cmd == '7') {
                        mode = MODE_ARM_SEQUENCE;
                        oled_state = OLED_HAPPY;

                        buzzer_alarm_active = false;
                        buzzer_off(buzzer_state);

                        head_servo_center();
                        arm_sequence_start(now);

                        serial_println(">> ARM SEQUENCE MODE ATTIVA");
                    }
                } else if (mode == MODE_PATROL) {
                    if (cmd == '6') {
                        mode = MODE_IDLE;
                        oled_state = OLED_HAPPY;

                        obstacle_active = false;
                        obstacle_seen_on_right = false;
                        buzzer_alarm_active = false;
                        buzzer_off(buzzer_state);

                        near_count = 0;
                        far_count = 0;

                        current_angle = HEAD_SERVO_CENTER_DEG;
                        patrol_angle_est = HEAD_SERVO_CENTER_DEG;
                        direction_pos = 1.0f;
                        direction_cont = true;
                        head_servo_center();

                        serial_println(">> PATROL MODE DISATTIVATA: testa al centro + faccia felice");
                    } else {
                        serial_println(">> Comando ignorato in PATROL. Usa 6 per uscire.");
                    }
                } else if (mode == MODE_ARM_LOOP || mode == MODE_ARM_SEQUENCE) {
                    if (cmd == '6') {
                        mode = MODE_IDLE;
                        oled_state = OLED_HAPPY;

                        buzzer_alarm_active = false;
                        buzzer_off(buzzer_state);

                        head_servo_center();
                        arm_reset_home();

                        serial_println(">> ARM DISATTIVATO: braccio home + faccia felice");
                    } else {
                        serial_println(">> Comando ignorato in ARM. Usa 6 per uscire.");
                    }
                }
            }
        }

        // ─────────────────────────────────────────────────────────────────────
        // HCSR04: lavora solo in PATROL
        // ─────────────────────────────────────────────────────────────────────
        if (mode == MODE_PATROL && now - lastSensorMs >= SENSOR_READ_MS) {
            lastSensorMs = now;

            float d = sensor.read_cm_avg(3, 15);

            if (d > 0.0f && d < 400.0f) {
                invalid_sensor_count = 0;
                last_dist = d;

                if (d < OBSTACLE_DIST_CM) {
                    near_count++;
                    far_count = 0;
                } else if (d > OBSTACLE_DIST_CM + HYSTERESIS_CM) {
                    far_count++;
                    near_count = 0;
                }

                if (!obstacle_active && near_count >= 2) {
                    obstacle_active = true;
                    obstacle_seen_on_right = patrol_angle_est <= HEAD_SERVO_CENTER_DEG;
                    buzzer_alarm_active = !obstacle_seen_on_right;
                    lastBuzzerToggleMs = now;
                    buzzer_off(buzzer_state);
                    head_servo_stop();

                    serial_printf(">> OSTACOLO: %.1f cm | Ang: %.1f deg | lato: %s -> TESTA STOP", last_dist, patrol_angle_est, obstacle_seen_on_right ? "DESTRA" : "SINISTRA");
                    serial_newline();
                }

                if (obstacle_active && far_count >= 2) {
                    obstacle_active = false;
                    obstacle_seen_on_right = false;
                    buzzer_alarm_active = false;
                    buzzer_off(buzzer_state);

                    serial_printf(">> VIA LIBERA: %.1f cm -> TESTA RIPARTE", last_dist);
                    serial_newline();
                }
            } else {
                invalid_sensor_count++;
                last_dist = -1.0f;
                near_count = 0;

                // Se il sensore non risponde mentre era fermo, non lasciare il robot bloccato per sempre.
                if (obstacle_active && invalid_sensor_count >= 5) {
                    obstacle_active = false;
                    obstacle_seen_on_right = false;
                    buzzer_alarm_active = false;
                    far_count = 0;
                    buzzer_off(buzzer_state);
                    serial_println(">> HCSR04 NO ECHO: misura non valida, testa libera per sicurezza");
                }
            }
        }

        led = (mode == MODE_PATROL && obstacle_active) ? 1 : 0;

        // BUZZER intermittente se ostacolo visto a sinistra.
        buzzer_update(now, mode == MODE_PATROL && obstacle_active && buzzer_alarm_active, buzzer_state, lastBuzzerToggleMs);

        // ─────────────────────────────────────────────────────────────────────
        // TESTA
        // ─────────────────────────────────────────────────────────────────────
        if (now - lastHeadServoMs >= HEAD_SERVO_STEP_MS) {
            lastHeadServoMs = now;

            if (mode == MODE_IDLE || mode == MODE_ARM_LOOP || mode == MODE_ARM_SEQUENCE) {
                head_servo_center();
            } else if (mode == MODE_PATROL) {
                if (obstacle_active) {
                    head_servo_stop();
                } else {
                    current_angle += direction_pos * HEAD_SERVO_STEP_DEG;

                    if (current_angle >= PATROL_RIGHT_ANGLE) {
                        current_angle = PATROL_RIGHT_ANGLE;
                        direction_pos = -1.0f;
                    }

                    if (current_angle <= PATROL_LEFT_ANGLE) {
                        current_angle = PATROL_LEFT_ANGLE;
                        direction_pos = 1.0f;
                    }

                    patrol_angle_est = current_angle;
                    head_servo_move_positional(current_angle);
                }
            }
        }

        // ─────────────────────────────────────────────────────────────────────
        // BRACCIO
        // ─────────────────────────────────────────────────────────────────────
        if (mode == MODE_ARM_LOOP) {
            if (arm_loop_update(now)) {
                mode = MODE_IDLE;
                oled_state = OLED_HAPPY;
                head_servo_center();
                arm_reset_home();
                serial_println(">> Saluto finito: ritorno automatico in IDLE");
            }
        } else if (mode == MODE_ARM_SEQUENCE) {
            if (arm_sequence_update(now)) {
                mode = MODE_IDLE;
                oled_state = OLED_HAPPY;
                head_servo_center();
                arm_reset_home();
                serial_println(">> Sequenza braccio finita: ritorno automatico in IDLE");
            }
        }

        // ─────────────────────────────────────────────────────────────────────
        // PRINT DEBUG
        // ─────────────────────────────────────────────────────────────────────
        if (now - lastPrintMs >= PRINT_MS) {
            lastPrintMs = now;

            if (mode == MODE_IDLE) {
                serial_println("Mode: IDLE | comandi: 1 2 3 4 5 7");
            } else if (mode == MODE_PATROL) {
                serial_printf(
                    "Mode: PATROL | Ang: %6.1f deg | Dist: %6.1f cm | %s | lato: %s | buzzer: %s",
                    patrol_angle_est,
                    last_dist,
                    obstacle_active ? "[FERMO]" : "[GIRA]",
                    patrol_angle_est <= HEAD_SERVO_CENTER_DEG ? "DESTRA" : "SINISTRA",
                    buzzer_alarm_active ? "ON" : "OFF"
                );
                serial_newline();
            } else if (mode == MODE_ARM_LOOP) {
                serial_printf("Mode: ARM LOOP | ciclo=%d/%d | p1=%d p2=%d p3=%d p4=%d p5=%d", armLoopCompletedCycles, ARM_LOOP_MAX_CYCLES, p1, p2, p3, p4, p5);
                serial_newline();
            } else if (mode == MODE_ARM_SEQUENCE) {
                serial_printf("Mode: ARM SEQ | step=%d/%d | p1=%d p2=%d p3=%d p4=%d p5=%d", armSequenceIndex + 1, ARM_SEQUENCE_LEN, p1, p2, p3, p4, p5);
                serial_newline();
            }
        }

        // ─────────────────────────────────────────────────────────────────────
        // OLED
        // ─────────────────────────────────────────────────────────────────────
        if (now - lastOledMs >= OLED_FRAME_MS) {
            lastOledMs = now;
            memset(fb, 0, sizeof(fb));

            float t = now / 1000.0f;

            switch (oled_state) {
                case OLED_HAPPY:
                    drawHappy();
                    break;

                case OLED_SAD:
                    drawSad();
                    break;

                case OLED_KISS: {
                    drawKiss(64, 35);

                    heartX += 1.2f;
                    float currentY = heartY + sinf(t * 8.0f) * 4.0f;
                    int pulse = 6 + (int)(1.5f * sinf(t * 12.0f));

                    if (heartX > 135.0f) {
                        heartX = 75.0f;
                        heartY = 25.0f;
                    }

                    drawHeart((int)heartX, (int)currentY, pulse);
                    break;
                }

                case OLED_WARNING:
                    drawWarningTriangle();
                    break;
            }

            oled_flush();
        }

        ThisThread::sleep_for(5ms);
    }
}

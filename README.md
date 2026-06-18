# Atwood
Atwood Machine Ghina
// ╔══════════════════════════════════════════════════════════════╗
// ║          ATWOOD MACHINE — ESP32 + Blynk Firmware             ║
// ║  Physics corrected: a = 2d/t²  (from rest, uniform accel)    ║
// ║  All LCD rows guaranteed 16-char wide — no overlap possible   ║
// ║  Mid-flight Blynk writes are snapshotted at drop moment       ║
// ║  Timeout guard prevents infinite hang on IR_STOP failure      ║
// ╚══════════════════════════════════════════════════════════════╝

#define BLYNK_PRINT Serial

// ── Blynk credentials ───────────────────────────────────────────
#define BLYNK_TEMPLATE_ID "TMPL6C1eRBQIo" 
#define BLYNK_TEMPLATE_NAME "Atwood Machine Reza" 
#define BLYNK_AUTH_TOKEN "eN7bh9U7jy9Z2-O06R_xEXTqTg2fmRhV"

// ── Blynk Virtual Pin Map ───────────────────────────────────────
//  INPUTS  (slider → ESP32)
//    V0  = Mass A    [g]
//    V1  = Mass B    [g]
//    V2  = Distance  [cm]
//  OUTPUTS (ESP32 → display widget)
//    V3  = Acceleration  [m/s²]
//    V4  = Tension       [N]
//    V5  = Velocity avg  [m/s]
//    V6  = Velocity final[m/s]
//    V7  = Force A       [N]
//    V8  = Force B       [N]
//    V9  = Weight A      [N]
//    V10 = Weight B      [N]

#include <stdarg.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ── Hardware ────────────────────────────────────────────────────
LiquidCrystal_I2C lcd(0x27, 16, 2);
const int IR_START = 26;   // breaks beam when mass is at rest on line
const int IR_STOP  = 25;   // breaks beam when mass passes lower line

// ── WiFi ────────────────────────────────────────────────────────
char ssid[] = "PinkGinny";
char pass[] = "gendiscantik";

// ── Physics constant ────────────────────────────────────────────
const float G = 9.81f;   // m/s²

// ── Input state (updated by Blynk sliders, user units) ──────────
float distanceCm = 30.0f;
float massAGram  = 100.0f;
float massBGram  = 100.0f;

// ── Snapshot variables ──────────────────────────────────────────
// Locked at the instant the mass leaves IR_START.
// Prevents Blynk slider writes from corrupting physics mid-flight.
float snapDistCm = 30.0f;
float snapMassAg = 100.0f;
float snapMassBg = 100.0f;

// ── State machine ───────────────────────────────────────────────
unsigned long startTime   = 0;
bool isCalculating        = false;
bool readyToStart         = false;

// 30-second safety timeout: recovers if IR_STOP never fires
const unsigned long TIMEOUT_US = 30000000UL;

// ── Standby scrolling (LCD row 1) ───────────────────────────────
String        scrollText  = "";
int           scrollPos   = 0;
unsigned long lastScrollMs = 0;
const unsigned int SCROLL_MS = 350;

// ════════════════════════════════════════════════════════════════
//  lcdRow() — Safe LCD printer
//
//  Writes exactly 16 chars on the given row.
//  vsnprintf caps the format output at 16 chars, then right-pads
//  with spaces to guarantee no leftover characters from a longer
//  previous message are ever visible.
// ════════════════════════════════════════════════════════════════
void lcdRow(uint8_t row, const char* fmt, ...) {
    char buf[17];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);   // max 16 content chars + '\0'
    va_end(ap);
    // Pad right with spaces to exactly 16 chars
    for (int i = (int)strlen(buf); i < 16; i++) buf[i] = ' ';
    buf[16] = '\0';
    lcd.setCursor(0, row);
    lcd.print(buf);
}

// ════════════════════════════════════════════════════════════════
//  BLYNK WRITE HANDLERS  (inputs from sliders)
// ════════════════════════════════════════════════════════════════
BLYNK_WRITE(V0) {   // Mass A [g]
    massAGram = max(1.0f, param.asFloat());
    if (!isCalculating && !readyToStart) updateStandby();
}
BLYNK_WRITE(V1) {   // Mass B [g]
    massBGram = max(1.0f, param.asFloat());
    if (!isCalculating && !readyToStart) updateStandby();
}
BLYNK_WRITE(V2) {   // Distance [cm]
    distanceCm = max(1.0f, param.asFloat());
    if (!isCalculating && !readyToStart) updateStandby();
}

// ════════════════════════════════════════════════════════════════
//  STANDBY DISPLAY
// ════════════════════════════════════════════════════════════════
void updateStandby() {
    // Row 0: masses — worst case "A:200g   B:200g" = 15 chars ✓
    lcdRow(0, "A:%.0fg   B:%.0fg", massAGram, massBGram);
    // Row 1: scrolling distance + status
    scrollText = "  D:" + String(distanceCm, 1) + "cm  Waiting...  ";
    scrollPos  = 0;
}

void scrollStandby() {
    if (millis() - lastScrollMs >= SCROLL_MS) {
        lastScrollMs = millis();
        int len = scrollText.length();
        String disp = "";
        for (int i = 0; i < 16; i++)
            disp += scrollText[(scrollPos + i) % len];
        lcd.setCursor(0, 1);
        lcd.print(disp);
        if (++scrollPos >= len) scrollPos = 0;
    }
}

// ════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    Serial.println(F("[BOOT] Waiting for I2C voltage stabilization..."));
    delay(3000);   // Preserved: allows I2C bus to stabilize before init

    lcd.init();
    delay(500);    // Preserved: second settle after init
    lcd.backlight();
    lcdRow(0, "System Booting..");
    lcdRow(1, "Please wait...  ");

    pinMode(IR_START, INPUT);
    pinMode(IR_STOP,  INPUT);

    Serial.println(F("[BOOT] Connecting to Blynk..."));
    Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
    Blynk.syncVirtual(V0, V1, V2);   // pull current slider values on boot

    lcdRow(0, "Blynk Connected!");
    lcdRow(1, "                ");
    delay(1500);

    lcd.clear();
    updateStandby();
}

// ════════════════════════════════════════════════════════════════
//  MAIN LOOP — 4-state machine
//
//  A → STANDBY    : idle, scroll distance info
//  B → DETECTED   : mass rests on IR_START (beam broken)
//  C → TIMING     : mass leaves IR_START (timing begins)
//  D → CALCULATE  : mass reaches IR_STOP (timing ends, results)
// ════════════════════════════════════════════════════════════════
void loop() {
    Blynk.run();

    // ── A: STANDBY — beam clear, nothing loaded ──────────────────
    if (!isCalculating && !readyToStart && digitalRead(IR_START) == HIGH) {
        scrollStandby();
    }

    // ── B: DETECTED — mass placed on start line (beam broken) ────
    else if (!isCalculating && !readyToStart && digitalRead(IR_START) == LOW) {
        readyToStart = true;
        lcdRow(0, "Object Detected!");
        lcdRow(1, "Release to drop ");
    }

    // ── C: TIMING START — mass leaves start line (beam clear) ────
    else if (readyToStart && !isCalculating && digitalRead(IR_START) == HIGH) {
        delay(5);   // IR debounce: confirm beam is genuinely clear
        if (digitalRead(IR_START) == HIGH) {
            // ── SNAPSHOT: lock inputs against mid-flight Blynk writes ──
            snapDistCm    = distanceCm;
            snapMassAg    = massAGram;
            snapMassBg    = massBGram;

            startTime     = micros();
            isCalculating = true;
            readyToStart  = false;

            lcdRow(0, "  Calculating...");
            lcdRow(1, "                ");
        }
    }

    // ── TIMEOUT: recover if IR_STOP never fires ──────────────────
    if (isCalculating && (micros() - startTime) >= TIMEOUT_US) {
        isCalculating = false;
        lcdRow(0, "ERR: IR Timeout!");
        lcdRow(1, "Check IR_STOP   ");
        delay(3000);
        lcd.clear();
        updateStandby();
        return;
    }

    // ── D: STOP — mass reaches lower sensor ──────────────────────
    if (isCalculating && digitalRead(IR_STOP) == LOW) {
        const unsigned long stopTime = micros();
        isCalculating = false;

        // ── Unit conversion to SI ────────────────────────────────
        const float distM   = snapDistCm / 100.0f;          // cm  → m
        const float massAkg = snapMassAg / 1000.0f;         // g   → kg
        const float massBkg = snapMassBg / 1000.0f;         // g   → kg

        // ── Elapsed time (guarded against zero) ─────────────────
        const float t = max(0.0001f, (float)(stopTime - startTime) / 1e6f);

        // ════════════════════════════════════════════════════════
        //  KINEMATICS  — uniform acceleration from rest
        //
        //  Starting from rest:  d = ½·a·t²
        //  → a_measured = 2d / t²
        //
        //  v_avg   = d / t           (measured average velocity)
        //  v_final = a·t = 2d / t    (final velocity = 2 × v_avg)
        //
        //  NOTE: v_final = 2 × v_avg is always true for constant
        //  acceleration from rest. This is a useful cross-check.
        // ════════════════════════════════════════════════════════
        const float accelMes     = (2.0f * distM) / (t * t);
        const float velocityAvg  = distM / t;
        const float velocityFinal = accelMes * t;            // = 2 × v_avg

        // ── Weights ──────────────────────────────────────────────
        const float weightA = massAkg * G;
        const float weightB = massBkg * G;

        // ── Net force on each mass (F = m·a) ─────────────────────
        const float forceA = massAkg * accelMes;
        const float forceB = massBkg * accelMes;

        // ── Tension (Newton's 2nd on each side of string) ────────
        //
        //  Let mH = heavier (descending), mL = lighter (ascending)
        //
        //  Descending: mH·g − T = mH·a  →  T_H = mH·(g − a)
        //  Ascending:  T − mL·g = mL·a  →  T_L = mL·(g + a)
        //
        //  Both T_H and T_L should be equal in ideal case.
        //  Average them for the best measured estimate.
        const float mH = max(massAkg, massBkg);
        const float mL = min(massAkg, massBkg);
        const float tFromHeavy = max(0.0f, mH * (G - accelMes));
        const float tFromLight = mL * (G + accelMes);
        const float tension    = (tFromHeavy + tFromLight) / 2.0f;

        // ── Theoretical values (no friction assumed) ─────────────
        //
        //  a_theory  = (mH − mL)·g / (mH + mL)
        //  T_theory  = 2·mH·mL·g   / (mH + mL)
        const float drivingForce = fabsf(massAkg - massBkg) * G;
        const float accelTh      = drivingForce / (massAkg + massBkg);
        const float tensionTh    = (2.0f * mH * mL * G) / (mH + mL);

        // ── Friction ─────────────────────────────────────────────
        //
        //  Net driving force − measured net force = friction losses
        //  f = |mA−mB|·g − (mA+mB)·a_measured
        //  Clamped to 0 (negative = pure measurement noise)
        const float friction = max(0.0f,
            drivingForce - (massAkg + massBkg) * accelMes);

        // ── Percent errors vs theoretical ────────────────────────
        const float errAccel   = (accelTh   > 0.0f)
            ? fabsf(accelMes - accelTh)  / accelTh   * 100.0f : 0.0f;
        const float errTension = (tensionTh > 0.0f)
            ? fabsf(tension  - tensionTh) / tensionTh * 100.0f : 0.0f;

        // ════════════════════════════════════════════════════════
        //  SERIAL REPORT
        // ════════════════════════════════════════════════════════
        Serial.println(F("\n╔══════════ ATWOOD MACHINE RESULTS ══════════╗"));
        Serial.printf( "║ Snapshot  : A=%.1fg  B=%.1fg  d=%.1fcm\n",
                        snapMassAg, snapMassBg, snapDistCm);
        Serial.printf( "║ Time      : t = %.4f s\n", t);
        Serial.println(F("║────────────── Kinematics ───────────────────"));
        Serial.printf( "║ Accel mes : %.4f m/s²\n", accelMes);
        Serial.printf( "║ Accel th  : %.4f m/s²   err = %.1f%%\n", accelTh, errAccel);
        Serial.printf( "║ V avg     : %.4f m/s\n",  velocityAvg);
        Serial.printf( "║ V final   : %.4f m/s\n",  velocityFinal);
        Serial.println(F("║────────────── Forces ───────────────────────"));
        Serial.printf( "║ Weight A  : %.4f N\n", weightA);
        Serial.printf( "║ Weight B  : %.4f N\n", weightB);
        Serial.printf( "║ Force A   : %.4f N\n", forceA);
        Serial.printf( "║ Force B   : %.4f N\n", forceB);
        Serial.printf( "║ Tension m : %.4f N  (Th=%.4f, H=%.4f, L=%.4f)\n",
                        tension, tensionTh, tFromHeavy, tFromLight);
        Serial.printf( "║ Tension err: %.1f%%\n", errTension);
        Serial.printf( "║ Friction  : %.4f N\n", friction);
        Serial.println(F("╚════════════════════════════════════════════╝"));

        // ════════════════════════════════════════════════════════
        //  BLYNK OUTPUT
        // ════════════════════════════════════════════════════════
        Blynk.virtualWrite(V3,  accelMes);
        Blynk.virtualWrite(V4,  tension);
        Blynk.virtualWrite(V5,  velocityAvg);
        Blynk.virtualWrite(V6,  velocityFinal);
        Blynk.virtualWrite(V7,  forceA);
        Blynk.virtualWrite(V8,  forceB);
        Blynk.virtualWrite(V9,  weightA);
        Blynk.virtualWrite(V10, weightB);
        Blynk.virtualWrite(V11, friction);
        Blynk.virtualWrite(V12, errAccel);

        // ════════════════════════════════════════════════════════
        //  LCD RESULTS — 4 pages
        //
        //  Every row goes through lcdRow() which:
        //    1. Uses vsnprintf(buf, 17, ...) → max 16 content chars
        //    2. Right-pads with spaces to exactly 16 chars
        //  → Overflow and leftover chars are BOTH impossible.
        //
        //  Character-count proof for worst-case values
        //  (mass ≤ 200g → weight/force/tension < 2N,
        //   distance ≤ 300cm, velocity < 10 m/s):
        // ════════════════════════════════════════════════════════

        // ── Page 1: Timing & Distance (3 s) ──────────────────────
        // Row 0: "t=%.4fs"      → "t=9.9999s"      = 10 chars ✓
        // Row 1: "d=%.2fcm"     → "d=300.00cm"     = 10 chars ✓
        lcd.clear();
        lcdRow(0, "t=%.4fs",   t);
        lcdRow(1, "d=%.2fcm",  snapDistCm);
        delay(3000);

        // ── Page 2: Kinematics (4 s) ──────────────────────────────
        // Row 0: "Va=%.2f Vf=%.2f"  → "Va=9.81 Vf=9.81" = 15 chars ✓
        //         (3 + 4 + 1 + 3 + 4 = 15)
        // Row 1: "a=%.4f m/s2"      → "a=9.8100 m/s2"   = 13 chars ✓
        lcd.clear();
        lcdRow(0, "Va=%.2f Vf=%.2f", velocityAvg, velocityFinal);
        lcdRow(1, "a=%.4f m/s2",     accelMes);
        delay(4000);

        // ── Page 3: Weights & Forces (4 s) ────────────────────────
        // Row 0: "WA=%.2fN WB=%.2f" → "WA=1.96N WB=1.96" = 16 chars ✓
        //         (3 + 4 + 2 + 3 + 4 = 16)
        // Row 1: "FA=%.2fN FB=%.2f" → "FA=1.96N FB=1.96" = 16 chars ✓
        //         (3 + 4 + 2 + 3 + 4 = 16)
        lcd.clear();
        lcdRow(0, "WA=%.2fN WB=%.2f", weightA, weightB);
        lcdRow(1, "FA=%.2fN FB=%.2f", forceA,  forceB);
        delay(4000);

        // ── Page 4: Tension & Friction (4 s) ──────────────────────
        // Row 0: "Tens:%.4fN" → "Tens:1.9620N" = 12 chars ✓
        // Row 1: "Fric:%.4fN" → "Fric:0.0000N" = 12 chars ✓
        lcd.clear();
        lcdRow(0, "Tens:%.4fN", tension);
        lcdRow(1, "Fric:%.4fN", friction);
        delay(4000);

        // ── Return to standby ─────────────────────────────────────
        lcd.clear();
        updateStandby();
    }
}

#include <LCD.h>
#include <LiquidCrystal_I2C.h>

 
#include "TimerOne.h"
 
// ----------------------------------------------------------------------
// Defines:
//
//

// BOT is PWD, TOP is On or Off

#define OUTPUT_PhaseA_BOT 6
#define OUTPUT_PhaseB_BOT 7
#define OUTPUT_PhaseC_BOT 8
#define OUTPUT_PhaseA_TOP 9
#define OUTPUT_PhaseB_TOP 10
#define OUTPUT_PhaseC_TOP 11

#define DISPLAY_UPDATE_FREQ_IN_MS 200
#define DISPLAY_BLANKINGIME_IN_MS 60 // For updating ~4 digits, blank them for this time before update.
#define DISPLAY_UPDATE_DELAY_CHECK_IN_MS (DISPLAY_UPDATE_FREQ_IN_MS-DISPLAY_BLANKINGIME_IN_MS)

// ----------------------------------------------------------------------
// LCD Stuff
// Find your address from I2C Scanner function and add it here:
#define LCD_I2C 0x3F
#define LCD_BACKLIGHT_PIN 3
#define LCD_En_pin  2
#define LCD_Rw_pin  1
#define LCD_Rs_pin  0
#define LCD_D4_pin  4
#define LCD_D5_pin  5
#define LCD_D6_pin  6
#define LCD_D7_pin  7
LiquidCrystal_I2C  lcd(LCD_I2C,LCD_En_pin,LCD_Rw_pin,LCD_Rs_pin,LCD_D4_pin,LCD_D5_pin,LCD_D6_pin,LCD_D7_pin);


// ----------------------------------------------------------------------
// Main Logic Data Structures:

typedef struct {
    int  powerLevel;
    long delay_in_us;
    int  curPhase;       // 1 to 6
} CONTROLCONTEXT_ST;

    
static CONTROLCONTEXT_ST controlContext = {0, 500000, 1};


static long prevDelay_in_us = controlContext.delay_in_us;
static int  prevPowerLevel = controlContext.powerLevel;

// 180/5 = 36 entries:
static unsigned char sinLookup[] = {0, 22, 44, 66, 87, 108, 128, 146, 164, 180, 195, 209, 221, 231, 240, 246, 251, 254, 255, 254, 251, 246, 240, 231, 221, 209, 195, 180, 164, 146, 128, 108, 87, 66, 44, 22 };
static int lookupSize = sizeof(sinLookup);
static int lookupIdx = 0;


// ----------------------------------------------------------------------


void setup() {
    // ----------------------------------------------------------------------
    // Change the frequency of the PWM output for pins 6, 7, and 8.
    // These pins are controlled byt the TCCR4 clock and the frequency is configurable
    // using its register: TCCR4B - Timer/Counter 4 Control Register B
    // The bits of interest are 2, 1, 0 which are defined as CS42, CS41, CS40 respectively.
    // The value to have no pre-scaler for these bits is 001.
    // A test showed this resulted in 32kHz.
    TCCR4B = (TCCR4B & 0b11111000) | 0x01;
    // ----------------------------------------------------------------------
    
    // Debug:
    Serial.begin(9600);
    Serial.println("Starting!!!");


    // Setup PWM outputs:
    pinMode(OUTPUT_PhaseA_TOP, OUTPUT);
    pinMode(OUTPUT_PhaseB_TOP, OUTPUT);
    pinMode(OUTPUT_PhaseC_TOP, OUTPUT);
    pinMode(OUTPUT_PhaseA_BOT, OUTPUT);
    pinMode(OUTPUT_PhaseB_BOT, OUTPUT);
    pinMode(OUTPUT_PhaseC_BOT, OUTPUT);
    digitalWrite(OUTPUT_PhaseA_TOP, HIGH);
    digitalWrite(OUTPUT_PhaseB_TOP, HIGH);
    digitalWrite(OUTPUT_PhaseC_TOP, HIGH);
    analogWrite(OUTPUT_PhaseA_BOT, 0);
    analogWrite(OUTPUT_PhaseB_BOT, 0);
    analogWrite(OUTPUT_PhaseC_BOT, 0);

    // Setup POT outputs:
    pinMode(50,OUTPUT);
    pinMode(51,OUTPUT);
    pinMode(52,OUTPUT);
    pinMode(53,OUTPUT);
    digitalWrite(50,LOW);
    digitalWrite(51,HIGH);
    digitalWrite(52,LOW);
    digitalWrite(53,HIGH);

    // Setup LCD:
    lcd.begin (20,4);
    lcd.setBacklightPin(LCD_BACKLIGHT_PIN,POSITIVE);
    lcd.setBacklight(HIGH);
    lcd.home (); // go home
    lcd.print("BLDC Control");
    lcd.setCursor(0,1);
    lcd.print("Initializing..."); 
    delay(3000);
    lcdReset();

    Timer1.initialize(controlContext.delay_in_us/lookupSize);         // initialize timer1, and set a 1/2 second period
    Timer1.attachInterrupt(timer_callback);  // attaches timer_callback() as a timer overflow interrupt
 
}

void lcdReset() {
    lcd.clear();
    lcd.home();
    // lcd.print("(X:");lcd.print(PacketsRX[1]);lcd.print(",Y:");lcd.print(PacketsRX[2]);lcd.print(") sum=");lcd.print(PacketsRX[0]);

    Serial.println("LCD Reset!!!");
    char line[41];

    lcd.clear();
    lcd.home();
    lcd.setCursor(0,0);
    sprintf(line, "Power:     [0-255]");
    lcd.print(line);

    lcd.setCursor(0,1);
    sprintf(line, "Delay:     [ms]");
    lcd.print(line);

    lcd.setCursor(0,2);
    sprintf(line, "Phase:     ");
    lcd.print(line);
}


static CONTROLCONTEXT_ST dispPrevContext = controlContext;
static CONTROLCONTEXT_ST dispPendingContext = controlContext;
static bool pendingUpdate = false;

// Blank out parts of display that need to change, then delay...
void UpdateDisplayBlankValues(CONTROLCONTEXT_ST & newContext) {
    char * text = "    ";

    if (newContext.powerLevel != dispPrevContext.powerLevel) {
        lcd.setCursor(6,0);
        lcd.print(text);
        pendingUpdate = true;
    }

    if (newContext.delay_in_us != dispPrevContext.delay_in_us) {
        lcd.setCursor(6,1);
        lcd.print(text);
        pendingUpdate = true;
    }

    if (newContext.curPhase != dispPrevContext.curPhase) {
        lcd.setCursor(6,2);
        lcd.print(text);
        pendingUpdate = true;
    }

    if (pendingUpdate) dispPendingContext = newContext;
}

// Update parts of display that have pending change...
void UpdateDisplay() {
    char text[20];

    if (dispPendingContext.powerLevel != dispPrevContext.powerLevel) {
        lcd.setCursor(6,0);
        sprintf(text, "%4d", dispPendingContext.powerLevel);
        lcd.print(text);
    }

    if (dispPendingContext.delay_in_us != dispPrevContext.delay_in_us) {
        lcd.setCursor(6,1);
        if (dispPendingContext.delay_in_us > 10000) {
            sprintf(text, "%4d [ms]", dispPendingContext.delay_in_us/1000);
        }
        else {
            sprintf(text, "%4d [us]", dispPendingContext.delay_in_us);
        }
        lcd.print(text);
    }

    if (dispPendingContext.curPhase != dispPrevContext.curPhase) {
        lcd.setCursor(6,2);
        sprintf(text, "%4d", dispPendingContext.curPhase);
        lcd.print(text);
    }

    dispPrevContext = dispPendingContext;
    pendingUpdate = false;
}

void adjustOutputs() {
    long curPower = ((long) controlContext.powerLevel) * ((long) sinLookup[lookupIdx]) / 255;

    if (controlContext.curPhase < 6) controlContext.curPhase++;
    else                             controlContext.curPhase = 1;

    // Sequence is:
    //   1) A_TOP, C_BOT
    //   2) B_TOP, C_BOT
    //   3) B_TOP, A_BOT
    //   4) C_TOP, A_BOT
    //   5) C_TOP, B_BOT
    //   6) A_TOP, B_BOT

    switch(controlContext.curPhase) {
    case 1:
        analogWrite (OUTPUT_PhaseB_BOT, 0);
        analogWrite (OUTPUT_PhaseC_BOT, curPower);
        break;
    case 2:
        digitalWrite(OUTPUT_PhaseA_TOP, HIGH);
        digitalWrite(OUTPUT_PhaseB_TOP, LOW);
        break;
    case 3:
        analogWrite (OUTPUT_PhaseC_BOT, 0);
        analogWrite (OUTPUT_PhaseA_BOT, curPower);
        break;
    case 4:
        digitalWrite(OUTPUT_PhaseB_TOP, HIGH);
        digitalWrite(OUTPUT_PhaseC_TOP, LOW);
        break;
    case 5:
        analogWrite (OUTPUT_PhaseA_BOT, 0);
        analogWrite (OUTPUT_PhaseB_BOT, curPower);
        break;
    case 6:
        digitalWrite(OUTPUT_PhaseC_TOP, HIGH);
        digitalWrite(OUTPUT_PhaseA_TOP, LOW);
        break;
    }

    return;
}


void timer_callback()
{
    lookupIdx++;

    if (lookupIdx == lookupSize) {
        lookupIdx = 0;
        adjustOutputs();
    }
    else {
        // Adjust power for new sin index:
        adjustPower();
    }

    // Adjust period if needed:
    if (prevDelay_in_us != controlContext.delay_in_us) {
        Timer1.setPeriod(controlContext.delay_in_us/lookupSize);
        prevDelay_in_us = controlContext.delay_in_us;
    }
}
 
void adjustPower() {
    long curPower = ((long) controlContext.powerLevel) * ((long) sinLookup[lookupIdx]) / 255;
    switch(controlContext.curPhase) {
    case 1:
    case 2:
        analogWrite (OUTPUT_PhaseC_BOT, curPower);
        break;
    case 3:
    case 4:
        analogWrite (OUTPUT_PhaseA_BOT, curPower);
        break;
    case 5:
    case 6:
        analogWrite (OUTPUT_PhaseB_BOT, curPower);
        break;
    }

    return;
}


void loop()
{
    unsigned long currentTime = millis();
    static bool firstPotRead = false;
    static unsigned long prevReadTime = 0;
    static unsigned long prevDispTime = 0;
    static unsigned long prevPhaseTime = 0;
 
    // Read Pots:
    long pot1 = 0;
    long pot2 = 0;

    if ((currentTime - prevReadTime) == 100) {
        if (!firstPotRead) {
            pot1 = analogRead(A15);
            controlContext.powerLevel = pot1/4;
            firstPotRead = true;
        }
    }
    else if ((currentTime - prevReadTime) > 200) {
        pot2 = analogRead(A14);
        controlContext.delay_in_us = 3000 + pot2*pot2;
        prevReadTime = currentTime;
        firstPotRead = false;
    }

    if (prevPowerLevel != controlContext.powerLevel) {
        noInterrupts();
        adjustPower();
        interrupts();
        prevPowerLevel = controlContext.powerLevel;
    }

    // Update display/debug:
    if (pendingUpdate) {
        // Write the pending update if delay has passed:
        if ((currentTime - prevDispTime) > DISPLAY_BLANKINGIME_IN_MS) {
            UpdateDisplay();
            prevDispTime = currentTime;
        }
    }
    else {
        // See if there is a change to get displayed:
        if ((currentTime - prevDispTime) > DISPLAY_UPDATE_DELAY_CHECK_IN_MS) {
            UpdateDisplayBlankValues(controlContext);
            prevDispTime = currentTime;
        }
    }
    
    // Debug info...
    // char buffer[200];
    // sprintf(buffer, "Power:%d, Delay:%d\n", controlContext.powerLevel, controlContext.delay_in_us);
    // Serial.print(buffer);
}


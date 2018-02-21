#include <LCD.h>
#include <LiquidCrystal_I2C.h>


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


typedef struct {
    int powerLevel;
    int delay_in_ms;
} CONTROLCONTEXT_ST;

    
static CONTROLCONTEXT_ST controlContext = {0, 500};


void setup() {

    // Debug:
    Serial.begin(9600);
    Serial.println("Starting!!!");

    for (int i=0; i<8; i++) {
        pinMode(i+22,OUTPUT);
    }
    lcd.begin (20,4);
    lcd.setBacklightPin(LCD_BACKLIGHT_PIN,POSITIVE);
    lcd.setBacklight(HIGH);
    lcd.home (); // go home
    lcd.print("BLDC Control");
    lcd.setCursor(0,1);
    lcd.print("Initializing..."); 
    delay(3000);
    allStop(); 
    lcdReset();
    Serial.println("LCD Reset!!!");
}

void lcdReset() {
    lcd.clear();
    lcd.home();
    // lcd.print("(X:");lcd.print(PacketsRX[1]);lcd.print(",Y:");lcd.print(PacketsRX[2]);lcd.print(") sum=");lcd.print(PacketsRX[0]);
}

void allStop() {
    // Serial.print('\n'); Serial.write("All STOP +++++");
    lcd.setCursor(12,3);
    lcd.print("All Stop");
}

void UpdateDisplay(CONTROLCONTEXT_ST & controlContext) {
    // lcd.setCursor(0,3);
    // TODO:: ensure not too much com. to LCD (update every 1 second)
    // lcd.print(controlContext.ctlParms.);
    //
    char line[41];

    lcd.clear();
    lcd.home();
    lcd.setCursor(0,0);
    sprintf(line, "Power:%4d", controlContext.powerLevel);
    lcd.print(line);

    lcd.setCursor(0,1);
    sprintf(line, "Delay:%4d", controlContext.delay_in_ms);
    lcd.print(line);
}

void loop()
{
    static unsigned long currentTime = millis();
    static unsigned long prevDispTime = 0;

    if ((currentTime - prevDispTime) > 400) {
        // Debug info...
        char buffer[200];
        sprintf(buffer, "Power:%d, Delay:%d\n", controlContext.powerLevel, controlContext.delay_in_ms);
        Serial.print(buffer);

        UpdateDisplay(controlContext);
        prevDispTime = currentTime;
    }

    // delay(100);
}


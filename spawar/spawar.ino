#include <U8glib.h>
#include "pin_map.h"
#include <Keypad.h>

#pragma region Keypad Section
#define KPD_1ST_ROW_PIN    SERVO0_PIN
#define KPD_2ND_ROW_PIN    SERVO1_PIN
#define KPD_3RD_ROW_PIN    SERVO2_PIN
#define KPD_4TH_ROW_PIN    SERVO3_PIN
#define KPD_1ST_COL_PIN    63
#define KPD_2ND_COL_PIN    42
#define KPD_3RD_COL_PIN    44
bool keyboard_last;
const int ROW_NUM = 4;
const int COLUMN_NUM = 3;
char keys[ROW_NUM][COLUMN_NUM] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
byte pin_rows[ROW_NUM] = { KPD_1ST_ROW_PIN, KPD_2ND_ROW_PIN, KPD_3RD_ROW_PIN, KPD_4TH_ROW_PIN };
byte pin_column[COLUMN_NUM] = { KPD_1ST_COL_PIN, KPD_2ND_COL_PIN, KPD_3RD_COL_PIN };
Keypad keypad = Keypad(makeKeymap(keys), pin_rows, pin_column, ROW_NUM, COLUMN_NUM);
#pragma endregion

#pragma region Display and Encoder Section
//Standard pins when on a RAMPS 1.4
#define DOGLCD_CS       16
#define DOGLCD_MOSI     17
#define DOGLCD_SCK      23
#define BTN_EN1         31
#define BTN_EN2         33
#define BTN_ENC         35
#define SD_DETECT_PIN   49
#define SDSS            53
#define BEEPER_PIN      37
#define KILL_PIN        41

int encoderPos = 1;                     //Current encoder position
int encoder0PinALast;                   //Used to decode rotory encoder, last value
char str15[15];
char str70[70];
char str10[10];
char tmp_string[16];
uint8_t enc_btn_last = HIGH;                     //Last read status of the encoder button
#define LINE_HEIGHT     10
U8GLIB_ST7920_128X64_1X u8g(DOGLCD_SCK, DOGLCD_MOSI, DOGLCD_CS);
bool screenUpdateNeeded;
#pragma endregion

#pragma region Stepper Section

#define STEPPER_STEP_PIN    E0_STEP_PIN
#define STEPPER_DIR_PIN     E0_DIR_PIN
#define STEPPER_ENABLE_PIN  E0_ENABLE_PIN 
#define POWERSWITCH_PIN     40
#define DIRECTIONSWITCH_RIGHT_PIN 64
#define DIRECTIONSWITCH_LEFT_PIN 59

#define GEAR_FACTOR 8.25
#define STEPS_PER_TURN 6400

bool powerOn;
bool reverseDirection;

unsigned long fixedDelay;
unsigned long stepDelay = 10;

#pragma endregion

 
double angularVelocity=3;     //prêdkoœæ k¹towa w obr/min
double diameter = 80;            //œrednica w mm
double weldingSpeed=80;           //prêdkoœæ liniowa spawania w mm/min
double circuit;                          //obwód w mm;

String  angularVelocityStr = String(angularVelocity);
String  diameterStr = String(diameter);
String  weldingSpeedStr = String(weldingSpeed);

#define MAX_VALUE_STR_LENGTH    6
#define HARDWARE_DELAY_MS   12

uint8_t currentOption = 1;
bool isEditing;

void setup() { 
    pinMode(STEPPER_STEP_PIN, OUTPUT);
    pinMode(STEPPER_DIR_PIN, OUTPUT);
    digitalWrite(STEPPER_DIR_PIN, HIGH);
    pinMode(POWERSWITCH_PIN, INPUT_PULLUP);
    pinMode(DIRECTIONSWITCH_RIGHT_PIN, INPUT_PULLUP);
    pinMode(DIRECTIONSWITCH_LEFT_PIN, INPUT_PULLUP);
    pinMode(KILL_PIN, INPUT);
    digitalWrite(KILL_PIN, HIGH);
    pinMode(BTN_EN1, INPUT);
    digitalWrite(BTN_EN1, HIGH);
    pinMode(BTN_EN2, INPUT);
    digitalWrite(BTN_EN2, HIGH);
    pinMode(BTN_ENC, INPUT);
    digitalWrite(BTN_ENC, HIGH);
    u8g.setFont(u8g_font_helvR08);
    u8g.setColorIndex(1);                 
    calcAngularVelocity();
}

//Main arduino loop
void loop() {
    
    if (digitalRead(POWERSWITCH_PIN) == LOW) { 
        if (!powerOn) 
        {
            powerOn = true;
            updateScreen();
        }     
    }
    else{
        if (powerOn)
        {
            powerOn = false;
            screenUpdateNeeded = true;
        }
    }
    if (powerOn) {
        digitalWrite(STEPPER_STEP_PIN, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(STEPPER_STEP_PIN, LOW);
        delayMicroseconds(fixedDelay);
        return;
    }

    if (digitalRead(DIRECTIONSWITCH_RIGHT_PIN) == LOW) {
        if (!reverseDirection)
        {
            digitalWrite(STEPPER_DIR_PIN, LOW);
            reverseDirection = true;
            screenUpdateNeeded = true;
        }
    }
    else if(digitalRead(DIRECTIONSWITCH_LEFT_PIN) == LOW) {
        if (reverseDirection)
        {
            digitalWrite(STEPPER_DIR_PIN, HIGH);
            reverseDirection = false;
            screenUpdateNeeded = true;
        }
    }

    char key = keypad.getKey();
    int enc = encoderRead();
    bool encBtn = encoderButtonClicked();
    if (encBtn) 
    {
        isEditing = !isEditing;
        screenUpdateNeeded = true;
    }
    if (key && isEditing) 
    {
        if (currentOption == 1) 
        {
            diameter = applySignToValue(&diameterStr, key);
            calcAngularVelocity();
        }
        if (currentOption == 2)
        { 
            weldingSpeed = applySignToValue(&weldingSpeedStr, key);
            calcAngularVelocity();
        }
        if (currentOption == 3) {
            angularVelocity = applySignToValue(&angularVelocityStr, key);
            calcWeldingSpeed();
            calculateStepDelay();
        }
        screenUpdateNeeded = true;
    }
    else if (enc && !isEditing)
    {
        screenUpdateNeeded = true;
        currentOption += enc;
        currentOption = currentOption == 0 ? 1 : currentOption == 4 ? 3 : currentOption;
    }
    
    if (screenUpdateNeeded)updateScreen();
}

void updateScreen() {  //clear and update screen with 
    u8g.nextPage();
    u8g.firstPage();
    do {
        drawMain();
    } while (u8g.nextPage());
    screenUpdateNeeded = false;
}

int encoderRead() {       //read move from encoder, 1 - forward, -1 - backward, 0 - no move
    int encoder0PinNow = digitalRead(BTN_EN1);
    if ((encoder0PinALast == LOW) && (encoder0PinNow == HIGH)) 
    {
        encoder0PinALast = encoder0PinNow;
        if (digitalRead(BTN_EN2) == LOW)
        {
            return 1;
        }
        else
        {
            return -1;
        }
    }
    encoder0PinALast = encoder0PinNow;
    return 0;
}



void drawMain() {               //draw main screen with u8g library
    for (uint8_t i = 1; i <= 3; i++) 
    {
        u8g.setDefaultForegroundColor();
        if (currentOption == i) { 
            if (isEditing)
            {
                u8g.drawBox(0, (i - 1) * LINE_HEIGHT + 1, 128, LINE_HEIGHT);
                u8g.setDefaultBackgroundColor();
            }
            u8g.drawTriangle(0, i * LINE_HEIGHT - 1, 0, (i - 1) * LINE_HEIGHT + 1, 7, i * LINE_HEIGHT - LINE_HEIGHT / 2);
        }
        switch (i)
        {
        case 1:
            sprintf(str15, "Srednica:");
            sprintf(str70, "%smm", diameterStr.c_str());
            u8g.drawStr(8, 10, str15);
            u8g.drawStr(67, 10, str70);
            break;
        case 2:
            sprintf(str15, "V spawania:");
            sprintf(str70, "%smm/min", weldingSpeedStr.c_str());
            u8g.drawStr(8, 20, str15);
            u8g.drawStr(67, 20, str70);
            break;
        case 3:
            sprintf(str15, "V katowa:");
            sprintf(str70, "%sobr/min", angularVelocityStr.c_str());
            u8g.drawStr(8, 30, str15);
            u8g.drawStr(67, 30, str70);
            break;
        default:
            break;
        }
    }
    if(powerOn)sprintf(str15, "powerON");
    else sprintf(str15, "powerOFF");
    u8g.drawStr(0, 50, str15);

    if (!reverseDirection)sprintf(str15, "Obroty: PRAWE");
    else sprintf(str15, "Obroty: LEWE");
    u8g.drawStr(0, 60, str15);
}

void calcAngularVelocity() {        //calculate angular velocity with given diameter and welding speed
    circuit = PI * diameter;
    angularVelocity = weldingSpeed / circuit;
    angularVelocityStr = String(angularVelocity);
    calculateStepDelay();
}

void calcWeldingSpeed() {           //calculate welding speed with given diameter and angular velocity
    circuit = PI * diameter;
    weldingSpeed = angularVelocity * circuit;
    weldingSpeedStr = String(weldingSpeed);
}

bool encoderButtonClicked() {       //get true if encoder button was clicked
    uint8_t enc_btn_now = digitalRead(BTN_ENC);
    if (enc_btn_now && !enc_btn_last) {
        enc_btn_last = enc_btn_now;
        return true;
    }
    enc_btn_last = enc_btn_now;
    return false;
}

double applySignToValue(String* value, char key) {           //perform operations with the given sign for the given number
    if (key == '#')value->remove(value->length() - 1);
	else if (value->length() <= MAX_VALUE_STR_LENGTH) {
		if (key == '*') {
			if (value->indexOf('.') == -1) {
				(*value) += '.';
			}
		}
		else {
			(*value) += key;
		}
	}
	return value->toDouble();
}

void calculateStepDelay() {
    stepDelay = 30000000.0 / (angularVelocity * GEAR_FACTOR * STEPS_PER_TURN);
    if (stepDelay < HARDWARE_DELAY_MS)fixedDelay = 0;
    else fixedDelay = stepDelay - HARDWARE_DELAY_MS;
}
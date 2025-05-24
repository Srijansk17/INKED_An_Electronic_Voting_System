#include <LiquidCrystal.h>

// Define LCD pins
#define RS 19
#define EN 23
#define D4 18
#define D5 17
#define D6 16
#define D7 15

// Define button pins
#define BTN1 32
#define BTN2 33
#define BTN3 25
#define BTN4 26
#define BTN5 27

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

int voteCount = 0;

void setup() {
    Serial.begin(115200);
    lcd.begin(16, 2);
    lcd.print("Voting System");
    delay(2000);
    lcd.clear();
    
    pinMode(BTN1, INPUT_PULLDOWN);
    pinMode(BTN2, INPUT_PULLDOWN);
    pinMode(BTN3, INPUT_PULLDOWN);
    pinMode(BTN4, INPUT_PULLDOWN);
    pinMode(BTN5, INPUT_PULLDOWN);
    
    lcd.setCursor(0, 0);
    lcd.print("EVM Ready");
    Serial.println("EVM Ready");
}

void loop() {
    int votedCandidate = 0;
    if (digitalRead(BTN1) == HIGH) votedCandidate = 1;
    if (digitalRead(BTN2) == HIGH) votedCandidate = 2;
    if (digitalRead(BTN3) == HIGH) votedCandidate = 3;
    if (digitalRead(BTN4) == HIGH) votedCandidate = 4;
    if (digitalRead(BTN5) == HIGH) votedCandidate = 5;
    
    if (votedCandidate > 0) {
        voteCount++;
        displayVote(votedCandidate);
        Serial.print("");
        Serial.print(voteCount);
        Serial.print(",");
        Serial.println(votedCandidate);
        delay(500); // Debounce delay
        while (digitalRead(BTN1) == HIGH || digitalRead(BTN2) == HIGH || digitalRead(BTN3) == HIGH || digitalRead(BTN4) == HIGH || digitalRead(BTN5) == HIGH);
        delay(500);
        
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Waiting..");
        delay(5000);
        
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("EVM Ready");
    }
}

void displayVote(int candidate) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Vote ");
    lcd.print(voteCount);
    lcd.setCursor(0, 1);
    lcd.print("Candidate ");
    lcd.print(candidate);
}

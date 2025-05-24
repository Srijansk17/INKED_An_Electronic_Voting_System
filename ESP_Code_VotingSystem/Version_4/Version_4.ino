// ESP32 Arduino Code (Version_4.ino) - Modified for 4-bit parallel LCD

// --- IMPORTANT: Replace these pin numbers with your actual connections ---
// LiquidCrystal display pins (adjust as per your wiring)
// rs, en, d4, d5, d6, d7
#define LCD_RS 23 // Example GPIO pin for Register Select
#define LCD_EN 22 // Example GPIO pin for Enable
#define LCD_D4 19 // Example GPIO pin for Data 4
#define LCD_D5 18 // Example GPIO pin for Data 5
#define LCD_D6 17 // Example GPIO pin for Data 6
#define LCD_D7 16 // Example GPIO pin for Data 7

// You will need this library for character LCDs
#include <LiquidCrystal.h>

// Initialize the LiquidCrystal library with the numbers of the interface pins
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// Button pins (adjust as per your wiring)
#define CANDIDATE1_PIN 13 // Example GPIO pin for Candidate 1
#define CANDIDATE2_PIN 12 // Example GPIO pin for Candidate 2
#define CANDIDATE3_PIN 14 // Example GPIO pin for Candidate 3
#define CANDIDATE4_PIN 27 // Example GPIO pin for Candidate 4
#define CANDIDATE5_PIN 26 // Example GPIO pin for Candidate 5

// EVM States
enum EVMState {
    BOOTING,
    WAITING_FOR_PC_READY, // PC is ready, but EVM hasn't confirmed voting slot open
    VOTER_AUTH_READY,     // PC has authorized, EVM is ready for vote input
    VOTE_CONFIRMED,       // EVM has received a vote, waiting for PC acknowledgment
    VOTE_FINALIZED_OK,    // PC acknowledged vote as OK
    VOTE_FINALIZED_ERROR, // PC acknowledged vote as ERROR
    ERROR_STATE
};

EVMState currentEVMState = BOOTING;

// --- Setup ---
void setup() {
    Serial.begin(115200); // Start serial communication at 115200 baud
    
    // Set up the LCD's number of columns and rows:
    lcd.begin(16, 2); // Assuming a 16x2 LCD. Change to 20,4 for 20x4 LCD.
    
    // Configure button pins as inputs with pull-up resistors
    pinMode(CANDIDATE1_PIN, INPUT_PULLUP);
    pinMode(CANDIDATE2_PIN, INPUT_PULLUP);
    pinMode(CANDIDATE3_PIN, INPUT_PULLUP);
    pinMode(CANDIDATE4_PIN, INPUT_PULLUP);
    pinMode(CANDIDATE5_PIN, INPUT_PULLUP);

    Serial.println("EVM_READY"); // Signal to PC that EVM is booted and ready
    currentEVMState = WAITING_FOR_PC_READY;
    displayBootMessage();
}

// --- Loop ---
void loop() {
    handleSerialCommunication(); // Always check for PC commands

    switch (currentEVMState) {
        case BOOTING:
            // Should transition to WAITING_FOR_PC_READY in setup()
            break;
        
        case WAITING_FOR_PC_READY:
            displayWaitingForPC();
            break;

        case VOTER_AUTH_READY:
            handleVoteInput(); // Check for button presses
            break;

        case VOTE_CONFIRMED:
            displayVoteSent();
            // Stays in this state until ACK:VOTE_OK or ACK:VOTE_ERROR is received from PC
            break;

        case VOTE_FINALIZED_OK:
            displayVoteRecordedOK();
            delay(2000); // Show success message for 2 seconds
            currentEVMState = WAITING_FOR_PC_READY; // Go back to waiting for next authorization
            break;
            
        case VOTE_FINALIZED_ERROR:
            displayVoteRecordedError();
            delay(2000); // Show error message for 2 seconds
            currentEVMState = WAITING_FOR_PC_READY; // Go back to waiting for next authorization
            break;

        case ERROR_STATE:
            displayError();
            // Potentially attempt to re-establish connection or allow manual reset
            break;
    }
}

// --- Serial Communication Handlers (ESP32 Side) ---
void handleSerialCommunication() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim(); // Remove any whitespace

        Serial.print("EVM Received: ");
        Serial.println(command); // Echo received command for debugging

        if (command == "PC_READY") {
            Serial.println("EVM:VOTER_SLOT_OPEN"); // Confirm to PC that EVM is ready for a vote
            currentEVMState = VOTER_AUTH_READY;
            displayReadyToVote();
        } else if (command == "ACK:VOTE_OK") {
            currentEVMState = VOTE_FINALIZED_OK;
        } else if (command == "ACK:VOTE_ERROR") {
            currentEVMState = VOTE_FINALIZED_ERROR;
        }
        // Add more commands if needed
    }
}

// --- Vote Input Handling ---
void handleVoteInput() {
    int votedCandidate = -1; // -1 means no vote yet

    if (digitalRead(CANDIDATE1_PIN) == LOW) {
        votedCandidate = 0; // Candidate 0
    } else if (digitalRead(CANDIDATE2_PIN) == LOW) {
        votedCandidate = 1; // Candidate 1
    } else if (digitalRead(CANDIDATE3_PIN) == LOW) {
        votedCandidate = 2; // Candidate 2
    } else if (digitalRead(CANDIDATE4_PIN) == LOW) {
        votedCandidate = 3; // Candidate 3
    } else if (digitalRead(CANDIDATE5_PIN) == LOW) {
        votedCandidate = 4; // Candidate 4
    }

    if (votedCandidate != -1) {
        delay(200); // Debounce delay
        while (digitalRead(CANDIDATE1_PIN) == LOW || digitalRead(CANDIDATE2_PIN) == LOW ||
               digitalRead(CANDIDATE3_PIN) == LOW || digitalRead(CANDIDATE4_PIN) == LOW ||
               digitalRead(CANDIDATE5_PIN) == LOW) {
            // Wait for button release
        }
        
        Serial.print("VOTE:");
        Serial.println(votedCandidate); // Send vote to PC
        currentEVMState = VOTE_CONFIRMED; // EVM waits for PC acknowledgment
        displayVoteSent();
    }
}

// --- Display Functions (Modified for LiquidCrystal) ---
void lcdClearAndSetCursor() {
    lcd.clear();
    lcd.setCursor(0, 0);
}

void displayBootMessage() {
    lcdClearAndSetCursor();
    lcd.print("EVM Initializing...");
}

void displayWaitingForPC() {
    lcdClearAndSetCursor();
    lcd.print("Waiting for PC");
    lcd.setCursor(0,1); // Go to second line
    lcd.print("To Start Voting...");
}

void displayReadyToVote() {
    lcdClearAndSetCursor();
    lcd.print("Please Vote");
    lcd.setCursor(0,1); // Go to second line
    lcd.print("Select Candidate");
}

void displayVoteSent() {
    lcdClearAndSetCursor();
    lcd.print("Vote Sent!");
    lcd.setCursor(0,1); // Go to second line
    lcd.print("Waiting for PC...");
}

void displayVoteRecordedOK() {
    lcdClearAndSetCursor();
    lcd.print("Vote Recorded!");
    lcd.setCursor(0,1); // Go to second line
    lcd.print("Thank You.");
}

void displayVoteRecordedError() {
    lcdClearAndSetCursor();
    lcd.print("Vote Error!");
    lcd.setCursor(0,1); // Go to second line
    lcd.print("Try Again.");
}

void displayError() {
    lcdClearAndSetCursor();
    lcd.print("EVM Error!");
    lcd.setCursor(0,1); // Go to second line
    lcd.print("Check Wiring.");
}
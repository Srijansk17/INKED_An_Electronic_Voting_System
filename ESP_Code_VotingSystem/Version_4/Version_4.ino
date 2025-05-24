// ESP32 Arduino Code (e.g., Version_3.ino)

#include <Wire.h>
  #include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED definitions
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Button pins (adjust as per your wiring)
#define CANDIDATE1_PIN 13 // Example GPIO pin for Candidate 1
#define CANDIDATE2_PIN 12 // Example GPIO pin for Candidate 2
#define CANDIDATE3_PIN 14 // Example GPIO pin for Candidate 2
#define CANDIDATE4_PIN 27 // Example GPIO pin for Candidate 2
#define CANDIDATE5_PIN 26 // Example GPIO pin for Candidate 2

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
    
    // Initialize OLED display
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
        Serial.println(F("SSD1306 allocation failed"));
        for (;;); // Don't proceed, loop forever
    }
    display.display(); // Clear display buffer
    delay(2000); // Small delay to let display power up
    display.clearDisplay();

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

// --- Display Functions ---
void displayClearAndSetCursor() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
}

void displayBootMessage() {
    displayClearAndSetCursor();
    display.println("EVM Initializing...");
    display.display();
}

void displayWaitingForPC() {
    displayClearAndSetCursor();
    display.println("Waiting for PC");
    display.println("To Start Voting...");
    display.display();
}

void displayReadyToVote() {
    displayClearAndSetCursor();
    display.setTextSize(2);
    display.println("Please Vote");
    display.setTextSize(1);
    display.println("Select Candidate");
    display.display();
}

void displayVoteSent() {
    displayClearAndSetCursor();
    display.setTextSize(2);
    display.println("Vote Sent!");
    display.setTextSize(1);
    display.println("Waiting for PC...");
    display.display();
}

void displayVoteRecordedOK() {
    displayClearAndSetCursor();
    display.setTextSize(2);
    display.println("Vote Recorded!");
    display.setTextSize(1);
    display.println("Thank You.");
    display.display();
}

void displayVoteRecordedError() {
    displayClearAndSetCursor();
    display.setTextSize(2);
    display.println("Vote Error!");
    display.setTextSize(1);
    display.println("Try Again.");
    display.display();
}

void displayError() {
    displayClearAndSetCursor();
    display.println("EVM Error!");
    display.println("Check Connection.");
    display.display();
}
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

// State variable for the EVM
enum EVM_STATE {
    WAITING_FOR_VOTER,
    VOTE_PENDING,
    VOTE_CONFIRMED,
    ERROR_STATE
};

EVM_STATE currentEVMState = WAITING_FOR_VOTER;
int currentCandidateVote = 0;
unsigned long lastButtonPressTime = 0;
const unsigned long DEBOUNCE_DELAY = 100; // milliseconds
const unsigned long VOTE_LOCKOUT_DELAY = 5000; // 5 seconds to prevent rapid double votes

void setup() {
    Serial.begin(115200);
    lcd.begin(16, 2);
    
    // Clear initial LCD message after setup
    lcd.clear(); 
    lcd.setCursor(0, 0);
    lcd.print("EVM Initializing...");
    delay(2000);
    lcd.clear();

    pinMode(BTN1, INPUT_PULLDOWN);
    pinMode(BTN2, INPUT_PULLDOWN);
    pinMode(BTN3, INPUT_PULLDOWN);
    pinMode(BTN4, INPUT_PULLDOWN);
    pinMode(BTN5, INPUT_PULLDOWN);
    
    Serial.println("EVM_READY"); // Signal to PC that EVM is online and ready
    lcd.setCursor(0, 0);
    lcd.print("Waiting for PC");
    lcd.setCursor(0, 1);
    lcd.print("To Start Voting");
    currentEVMState = WAITING_FOR_VOTER; // Set initial state
}

void loop() {
    handleSerialCommunication(); // Check for commands from PC

    switch (currentEVMState) {
        case WAITING_FOR_VOTER:
            // Display message: "Waiting for PC to authorize next voter"
            // Handled in setup for now, but could be dynamic
            break;

        case VOTE_PENDING:
            // Display "Please Vote"
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Please Vote");
            lcd.setCursor(0, 1);
            lcd.print("Select Candidate");

            int votedCandidate = 0;
            // Debounce logic: only read button if enough time has passed since last press
            if ((millis() - lastButtonPressTime) > DEBOUNCE_DELAY) {
                if (digitalRead(BTN1) == HIGH) votedCandidate = 1;
                else if (digitalRead(BTN2) == HIGH) votedCandidate = 2;
                else if (digitalRead(BTN3) == HIGH) votedCandidate = 3;
                else if (digitalRead(BTN4) == HIGH) votedCandidate = 4;
                else if (digitalRead(BTN5) == HIGH) votedCandidate = 5;

                if (votedCandidate > 0) {
                    currentCandidateVote = votedCandidate;
                    displayVote(currentCandidateVote); // Show vote on LCD

                    // Send vote data to PC
                    // Format: VOTE:<Candidate_ID>\n
                    // The PC will assign the Voter_ID based on its session management
                    Serial.print("VOTE:");
                    Serial.println(currentCandidateVote);
                    
                    // Immediately transition to VOTE_CONFIRMED and wait for PC ACK
                    currentEVMState = VOTE_CONFIRMED;
                    lcd.clear();
                    lcd.setCursor(0, 0);
                    lcd.print("Vote Sent!");
                    lcd.setCursor(0, 1);
                    lcd.print("Waiting for PC");
                }
            }
            break;

        case VOTE_CONFIRMED:
            // Waiting for "ACK:VOTE_OK" or "ACK:ERROR" from PC
            // This state is managed by handleSerialCommunication()
            break;
        
        case ERROR_STATE:
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("EVM Error!");
            lcd.setCursor(0, 1);
            lcd.print("Restart PC App");
            // Optionally, try to re-initialize or signal PC for help
            break;
    }
}

void displayVote(int candidate) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("You voted for");
    lcd.setCursor(0, 1);
    lcd.print("Candidate ");
    lcd.print(candidate);
}

void handleSerialCommunication() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim(); // Remove any whitespace

        if (command == "PC_READY") {
            // PC is ready, allow a voter to vote
            if (currentEVMState == WAITING_FOR_VOTER || currentEVMState == VOTE_CONFIRMED) {
                currentEVMState = VOTE_PENDING;
                lastButtonPressTime = millis(); // Reset debounce timer
                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("Voter Ready!");
                lcd.setCursor(0, 1);
                lcd.print("Please Vote");
                Serial.println("EVM:VOTER_SLOT_OPEN"); // Confirm to PC that EVM is ready for vote
            }
        } else if (command == "ACK:VOTE_OK") {
            // PC confirmed the vote was processed successfully
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Vote Recorded!");
            delay(2000); // Display for 2 seconds
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Waiting for PC");
            lcd.setCursor(0, 1);
            lcd.print("To Start Voting");
            currentEVMState = WAITING_FOR_VOTER; // Go back to waiting for next voter
            Serial.println("EVM_READY"); // Signal back to PC, EVM is back to idle
        } else if (command == "ACK:VOTE_ERROR") {
            // PC reported an error with the vote (e.g., duplicate voter, ineligible)
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Vote Error!");
            lcd.setCursor(0, 1);
            lcd.print("See PC For Details");
            delay(3000); // Display for 3 seconds
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Waiting for PC");
            lcd.setCursor(0, 1);
            lcd.print("To Start Voting");
            currentEVMState = WAITING_FOR_VOTER; // Go back to waiting for next voter
            Serial.println("EVM_READY"); // Signal back to PC, EVM is back to idle
        }
        // You can add more commands here, e.g., "RESET_EVM", "DISPLAY_MESSAGE:..."
    }
}
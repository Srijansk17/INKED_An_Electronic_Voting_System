#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>

// Platform-specific includes for serial communication
#ifdef _WIN32
#include <windows.h> // For HANDLE, DWORD, etc.
#include <process.h> // For _beginthreadex in a real threaded app
#else
#include <termios.h> // For serial port functions
#include <unistd.h>  // For read(), write(), close(), usleep()
#include <fcntl.h>   // For open()
#include <pthread.h> // For pthreads in a real threaded app
#endif

// --- Constants and Global Data Structures ---
#define MAX_NAME_LEN 50
#define MAX_LINE_LEN 256
#define VOTERS_FILE "voters.txt"
#define VOTES_FILE "votes_cast.txt"
#define SERIAL_READ_BUFFER_SIZE 128 // Size for incoming serial data

// Structure to represent a voter
typedef struct {
    int id;
    char name[MAX_NAME_LEN];
    bool is_eligible;
    bool has_voted;
} Voter;

// Structure to represent a candidate
typedef struct {
    char id[10];   // e.g., "CAND_1", "CAND_2"
    char name[MAX_NAME_LEN];
} Candidate;

// Global (or passed around) array/linked list to store voters in memory
Voter *all_voters = NULL;
int num_all_voters = 0;

// Global (or passed around) candidates list
Candidate candidates[] = {
    {"1", "Candidate Alpha"},
    {"2", "Candidate Beta"},
    {"3", "Candidate Gamma"},
    {"4", "Candidate Delta"},
    {"5", "Candidate Epsilon"}
};
int num_candidates = sizeof(candidates) / sizeof(candidates[0]);

// State variables for managing current voting session
// In a real GUI, this would be managed by the UI logic
int current_voter_id_for_evm = -1; // -1 means no voter currently authorized
bool evm_is_ready = false; // Flag for EVM status
bool evm_slot_open = false; // Flag indicating EVM is ready for a vote input

// --- Serial Port Handles/File Descriptors ---
#ifdef _WIN32
HANDLE serial_port_handle = INVALID_HANDLE_VALUE;
#else
int serial_port_fd = -1;
#endif

// --- Function Prototypes ---
// File Handling & Voter Management
void load_voters_from_file();
int compare_voters_by_id(const void *a, const void *b); // Comparison function for qsort/bsearch
Voter* find_voter_by_id(int id);
void register_new_voter(int id, const char *name, bool is_eligible);
void update_voter_status_in_file(int voter_id, bool voted_status);
void list_eligible_voters();

// Vote Recording & Security
void xor_encode_decode(char *data, size_t len, const char *key); // Single function for XOR
void record_vote(int voter_id, const char *candidate_id_str);
void calculate_and_display_results();

// Serial Communication
#ifdef _WIN32
bool open_serial_port(const char *port_name, DWORD baud_rate);
int read_serial_data(char *buffer, int max_len);
bool write_serial_data(const char *data);
void close_serial_port();
#else
bool open_serial_port(const char *port_name, speed_t baud_rate); // Use speed_t for baud_rate
int read_serial_data(char *buffer, int max_len);
bool write_serial_data(const char *data);
void close_serial_port();
#endif
void serial_monitor_loop(); // Runs in a thread for GUI apps
void handle_evm_data(const char *data); // Processes data received from EVM

// --- File Handling & Voter Management Implementations ---

void load_voters_from_file() {
    FILE *fp = fopen(VOTERS_FILE, "r");
    if (!fp) {
        perror("Info: Voters file not found, creating new one.");
        // Create an empty file if it doesn't exist
        fp = fopen(VOTERS_FILE, "w");
        if (fp) fclose(fp);
        return; // No voters to load yet
    }

    if (all_voters) {
        free(all_voters);
        all_voters = NULL;
        num_all_voters = 0;
    }

    char line[MAX_LINE_LEN];
    while (fgets(line, sizeof(line), fp)) {
        all_voters = realloc(all_voters, (num_all_voters + 1) * sizeof(Voter));
        if (!all_voters) {
            perror("Memory allocation failed for voters");
            fclose(fp);
            return;
        }

        Voter new_voter;
        int eligible_int, voted_int; // For reading bools as ints
        if (sscanf(line, "%d,%49[^,],%d,%d",
                   &new_voter.id, new_voter.name, &eligible_int, &voted_int) == 4) {
            new_voter.is_eligible = (bool)eligible_int;
            new_voter.has_voted = (bool)voted_int;
            all_voters[num_all_voters] = new_voter;
            num_all_voters++;
        } else {
            fprintf(stderr, "Warning: Skipping malformed line in %s: %s", VOTERS_FILE, line);
        }
    }
    fclose(fp);

    qsort(all_voters, num_all_voters, sizeof(Voter), compare_voters_by_id);
    printf("Loaded %d voters.\n", num_all_voters);
}

int compare_voters_by_id(const void *a, const void *b) {
    return ((Voter *)a)->id - ((Voter *)b)->id;
}

Voter* find_voter_by_id(int id) {
    if (!all_voters || num_all_voters == 0) {
        return NULL;
    }
    Voter key_voter;
    key_voter.id = id;
    return (Voter*)bsearch(&key_voter, all_voters, num_all_voters, sizeof(Voter), compare_voters_by_id);
}

void register_new_voter(int id, const char *name, bool is_eligible) {
    if (find_voter_by_id(id) != NULL) {
        printf("Error: Voter with ID %d already exists. Cannot register.\n", id);
        return;
    }

    FILE *fp = fopen(VOTERS_FILE, "a");
    if (!fp) {
        perror("Error opening voters file for appending");
        return;
    }
    fprintf(fp, "%d,%s,%d,%d\n", id, name, is_eligible ? 1 : 0, 0); // 0 for has_voted initially
    fclose(fp);
    printf("Voter %d (%s) registered successfully.\n", id, name);
    load_voters_from_file(); // Re-load to update in-memory list
}

void update_voter_status_in_file(int voter_id, bool voted_status) {
    FILE *old_fp = fopen(VOTERS_FILE, "r");
    if (!old_fp) {
        perror("Error opening voters file for reading (update)");
        return;
    }

    FILE *new_fp = fopen("voters_temp.txt", "w");
    if (!new_fp) {
        perror("Error creating temporary voters file");
        fclose(old_fp);
        return;
    }

    char line[MAX_LINE_LEN];
    while (fgets(line, sizeof(line), old_fp)) {
        int id, eligible_flag, voted_flag;
        char name[MAX_NAME_LEN];
        if (sscanf(line, "%d,%49[^,],%d,%d", &id, name, &eligible_flag, &voted_flag) == 4) {
            if (id == voter_id) {
                fprintf(new_fp, "%d,%s,%d,%d\n", id, name, eligible_flag, voted_status ? 1 : 0);
            } else {
                fprintf(new_fp, "%s", line);
            }
        } else {
            fprintf(new_fp, "%s", line);
        }
    }
    fclose(old_fp);
    fclose(new_fp);

    remove(VOTERS_FILE);
    rename("voters_temp.txt", VOTERS_FILE);
    load_voters_from_file(); // Re-load updated data
    printf("Voter %d status updated to voted=%d.\n", voter_id, voted_status);
}

void list_eligible_voters() {
    if (!all_voters || num_all_voters == 0) {
        printf("No voters registered yet.\n");
        return;
    }
    printf("\n--- All Registered Voters (Eligibility & Vote Status) ---\n");
    for (int i = 0; i < num_all_voters; i++) {
        printf("ID: %d, Name: %s, Eligible: %s, Voted: %s\n",
               all_voters[i].id,
               all_voters[i].name,
               all_voters[i].is_eligible ? "Yes" : "No",
               all_voters[i].has_voted ? "Yes" : "No");
    }
    printf("----------------------------------------------------------\n");
}

// --- Vote Recording & Security Implementations ---

// Simple XOR encoding/decoding function (for demonstration ONLY, not secure for real use)
// Use a consistent key for both encoding and decoding
void xor_encode_decode(char *data, size_t len, const char *key) {
    size_t key_len = strlen(key);
    for (size_t i = 0; i < len; i++) {
        data[i] ^= key[i % key_len];
    }
}

void record_vote(int voter_id, const char *candidate_id_str) {
    printf("Attempting to record vote for voter %d, candidate %s...\n", voter_id, candidate_id_str);

    Voter *voter = find_voter_by_id(voter_id);
    if (!voter) {
        printf("Error: Voter %d not found in system.\n", voter_id);
        write_serial_data("ACK:VOTE_ERROR\n");
        return;
    }
    if (!voter->is_eligible) {
        printf("Error: Voter %d is not eligible to vote.\n", voter_id);
        write_serial_data("ACK:VOTE_ERROR\n");
        return;
    }
    if (voter->has_voted) {
        printf("Error: Voter %d has already voted.\n", voter_id);
        write_serial_data("ACK:VOTE_ERROR\n");
        return;
    }

    // Check if the candidate ID from EVM is valid
    bool candidate_found = false;
    for (int i = 0; i < num_candidates; i++) {
        if (strcmp(candidate_id_str, candidates[i].id) == 0) {
            candidate_found = true;
            break;
        }
    }
    if (!candidate_found) {
        printf("Error: Invalid candidate ID '%s' received from EVM.\n", candidate_id_str);
        write_serial_data("ACK:VOTE_ERROR\n");
        return;
    }

    FILE *fp = fopen(VOTES_FILE, "a");
    if (!fp) {
        perror("Error opening votes file for appending");
        write_serial_data("ACK:VOTE_ERROR\n");
        return;
    }

    char record_string[MAX_LINE_LEN];
    time_t current_time = time(NULL);
    // Format: VoterID,CandidateID,Timestamp
    snprintf(record_string, sizeof(record_string), "%d,%s,%ld", voter_id, candidate_id_str, (long)current_time);

    char encoding_key[] = "MY_VOTING_SECRET_KEY"; // Use a strong, consistent key
    xor_encode_decode(record_string, strlen(record_string), encoding_key);

    fprintf(fp, "%s\n", record_string);
    fclose(fp);
    printf("Vote from voter %d for candidate %s recorded successfully.\n", voter_id, candidate_id_str);

    update_voter_status_in_file(voter_id, true); // Mark voter as voted

    // Signal EVM that vote was OK
    write_serial_data("ACK:VOTE_OK\n");
    // Reset the current voter slot for the next vote
    current_voter_id_for_evm = -1;
    evm_slot_open = false;
}

void calculate_and_display_results() {
    int vote_counts[num_candidates];
    memset(vote_counts, 0, sizeof(vote_counts));

    FILE *fp = fopen(VOTES_FILE, "r");
    if (!fp) {
        perror("Error opening votes file for reading. No votes cast yet?");
        return;
    }

    char line[MAX_LINE_LEN];
    char encoding_key[] = "MY_VOTING_SECRET_KEY"; // Same key as used for encoding

    while (fgets(line, sizeof(line), fp)) {
        // Need a copy because xor_encode_decode modifies in place
        char decoded_line[MAX_LINE_LEN];
        strncpy(decoded_line, line, sizeof(decoded_line) - 1);
        decoded_line[sizeof(decoded_line) - 1] = '\0';

        // Remove trailing newline if present for decoding
        size_t len = strlen(decoded_line);
        if (len > 0 && decoded_line[len-1] == '\n') {
            decoded_line[len-1] = '\0';
            len--;
        }
        
        xor_encode_decode(decoded_line, len, encoding_key);

        int voter_id;
        char candidate_id_str[10];
        long timestamp;
        if (sscanf(decoded_line, "%d,%9[^,],%ld", &voter_id, candidate_id_str, &timestamp) == 3) {
            for (int i = 0; i < num_candidates; i++) {
                if (strcmp(candidate_id_str, candidates[i].id) == 0) {
                    vote_counts[i]++;
                    break;
                }
            }
        } else {
            fprintf(stderr, "Warning: Skipping malformed or undecodable vote record: %s\n", line);
        }
    }
    fclose(fp);

    printf("\n--- Election Results ---\n");
    for (int i = 0; i < num_candidates; i++) {
        printf("%s (%s): %d votes\n", candidates[i].name, candidates[i].id, vote_counts[i]);
    }

    int total_votes_cast = 0;
    for(int i = 0; i < num_candidates; i++) {
        total_votes_cast += vote_counts[i];
    }
    printf("Total votes cast: %d\n", total_votes_cast);

    int total_eligible = 0;
    for(int i = 0; i < num_all_voters; i++) {
        if(all_voters[i].is_eligible) {
            total_eligible++;
        }
    }
    if (total_eligible > 0) {
        printf("Voter Turnout: %.2f%%\n", (double)total_votes_cast / total_eligible * 100.0);
    } else {
        printf("No eligible voters registered to calculate turnout.\n");
    }

    printf("------------------------\n");
}


// --- Serial Communication Implementations (Platform Specific) ---

#ifdef _WIN32
bool open_serial_port(const char *port_name, DWORD baud_rate) {
    serial_port_handle = CreateFileA(port_name,
                                     GENERIC_READ | GENERIC_WRITE,
                                     0,    // No sharing
                                     NULL, // No security attributes
                                     OPEN_EXISTING,
                                     FILE_ATTRIBUTE_NORMAL,
                                     NULL);
    if (serial_port_handle == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "Error opening serial port %s (Error Code: %lu)\n", port_name, GetLastError());
        return false;
    }

    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(serial_port_handle, &dcbSerialParams)) {
        fprintf(stderr, "Error getting current serial port settings.\n");
        CloseHandle(serial_port_handle);
        return false;
    }

    dcbSerialParams.BaudRate = baud_rate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(serial_port_handle, &dcbSerialParams)) {
        fprintf(stderr, "Error setting serial port settings.\n");
        CloseHandle(serial_port_handle);
        return false;
    }

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;       // Max time between chars in a ReadFile operation
    timeouts.ReadTotalTimeoutConstant = 50;  // Total time for ReadFile
    timeouts.ReadTotalTimeoutMultiplier = 10;// Multiplier for ReadFile
    timeouts.WriteTotalTimeoutConstant = 50; // Total time for WriteFile
    timeouts.WriteTotalTimeoutMultiplier = 10;// Multiplier for WriteFile
    if (!SetCommTimeouts(serial_port_handle, &timeouts)) {
        fprintf(stderr, "Error setting serial port timeouts.\n");
        CloseHandle(serial_port_handle);
        return false;
    }
    printf("Serial port %s opened successfully.\n", port_name);
    return true;
}

int read_serial_data(char *buffer, int max_len) {
    DWORD bytes_read;
    // Set up a small timeout for non-blocking read
    // This allows the thread to check other conditions and not get stuck forever.
    SetCommTimeouts(serial_port_handle, &((COMMTIMEOUTS){.ReadIntervalTimeout = 50, .ReadTotalTimeoutConstant = 50, .ReadTotalTimeoutMultiplier = 10}));

    if (!ReadFile(serial_port_handle, buffer, max_len, &bytes_read, NULL)) {
        // Check if error is due to timeout or actual error
        DWORD error = GetLastError();
        if (error == ERROR_IO_PENDING) { // Read pending, will be completed later
            return 0; // No data yet
        } else if (error != ERROR_SUCCESS) {
            // Real error occurred
            fprintf(stderr, "Error reading from serial port: %lu\n", error);
            return -1;
        }
    }
    buffer[bytes_read] = '\0'; // Null-terminate the string
    return bytes_read;
}

bool write_serial_data(const char *data) {
    DWORD bytes_written;
    if (!WriteFile(serial_port_handle, data, strlen(data), &bytes_written, NULL)) {
        fprintf(stderr, "Error writing to serial port: %lu\n", GetLastError());
        return false;
    }
    printf("PC Sent: %s", data); // Echo what was sent
    return true;
}

void close_serial_port() {
    if (serial_port_handle != INVALID_HANDLE_VALUE) {
        CloseHandle(serial_port_handle);
        serial_port_handle = INVALID_HANDLE_VALUE;
        printf("Serial port closed.\n");
    }
}

#else // Linux/macOS
bool open_serial_port(const char *port_name, speed_t baud_rate) { // Use speed_t for baud_rate
    serial_port_fd = open(port_name, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_port_fd < 0) {
        perror("Error opening serial port");
        return false;
    }

    struct termios tty;
    if (tcgetattr(serial_port_fd, &tty) != 0) {
        perror("Error from tcgetattr");
        close(serial_port_fd);
        return false;
    }

    cfsetospeed(&tty, baud_rate); // Set output baud rate
    cfsetispeed(&tty, baud_rate); // Set input baud rate

    tty.c_cflag &= ~PARENB;        // No parity
    tty.c_cflag &= ~CSTOPB;        // 1 stop bit
    tty.c_cflag &= ~CSIZE;         // Clear data size bits
    tty.c_cflag |= CS8;            // 8 data bits
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines

    tty.c_lflag &= ~ICANON; // Disable canonical mode (raw input)
    tty.c_lflag &= ~ECHO;   // Disable echo
    tty.c_lflag &= ~ECHOE;  // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable special handling of CR/LF

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = 1; // Wait for up to 1 decisecond (100ms) for data (0.1 seconds)
    tty.c_cc[VMIN] = 0;  // Return immediately if no data

    if (tcsetattr(serial_port_fd, TCSANOW, &tty) != 0) {
        perror("Error from tcsetattr");
        close(serial_port_fd);
        return false;
    }
    printf("Serial port %s opened successfully.\n", port_name);
    return true;
}

int read_serial_data(char *buffer, int max_len) {
    int bytes_read = read(serial_port_fd, buffer, max_len);
    if (bytes_read < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0; // No data available right now (non-blocking)
        }
        perror("Error reading from serial port");
        return -1;
    }
    buffer[bytes_read] = '\0'; // Null-terminate
    return bytes_read;
}

bool write_serial_data(const char *data) {
    int bytes_written = write(serial_port_fd, data, strlen(data));
    if (bytes_written < 0) {
        perror("Error writing to serial port");
        return false;
    }
    printf("PC Sent: %s", data); // Echo what was sent
    return true;
}

void close_serial_port() {
    if (serial_port_fd != -1) {
        close(serial_port_fd);
        serial_port_fd = -1;
        printf("Serial port closed.\n");
    }
}
#endif // _WIN32


// --- Serial Communication Logic (Called by a Thread) ---

void handle_evm_data(const char *data) {
    printf("PC Received: %s", data); // Raw data received from EVM

    // Remove newline character if present for clean parsing
    char clean_data[SERIAL_READ_BUFFER_SIZE];
    strncpy(clean_data, data, sizeof(clean_data) - 1);
    clean_data[sizeof(clean_data) - 1] = '\0';
    size_t len = strlen(clean_data);
    if (len > 0 && clean_data[len-1] == '\n') {
        clean_data[len-1] = '\0';
    }


    if (strcmp(clean_data, "EVM_READY") == 0) {
        printf("EVM reports it is ready.\n");
        evm_is_ready = true;
    } else if (strcmp(clean_data, "EVM:VOTER_SLOT_OPEN") == 0) {
        printf("EVM confirms voter slot is open and ready for input.\n");
        evm_slot_open = true;
        // At this point, your GUI would show "Please ask voter to cast vote"
    } else if (strncmp(clean_data, "VOTE:", 5) == 0) {
        // This is a vote message, e.g., "VOTE:1" (candidate ID)
        char candidate_id_str[10];
        if (sscanf(clean_data + 5, "%9s", candidate_id_str) == 1) {
            printf("Received vote for candidate: %s from EVM.\n", candidate_id_str);
            // Now, record the vote using the current_voter_id_for_evm
            if (current_voter_id_for_evm != -1) {
                record_vote(current_voter_id_for_evm, candidate_id_str);
            } else {
                fprintf(stderr, "Error: Received vote from EVM but no voter was authorized! (Candidate: %s)\n", candidate_id_str);
                write_serial_data("ACK:VOTE_ERROR\n");
            }
        } else {
            fprintf(stderr, "Malformed VOTE message from EVM: %s\n", clean_data);
            write_serial_data("ACK:VOTE_ERROR\n");
        }
    } else {
        printf("Unhandled message from EVM: %s\n", clean_data);
    }
}

// This function should be run in a separate thread in a GUI application
// to continuously read from the serial port.
void serial_monitor_loop() {
    char read_buffer[SERIAL_READ_BUFFER_SIZE];
    int bytes_read_total = 0;
    char incoming_line[SERIAL_READ_BUFFER_SIZE]; // To build up a full line

    printf("Starting serial monitor loop...\n");
    while (true) { // Replace 'true' with a global flag that controls application exit
        int bytes_read = read_serial_data(read_buffer, sizeof(read_buffer) - 1);

        if (bytes_read > 0) {
            read_buffer[bytes_read] = '\0'; // Ensure null-termination
            // Append to our line buffer
            strncat(incoming_line, read_buffer, sizeof(incoming_line) - strlen(incoming_line) -1 );
            
            // Check for a complete line (newline character)
            char *newline_pos;
            while ((newline_pos = strchr(incoming_line, '\n')) != NULL) {
                *newline_pos = '\0'; // Null-terminate the current line
                handle_evm_data(incoming_line); // Process the complete line
                
                // Shift the remaining data to the beginning of the buffer
                memmove(incoming_line, newline_pos + 1, strlen(newline_pos + 1) + 1);
            }
        } else if (bytes_read == -1) {
            fprintf(stderr, "Serial read error. Stopping serial monitor loop.\n");
            break; // Exit loop on error
        }
        // Small delay to prevent busy-waiting and allow other threads to run
        #ifdef _WIN32
        Sleep(10); // 10 milliseconds
        #else
        usleep(10000); // 10,000 microseconds = 10 milliseconds
        #endif
    }
    printf("Serial monitor loop stopped.\n");
}

// --- Main Application Loop (Conceptual) ---

int main() {
    // 1. Initialize data (load existing voters)
    load_voters_from_file();

    // 2. Open serial port for ESP32 communication
    // !!! IMPORTANT: CHANGE "COM3" TO YOUR ESP32's ACTUAL PORT !!!
    // Linux/macOS example: "/dev/ttyUSB0" or "/dev/ttyACM0"
    #ifdef _WIN32
    if (!open_serial_port("COM3", CBR_115200)) {
    #else
    if (!open_serial_port("/dev/ttyUSB0", B115200)) { // Use B115200 for 115200 baud
    #endif
        fprintf(stderr, "Failed to open serial port. Exiting.\n");
        return 1;
    }

    // --- Start Serial Communication Thread (Recommended for GUI apps) ---
    // In a real GUI application, you would create a new thread here
    // and run serial_monitor_loop in that thread.
    // Example (pthread):
    // pthread_t serial_thread;
    // pthread_create(&serial_thread, NULL, (void *(*)(void *))serial_monitor_loop, NULL);
    //
    // For this console example, we'll just call it directly for a short period
    // or integrate it into a simple menu loop.

    printf("\n--- Console Voting System Control ---\n");
    printf("Wait for 'EVM reports it is ready.' before proceeding.\n");

    int choice;
    do {
        printf("\nMenu:\n");
        printf("1. Register New Voter\n");
        printf("2. List All Voters\n");
        printf("3. Authorize Next Voter (Sends PC_READY to EVM)\n");
        printf("4. Display Current Election Results\n");
        printf("5. Listen for EVM data (temporary, for console test)\n"); // Will block, for debugging serial loop
        printf("0. Exit\n");
        printf("Enter choice: ");
        scanf("%d", &choice);
        // Clear input buffer
        while (getchar() != '\n');

        switch (choice) {
            case 1: {
                int id;
                char name[MAX_NAME_LEN];
                int eligible_flag;
                printf("Enter new voter ID: ");
                scanf("%d", &id);
                while (getchar() != '\n');
                printf("Enter voter name: ");
                fgets(name, MAX_NAME_LEN, stdin);
                name[strcspn(name, "\n")] = 0; // Remove newline
                printf("Is voter eligible (1=Yes, 0=No): ");
                scanf("%d", &eligible_flag);
                while (getchar() != '\n');
                register_new_voter(id, name, (bool)eligible_flag);
                break;
            }
            case 2:
                list_eligible_voters();
                break;
            case 3:
                if (!evm_is_ready) {
                    printf("EVM is not ready. Please wait or check connection.\n");
                    break;
                }
                if (evm_slot_open) {
                    printf("EVM is already waiting for a vote. Please wait for current voter or EVM error.\n");
                    break;
                }
                int voter_to_authorize;
                printf("Enter Voter ID to authorize for voting: ");
                scanf("%d", &voter_to_authorize);
                while (getchar() != '\n');

                Voter *voter = find_voter_by_id(voter_to_authorize);
                if (!voter) {
                    printf("Voter with ID %d not found. Cannot authorize.\n", voter_to_authorize);
                } else if (!voter->is_eligible) {
                    printf("Voter %d is not eligible. Cannot authorize.\n", voter_to_authorize);
                } else if (voter->has_voted) {
                    printf("Voter %d has already voted. Cannot authorize again.\n", voter_to_authorize);
                } else {
                    current_voter_id_for_evm = voter_to_authorize;
                    write_serial_data("PC_READY\n"); // Tell EVM to open slot
                    printf("Sent PC_READY to EVM for voter %d. Waiting for EVM to confirm slot open and vote.\n", voter_to_authorize);
                    // In a GUI, you'd then wait for EVM:VOTER_SLOT_OPEN and the subsequent VOTE: message
                }
                break;
            case 4:
                calculate_and_display_results();
                break;
            case 5:
                printf("Entering blocking serial monitor loop (Ctrl+C to exit this loop)...\n");
                serial_monitor_loop(); // For testing, this will block
                break;
            case 0:
                printf("Exiting application.\n");
                break;
            default:
                printf("Invalid choice. Please try again.\n");
        }
    } while (choice != 0);


    // --- Cleanup ---
    if (all_voters) {
        free(all_voters);
    }
    close_serial_port();

    printf("Voting system application finished.\n");
    return 0;
}
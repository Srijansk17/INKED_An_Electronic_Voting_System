#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h> // For bool type

// Platform-specific includes for serial communication and threading
#ifdef _WIN32
#include <windows.h> // For serial communication and Sleep()
#include <process.h> // For _beginthreadex (threading)
#else
#include <unistd.h>  // For usleep()
#include <fcntl.h>   // For file control options (non-blocking)
#include <termios.h> // For serial port settings
#include <errno.h>   // For errno
#include <pthread.h> // For pthreads (threading)
#endif

// --- Constants ---
#define MAX_VOTERS 100
#define ID_LENGTH 10
#define NAME_LENGTH 50
#define CANDIDATE_COUNT 5 // Assuming 5 candidates (0-4)
#define SERIAL_READ_BUFFER_SIZE 256
#define VOTERS_FILE "voters.txt"
#define RESULTS_FILE "results.txt"

// --- Data Structures ---
typedef struct {
    char id[ID_LENGTH];
    char name[NAME_LENGTH];
    bool has_voted;
} Voter;

// --- Global Variables ---
Voter *all_voters = NULL;
int voter_count = 0;
int election_results[CANDIDATE_COUNT] = {0}; // Initialize all to 0

// --- State variables for managing current voting session ---
int current_voter_id_for_evm = -1; // -1 means no voter currently authorized
volatile bool evm_is_ready = false;         // Flag for EVM status, volatile as accessed by multiple threads
volatile bool evm_slot_open = false;        // Flag indicating EVM is ready for a vote input

volatile bool app_running = true; // <--- NEW GLOBAL FLAG: Controls the lifetime of the serial thread

// --- Serial Port Handles/File Descriptors ---
#ifdef _WIN32
HANDLE serial_port_handle = INVALID_HANDLE_VALUE;
#else
int serial_port_fd = -1;
#endif

// --- Function Prototypes ---
// Serial Communication Functions
#ifdef _WIN32
bool open_serial_port(const char *port_name, DWORD baud_rate);
#else
bool open_serial_port(const char *port_name, speed_t baud_rate);
#endif
void close_serial_port();
int write_serial_data(const char *data);
int read_serial_data(char *buffer, int max_len);
void handle_evm_data(const char *data);
// Thread entry point function
#ifdef _WIN32
unsigned __stdcall serial_monitor_thread_func(void* args);
#else
void *serial_monitor_thread_func(void* args);
#endif

// Voter Management Functions
void register_voter();
void list_voters();
Voter *find_voter(const char *id);
void load_voters_from_file();
void save_voters_to_file();
void record_vote(const char *voter_id, int candidate_id);
void authorize_next_voter();
void display_current_election_results();
void load_results_from_file();
void save_results_to_file();

// --- Main Program ---
int main() {
    // 1. Initialize data (load existing voters and results)
    load_voters_from_file();
    load_results_from_file();

    // 2. Open serial port for ESP32 communication
    #ifdef _WIN32
    // IMPORTANT: Change "COM3" to your actual COM port number!
    if (!open_serial_port("COM3", CBR_115200)) {
    #else
    // IMPORTANT: Change "/dev/ttyUSB0" to your actual serial port path!
    if (!open_serial_port("/dev/ttyUSB0", B115200)) {
    #endif
        fprintf(stderr, "Failed to open serial port. Exiting.\n");
        return 1;
    }

    // --- Start Serial Communication Thread ---
    #ifdef _WIN32
    HANDLE serial_thread_handle;
    serial_thread_handle = (HANDLE)_beginthreadex(NULL, 0, serial_monitor_thread_func, NULL, 0, NULL);
    if (serial_thread_handle == 0) {
        fprintf(stderr, "Failed to create serial monitor thread. Error: %lu\n", GetLastError());
        close_serial_port();
        return 1;
    }
    #else
    pthread_t serial_thread_id;
    if (pthread_create(&serial_thread_id, NULL, serial_monitor_thread_func, NULL) != 0) {
        perror("Failed to create serial monitor thread");
        close_serial_port();
        return 1;
    }
    #endif

    printf("\n--- Console Voting System Control ---\n");
    printf("Waiting for 'EVM reports it is ready.' before proceeding.\n");
    printf("Please ensure your ESP32 is powered on and press its RST/EN button if needed.\n");

    // Wait for EVM to report ready - the background thread will update evm_is_ready
    while (!evm_is_ready && app_running) { 
        // Small delay to prevent busy-waiting for the main thread
        #ifdef _WIN32
        Sleep(100); 
        #else
        usleep(100000); 
        #endif
    }

    if (!app_running) { // If app_running became false due to serial error in thread
        fprintf(stderr, "Application terminated due to serial error during EVM ready check.\n");
        close_serial_port();
        return 1;
    }
    printf("EVM is ready! Proceeding to menu.\n");


    int choice;
    do {
        printf("\nMenu:\n");
        printf("1. Register New Voter\n");
        printf("2. List All Voters\n");
        printf("3. Authorize Next Voter (Sends PC_READY to EVM)\n");
        printf("4. Display Current Election Results\n");
        printf("0. Exit\n");
        printf("Enter choice: ");
        scanf("%d", &choice);
        // Clear input buffer (important after scanf for integers)
        while (getchar() != '\n');

        switch (choice) {
            case 1:
                register_voter();
                break;
            case 2:
                list_voters();
                break;
            case 3:
                authorize_next_voter();
                break;
            case 4:
                display_current_election_results();
                break;
            case 0:
                printf("Exiting application.\n");
                app_running = false; // <--- SIGNAL THE SERIAL THREAD TO STOP
                break;
            default:
                printf("Invalid choice. Please try again.\n");
        }
    } while (choice != 0);

    // --- Cleanup ---
    // Wait for the serial communication thread to finish its work
    #ifdef _WIN32
    WaitForSingleObject(serial_thread_handle, INFINITE);
    CloseHandle(serial_thread_handle);
    #else
    pthread_join(serial_thread_id, NULL);
    #endif

    save_voters_to_file();
    save_results_to_file();

    if (all_voters) {
        free(all_voters);
    }
    close_serial_port();

    printf("Voting system application finished.\n");
    return 0;
}


// --- Serial Communication Implementations ---

#ifdef _WIN32
bool open_serial_port(const char *port_name, DWORD baud_rate) {
    serial_port_handle = CreateFile(port_name,
                                    GENERIC_READ | GENERIC_WRITE,
                                    0,      // no sharing
                                    NULL,   // no security
                                    OPEN_EXISTING,
                                    FILE_FLAG_OVERLAPPED, // Use overlapped for non-blocking I/O
                                    NULL);  // no templates

    if (serial_port_handle == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "Error opening serial port %s. Error: %lu\n", port_name, GetLastError());
        return false;
    }

    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(serial_port_handle, &dcbSerialParams)) {
        fprintf(stderr, "Error getting current serial port state.\n");
        CloseHandle(serial_port_handle);
        return false;
    }

    dcbSerialParams.BaudRate = baud_rate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(serial_port_handle, &dcbSerialParams)) {
        fprintf(stderr, "Error setting serial port state.\n");
        CloseHandle(serial_port_handle);
        return false;
    }

    // Set timeouts for non-blocking reads and reasonable writes
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = MAXDWORD; // Return immediately if no data is available
    timeouts.ReadTotalTimeoutConstant = 0;   // No constant timeout
    timeouts.ReadTotalTimeoutMultiplier = 0; // No multiplier timeout
    timeouts.WriteTotalTimeoutConstant = 50;     // Max total time to write in ms
    timeouts.WriteTotalTimeoutMultiplier = 10;   // Multiplier

    if (!SetCommTimeouts(serial_port_handle, &timeouts)) {
        fprintf(stderr, "Error setting serial port timeouts.\n");
        CloseHandle(serial_port_handle);
        return false;
    }

    printf("Serial port %s opened successfully.\n", port_name);
    return true;
}

void close_serial_port() {
    if (serial_port_handle != INVALID_HANDLE_VALUE) {
        CloseHandle(serial_port_handle);
        serial_port_handle = INVALID_HANDLE_VALUE;
        printf("Serial port closed.\n");
    }
}

int write_serial_data(const char *data) {
    DWORD bytes_written;
    if (serial_port_handle == INVALID_HANDLE_VALUE) return -1;
    
    // Add \r\n explicitly for cross-platform consistency with Arduino's Serial.println()
    char temp_data[SERIAL_READ_BUFFER_SIZE];
    // Ensure enough space for data + "\r\n" + null terminator
    strncpy(temp_data, data, sizeof(temp_data) - 3); 
    temp_data[sizeof(temp_data) - 3] = '\0'; // Ensure termination
    strcat(temp_data, "\r\n"); 

    OVERLAPPED osWrite = {0};
    osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    if (osWrite.hEvent == NULL) {
        fprintf(stderr, "Error creating write event: %lu\n", GetLastError());
        return -1;
    }

    if (!WriteFile(serial_port_handle, temp_data, strlen(temp_data), &bytes_written, &osWrite)) {
        if (GetLastError() != ERROR_IO_PENDING) { // Write not complete immediately
            fprintf(stderr, "Error writing to serial port: %lu\n", GetLastError());
            CloseHandle(osWrite.hEvent);
            return -1;
        }
        // Write is pending, wait for it
        if (WaitForSingleObject(osWrite.hEvent, INFINITE) != WAIT_OBJECT_0) {
            fprintf(stderr, "Error waiting for write event: %lu\n", GetLastError());
            CloseHandle(osWrite.hEvent);
            return -1;
        }
        if (!GetOverlappedResult(serial_port_handle, &osWrite, &bytes_written, FALSE)) {
            fprintf(stderr, "Error getting overlapped write result: %lu\n", GetLastError());
            CloseHandle(osWrite.hEvent);
            return -1;
        }
    }
    CloseHandle(osWrite.hEvent);
    printf("PC Sent: %s\n", data); // Print original data for console log
    return bytes_written;
}

int read_serial_data(char *buffer, int max_len) {
    DWORD bytes_read;
    if (serial_port_handle == INVALID_HANDLE_VALUE) return -1;

    OVERLAPPED osRead = {0};
    osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    if (osRead.hEvent == NULL) {
        fprintf(stderr, "Error creating read event: %lu\n", GetLastError());
        return -1;
    }

    // Issue the read operation
    if (!ReadFile(serial_port_handle, buffer, max_len, &bytes_read, &osRead)) {
        if (GetLastError() == ERROR_IO_PENDING) {
            // Read is pending, wait briefly for data
            DWORD waitResult = WaitForSingleObject(osRead.hEvent, 100); // Wait up to 100ms
            if (waitResult == WAIT_OBJECT_0) {
                if (!GetOverlappedResult(serial_port_handle, &osRead, &bytes_read, FALSE)) {
                    fprintf(stderr, "Error getting overlapped read result: %lu\n", GetLastError());
                    CloseHandle(osRead.hEvent);
                    return -1;
                }
            } else if (waitResult == WAIT_TIMEOUT) {
                // No data within timeout, cancel the pending read
                CancelIo(serial_port_handle); // This will cause the pending read to fail
                bytes_read = 0; // Indicate no data read
            } else {
                fprintf(stderr, "Error waiting for read event: %lu\n", GetLastError());
                CloseHandle(osRead.hEvent);
                return -1;
            }
        } else {
            fprintf(stderr, "Error reading from serial port: %lu\n", GetLastError());
            CloseHandle(osRead.hEvent);
            return -1;
        }
    }
    CloseHandle(osRead.hEvent);
    buffer[bytes_read] = '\0';
    return bytes_read;
}

#else // Linux/macOS
bool open_serial_port(const char *port_name, speed_t baud_rate) {
    // O_RDWR   = Read/Write
    // O_NOCTTY = No controlling terminal (prevents kernel from making it controlling tty)
    // O_NONBLOCK = Non-blocking mode
    serial_port_fd = open(port_name, O_RDWR | O_NOCTTY | O_NONBLOCK); 
    if (serial_port_fd < 0) {
        perror("Error opening serial port");
        return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty)); // Clear struct

    if (tcgetattr(serial_port_fd, &tty) != 0) {
        perror("Error from tcgetattr");
        close(serial_port_fd);
        return false;
    }

    cfsetospeed(&tty, baud_rate);
    cfsetispeed(&tty, baud_rate);

    tty.c_cflag |= (CLOCAL | CREAD); // Enable the receiver and set local mode
    tty.c_cflag &= ~PARENB;          // No parity
    tty.c_cflag &= ~CSTOPB;          // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;              // 8 data bits
    tty.c_cflag &= ~CRTSCTS;         // Disable RTS/CTS hardware flow control

    tty.c_lflag &= ~ICANON;  // Disable canonical mode (raw input)
    tty.c_lflag &= ~ECHO;    // Disable echo
    tty.c_lflag &= ~ECHOE;   // Disable erasure
    tty.c_lflag &= ~ECHONL;  // Disable new-line echo
    tty.c_lflag &= ~ISIG;    // Disable interpretation of INTR, QUIT and SUSP

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow control
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    // Set VMIN and VTIME to 0 for non-blocking read
    tty.c_cc[VMIN] = 0;  // Read non-blocking: read returns immediately with available bytes
    tty.c_cc[VTIME] = 0; // Read non-blocking: read returns immediately

    if (tcsetattr(serial_port_fd, TCSANOW, &tty) != 0) {
        perror("Error from tcsetattr");
        close(serial_port_fd);
        return false;
    }

    printf("Serial port %s opened successfully.\n", port_name);
    return true;
}

void close_serial_port() {
    if (serial_port_fd != -1) {
        close(serial_port_fd);
        serial_port_fd = -1;
        printf("Serial port closed.\n");
    }
}

int write_serial_data(const char *data) {
    if (serial_port_fd == -1) return -1;
    
    // Add \r\n explicitly for cross-platform consistency with Arduino's Serial.println()
    char temp_data[SERIAL_READ_BUFFER_SIZE];
    strncpy(temp_data, data, sizeof(temp_data) - 3); 
    temp_data[sizeof(temp_data) - 3] = '\0'; // Ensure termination
    strcat(temp_data, "\r\n"); 

    int bytes_written = write(serial_port_fd, temp_data, strlen(temp_data));
    if (bytes_written < 0) {
        perror("Error writing to serial port");
    } else {
        printf("PC Sent: %s\n", data); // Print original data for console log
    }
    return bytes_written;
}

int read_serial_data(char *buffer, int max_len) {
    if (serial_port_fd == -1) return -1;
    int bytes_read = read(serial_port_fd, buffer, max_len);
    if (bytes_read < 0) {
        if (errno == EWOULDBLOCK || errno == EAGAIN) {
            return 0; // No data available right now (non-blocking)
        }
        perror("Error reading from serial port");
        return -1;
    }
    buffer[bytes_read] = '\0';
    return bytes_read;
}
#endif

// --- Thread Entry Point Function ---
#ifdef _WIN32
unsigned __stdcall serial_monitor_thread_func(void* args)
#else
void *serial_monitor_thread_func(void* args)
#endif
{
    char incoming_line_buffer[SERIAL_READ_BUFFER_SIZE] = {'\0'}; 

    printf("Serial monitor thread started.\n");
    while (app_running) { // Loop as long as the main application is running
        int remaining_space = sizeof(incoming_line_buffer) - strlen(incoming_line_buffer) - 1;
        if (remaining_space <= 0) {
            fprintf(stderr, "Warning: Incoming serial line buffer full in thread. Clearing buffer.\n");
            incoming_line_buffer[0] = '\0'; // Clear the buffer if full
            remaining_space = sizeof(incoming_line_buffer) - 1; 
        }

        int bytes_read = read_serial_data(incoming_line_buffer + strlen(incoming_line_buffer), remaining_space);

        if (bytes_read > 0) {
            incoming_line_buffer[strlen(incoming_line_buffer)] = '\0'; // Null-terminate after new data

            char *newline_pos;
            // Process all complete lines currently in the buffer
            while ((newline_pos = strchr(incoming_line_buffer, '\n')) != NULL) {
                *newline_pos = '\0'; // Null-terminate the current line
                handle_evm_data(incoming_line_buffer); // Process the complete line
                
                // Shift the remaining data (if any) to the beginning of the buffer
                memmove(incoming_line_buffer, newline_pos + 1, strlen(newline_pos + 1) + 1);
            }
        } else if (bytes_read == -1) {
            fprintf(stderr, "Serial read error in thread. Signalling application to stop.\n");
            app_running = false; // Signal main program to exit on error
            break; // Exit loop on error
        }
        // Small delay to prevent busy-waiting and allow other threads/processes time
        #ifdef _WIN32
        Sleep(10); 
        #else
        usleep(10000); 
        #endif
    }
    printf("Serial monitor thread stopped.\n");
    #ifdef _WIN32
    _endthreadex(0); // Explicitly end the thread on Windows
    return 0;
    #else
    return NULL; // Return NULL for pthreads
    #endif
}

// --- Handle Incoming EVM Data ---
// This function processes complete lines received from the ESP32
void handle_evm_data(const char *data) {
    printf("PC Received (raw): '%s'\n", data);

    char clean_data[SERIAL_READ_BUFFER_SIZE];
    strncpy(clean_data, data, sizeof(clean_data) - 1);
    clean_data[sizeof(clean_data) - 1] = '\0';

    // Remove any trailing newline (\n) or carriage return (\r) characters
    size_t len = strlen(clean_data);
    while (len > 0 && (clean_data[len-1] == '\n' || clean_data[len-1] == '\r')) {
        clean_data[len-1] = '\0';
        len--;
    }
    
    printf("PC Received (cleaned): '%s'\n", clean_data); 

    if (strcmp(clean_data, "EVM_READY") == 0) {
        printf("EVM reports it is ready.\n");
        evm_is_ready = true;
        current_voter_id_for_evm = -1; // Reset voter ID on EVM ready
        evm_slot_open = false; // Reset slot status
    } else if (strcmp(clean_data, "EVM:VOTER_SLOT_OPEN") == 0) {
        printf("EVM confirms voter slot is open and ready for input.\n");
        evm_slot_open = true;
    } else if (strncmp(clean_data, "VOTE:", 5) == 0) {
        if (current_voter_id_for_evm == -1) {
            printf("Error: Received vote but no voter was authorized. Sending ACK:VOTE_ERROR.\n");
            write_serial_data("ACK:VOTE_ERROR");
            return;
        }

        int candidate_id = atoi(clean_data + 5); // Extract candidate ID
        if (candidate_id >= 0 && candidate_id < CANDIDATE_COUNT) {
            record_vote(all_voters[current_voter_id_for_evm].id, candidate_id);
            printf("Vote for candidate %d recorded for voter %s.\n", candidate_id, all_voters[current_voter_id_for_evm].id);
            write_serial_data("ACK:VOTE_OK");
            
            // After successful vote, reset authorization for next voter
            current_voter_id_for_evm = -1; 
            evm_slot_open = false;

        } else {
            printf("Error: Invalid candidate ID received: %d. Sending ACK:VOTE_ERROR.\n", candidate_id);
            write_serial_data("ACK:VOTE_ERROR");
            // Still reset authorization even on error for next voter
            current_voter_id_for_evm = -1; 
            evm_slot_open = false;
        }
    } else {
        printf("Unhandled message from EVM: '%s'\n", clean_data);
    }
}

// --- Voter Management Implementations (Remain the same as before) ---

void register_voter() {
    if (voter_count >= MAX_VOTERS) {
        printf("Maximum voters reached. Cannot register more.\n");
        return;
    }

    // Dynamically expand voter array if needed
    if (voter_count == 0) {
        all_voters = (Voter *)malloc(sizeof(Voter));
    } else {
        Voter *temp_voters = (Voter *)realloc(all_voters, (voter_count + 1) * sizeof(Voter));
        if (temp_voters == NULL) {
            fprintf(stderr, "Memory re-allocation failed for new voter.\n");
            return; // Keep existing voters
        }
        all_voters = temp_voters;
    }

    Voter new_voter;
    printf("Enter new voter ID (up to %d chars): ", ID_LENGTH - 1);
    scanf("%s", new_voter.id);
    while (getchar() != '\n'); // Clear buffer

    // Check if voter ID already exists
    if (find_voter(new_voter.id) != NULL) {
        printf("Voter with ID '%s' already exists.\n", new_voter.id);
        // If realloc happened, and then ID exists, shrink array back
        if (voter_count > 0) {
            all_voters = (Voter *)realloc(all_voters, voter_count * sizeof(Voter));
        } else {
            free(all_voters);
            all_voters = NULL;
        }
        return;
    }

    printf("Enter new voter Name (up to %d chars): ", NAME_LENGTH - 1);
    fgets(new_voter.name, NAME_LENGTH, stdin);
    new_voter.name[strcspn(new_voter.name, "\n")] = '\0'; // Remove newline
    new_voter.has_voted = false;

    all_voters[voter_count] = new_voter;
    voter_count++;
    printf("Voter '%s' registered successfully.\n", new_voter.name);
}

void list_voters() {
    if (voter_count == 0) {
        printf("No voters registered yet.\n");
        return;
    }
    printf("\n--- Registered Voters ---\n");
    for (int i = 0; i < voter_count; i++) {
        printf("ID: %s, Name: %s, Voted: %s\n", 
               all_voters[i].id, all_voters[i].name, 
               all_voters[i].has_voted ? "Yes" : "No");
    }
    printf("-------------------------\n");
}

Voter *find_voter(const char *id) {
    for (int i = 0; i < voter_count; i++) {
        if (strcmp(all_voters[i].id, id) == 0) {
            return &all_voters[i];
        }
    }
    return NULL;
}

void load_voters_from_file() {
    FILE *file = fopen(VOTERS_FILE, "r");
    if (!file) {
        printf("Voters file not found. Starting with no registered voters.\n");
        return;
    }

    if (all_voters != NULL) { // Free existing data if already loaded
        free(all_voters);
        all_voters = NULL;
        voter_count = 0;
    }

    Voter temp_voter;
    int items_read;
    while ((items_read = fscanf(file, "%[^,],%[^,],%d\n", temp_voter.id, temp_voter.name, (int *)&temp_voter.has_voted)) == 3) {
        if (voter_count >= MAX_VOTERS) {
            fprintf(stderr, "Warning: Max voters (%d) reached in file, truncating.\n", MAX_VOTERS);
            break;
        }
        if (voter_count == 0) {
            all_voters = (Voter *)malloc(sizeof(Voter));
        } else {
            Voter *temp_voters = (Voter *)realloc(all_voters, (voter_count + 1) * sizeof(Voter));
            if (temp_voters == NULL) {
                fprintf(stderr, "Memory re-allocation failed while loading voters. Truncating file read.\n");
                break;
            }
            all_voters = temp_voters;
        }
        all_voters[voter_count] = temp_voter;
        voter_count++;
    }
    if (items_read != EOF && items_read != 3) {
        fprintf(stderr, "Warning: Malformed line encountered in voters file.\n");
    }
    printf("Loaded %d voters.\n", voter_count);
    fclose(file);
}

void save_voters_to_file() {
    FILE *file = fopen(VOTERS_FILE, "w");
    if (!file) {
        perror("Error opening voters file for writing");
        return;
    }
    for (int i = 0; i < voter_count; i++) {
        fprintf(file, "%s,%s,%d\n", all_voters[i].id, all_voters[i].name, all_voters[i].has_voted);
    }
    fclose(file);
    printf("Voter data saved.\n");
}

void load_results_from_file() {
    FILE *file = fopen(RESULTS_FILE, "r");
    if (!file) {
        printf("Results file not found. Starting with empty results.\n");
        // Ensure results array is zeroed out if file doesn't exist
        for (int i = 0; i < CANDIDATE_COUNT; i++) {
            election_results[i] = 0;
        }
        return;
    }
    for (int i = 0; i < CANDIDATE_COUNT; i++) {
        if (fscanf(file, "%d\n", &election_results[i]) != 1) {
            fprintf(stderr, "Error reading results file or file is incomplete. Resetting results.\n");
            for (int j = 0; j < CANDIDATE_COUNT; j++) {
                election_results[j] = 0;
            }
            break;
        }
    }
    printf("Loaded election results.\n");
    fclose(file);
}

void save_results_to_file() {
    FILE *file = fopen(RESULTS_FILE, "w");
    if (!file) {
        perror("Error opening results file for writing");
        return;
    }
    for (int i = 0; i < CANDIDATE_COUNT; i++) {
        fprintf(file, "%d\n", election_results[i]);
    }
    fclose(file);
    printf("Election results saved.\n");
}

void record_vote(const char *voter_id_str, int candidate_id) {
    Voter *voter = find_voter(voter_id_str);
    if (voter == NULL) {
        printf("Error: Voter '%s' not found locally (should not happen after authorization).\n", voter_id_str);
        return;
    }
    if (voter->has_voted) {
        printf("Error: Voter '%s' has already voted (should not happen after authorization).\n", voter_id_str);
        return;
    }
    if (candidate_id < 0 || candidate_id >= CANDIDATE_COUNT) {
        printf("Error: Invalid candidate ID %d received from EVM.\n", candidate_id);
        return;
    }

    election_results[candidate_id]++;
    voter->has_voted = true;
    printf("Vote for candidate %d recorded for voter %s.\n", candidate_id, voter_id_str);
    // write_serial_data("ACK:VOTE_OK") is called in handle_evm_data after record_vote
}


void authorize_next_voter() {
    if (!evm_is_ready) {
        printf("EVM is not ready. Please wait or check connection.\n");
        return;
    }
    if (evm_slot_open) {
        printf("An EVM voting slot is already open. Please wait for the current vote to complete.\n");
        return;
    }

    char voter_id[ID_LENGTH];
    printf("Enter voter ID to authorize: ");
    scanf("%s", voter_id);
    while (getchar() != '\n'); // Clear buffer

    Voter *voter = find_voter(voter_id);
    if (voter == NULL) {
        printf("Voter with ID '%s' not found.\n", voter_id);
        return;
    }
    if (voter->has_voted) {
        printf("Voter '%s' has already voted.\n", voter_id);
        return;
    }

    // Store the index of the authorized voter in our global variable for later lookup
    current_voter_id_for_evm = (int)(voter - all_voters); 

    // Send authorization signal to EVM
    printf("Authorizing voter %s...\n", voter_id);
    write_serial_data("PC_READY"); // Signal EVM to open slot

    // The serial thread will now wait for "EVM:VOTER_SLOT_OPEN" and then "VOTE:X"
}

void display_current_election_results() {
    printf("\n--- Current Election Results ---\n");
    for (int i = 0; i < CANDIDATE_COUNT; i++) {
        printf("Candidate %d: %d votes\n", i, election_results[i]);
    }
    printf("--------------------------------\n");
}
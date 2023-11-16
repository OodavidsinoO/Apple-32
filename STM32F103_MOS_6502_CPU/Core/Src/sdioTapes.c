// Author: David Sin
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "fatfs.h"
#include "mos6502.h"

// External
extern uint8_t RAM[RAM_SIZE];

// 20KB of tape space
#define MAX_TAPE_SIZE 20480
#define BASIC_HEADER_START 0x004A
#define BASIC_HEADER_SIZE 182

typedef struct {
	TCHAR fullName[64];
    char name[64];
    uint16_t start;
    uint16_t end;
    char type[5];
} tapeFile;

// Filename parser
// Extract program name and start/end addresses
// Example: matrix.004A.00FF.basic
// Extract matrix, 004A
// Example: apple30th.0280.0FFF.bin
// Extract apple30th, 0280, 0FFF, bin
// return tapeFile struct

tapeFile* parseFilename(char* filename) {
    tapeFile* file = malloc(sizeof(tapeFile));
    strcpy(file->fullName, filename);
    char* token = strtok(filename, ".");
    strcpy(file->name, token);
    token = strtok(NULL, ".");
    file->start = strtol(token, NULL, 16);
    token = strtok(NULL, ".");
    file->end = strtol(token, NULL, 16);
    token = strtok(NULL, ".");
    strcpy(file->type, token);
    return file;
}

// Loading .basic files
// Create a buffer to hold the file
// Parse the filename to get the starting address and program name
// Read binary file into buffer
// Write first 182 bytes header to RAM starting from 0x004A
// Write the rest of the file to RAM starting from the starting address
// delete the buffer
// Return 0 on success

int loadBasic(tapeFile* file) {
    FATFS fs;
    FIL fileHandle;
    UINT bytesRead;
    uint8_t* buffer = malloc(MAX_TAPE_SIZE);
    FRESULT result = f_mount(&fs, SDPath, 1);
    char* line = malloc(64);

    if (result != FR_OK) {
        return -1;
    }
    // open file and read in the data (up to maximum tape size)
    f_open(&fileHandle, file->fullName, FA_READ | FA_OPEN_EXISTING);
		f_read(&fileHandle, buffer, MAX_TAPE_SIZE, &bytesRead);
		sprintf(line, "Bytes read: %d", bytesRead);
		writelineTerminal(line);
    f_close(&fileHandle);
    // copy first 'BASIC_HEADER_SIZE' worth of data from the basic header start address
    // the rest from the file's provided start address
    memcpy(&RAM[BASIC_HEADER_START], buffer, BASIC_HEADER_SIZE);
    memcpy(&RAM[file->start], &buffer[BASIC_HEADER_SIZE], file->end - file->start + 1);
    free(buffer);
    free(line);
    return 0;
}

// Loading .bin files
// Create a buffer to hold the file
// Parse the filename to get the starting address and program name
// Read binary file into buffer
// Write the file to RAM starting from the starting address
// delete the buffer
// Return 0 on success

int loadBin(tapeFile* file) {
    FATFS fs;
    FIL fileHandle;
    UINT bytesRead;
    uint8_t* buffer = malloc(MAX_TAPE_SIZE);
    FRESULT result = f_mount(&fs, SDPath, 1);
    char* line = malloc(64);

    if (result != FR_OK) {
        return -1;
    }
    // open file and read in the data (up to maximum tape size)
    f_open(&fileHandle, file->fullName, FA_READ | FA_OPEN_EXISTING);
		f_read(&fileHandle, buffer, MAX_TAPE_SIZE, &bytesRead);
		sprintf(line, "Bytes read: %d", bytesRead);
		writelineTerminal(line);
    f_close(&fileHandle);
    // copy into ram at the specified start & end address
    memcpy(&RAM[file->start], buffer, file->end - file->start + 1);
    free(buffer);
    free(line);
    return 0;
}

// tapeLoading main function
// Check if the file is .basic or .bin
// Call the appropriate function
// writelineTerminal to show info and how to run the program (1. [starting address]R) if .bin
// writelineTerminal to show info and how to run the program (1. E2B3R, 2. RUN) if .basic

void tapeLoading(char* filename) {
    tapeFile* file = parseFilename(filename);
    if (strcmp(file->type, "basic") == 0) {
        writelineTerminal("Loading BASIC program...");
        int result = loadBasic(file);
        if (result != 0) {
            writelineTerminal("Error loading file.");
            return;
        }
        writelineTerminal("Program loaded.");
        writelineTerminal("To run, type:");
        writelineTerminal("1. E2B3R, 2. RUN");
    } else if (strcmp(file->type, "bin") == 0) {
        char* line = malloc(64);
        writelineTerminal("Loading binary program...");
        sprintf(line, "Program: %s", file->name);
        writelineTerminal(line);
        sprintf(line, "Start: %04XR", file->start);
        writelineTerminal(line);
        sprintf(line, "End: %04XR", file->end);
        writelineTerminal(line);
        int result = loadBin(file);
        if (result != 0) {
            writelineTerminal("Error loading file.");
            return;
        }
        writelineTerminal("Program loaded.");
        writelineTerminal("To run, type:");
        sprintf(line, "1. %04XR", file->start);
        writelineTerminal(line);
        free(line);
    }
    free(file);
}

// Author: David Sin
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include "fatfs.h"
#include "mos6502.h"
#include "main.h"

// External
extern uint8_t RAM[RAM_SIZE];

// 20KB of tape space
#define MAX_TAPE_SIZE 20480
#define BASIC_HEADER_START 0x004A
#define BASIC_HEADER_SIZE 182
// enable debug for file loading
#define LOAD_DEBUG 1
#define TAB "    "

typedef struct {
	TCHAR fullName[64];
    char name[64];
    uint16_t start;
    uint16_t end;
    char type[5];
    uint8_t debug;
} tapeFile;

// Filename parser
// Extract program name and start/end addresses
// Example: matrix.004A.00FF.basic
// Extract matrix, 004A
// Example: apple30th.0280.0FFF.bin
// Extract apple30th, 0280, 0FFF, bin
// return tapeFile struct

// function to match FRESULT to relevant error
char* getResultStatus(FRESULT result) {
    switch (result) {
        case FR_OK:
            return "Succeeded";
        case FR_DISK_ERR:
            return "A hard error occurred in the low level disk I/O layer";
        case FR_INT_ERR:
            return "Assertion failed";
        case FR_NOT_READY:
            return "The physical drive cannot work";
        case FR_NO_FILE:
            return "Could not find the file";
        case FR_NO_PATH:
            return "Could not find the path";
        case FR_INVALID_NAME:
            return "The path name format is invalid";
        case FR_DENIED:
            return "Access denied due to prohibited access or directory full";
        case FR_EXIST:
            return "Access denied due to prohibited access";
        case FR_INVALID_OBJECT:
            return "The file/directory object is invalid";
        case FR_WRITE_PROTECTED:
            return "The physical drive is write protected";
        case FR_INVALID_DRIVE:
            return "The logical drive number is invalid";
        case FR_NOT_ENABLED:
            return "The volume has no work area";
        case FR_NO_FILESYSTEM:
            return "There is no valid FAT volume";
        case FR_MKFS_ABORTED:
            return "The f_mkfs() aborted due to any parameter error";
        case FR_TIMEOUT:
            return "Could not get a grant to access the volume within defined period";
        case FR_LOCKED:
            return "The operation is rejected according to the file sharing policy";
        case FR_NOT_ENOUGH_CORE:
            return "LFN working buffer could not be allocated";
        case FR_TOO_MANY_OPEN_FILES:
            return "Number of open files > _FS_SHARE";
        case FR_INVALID_PARAMETER:
            return "Given parameter is invalid";
        default:
            return "Unknown FRESULT";
    }
}

tapeFile* parseFilename(char* filename) {
    tapeFile* file = malloc(sizeof(tapeFile));
    file->debug = LOAD_DEBUG;
    strcpy(file->fullName, filename);
    char* token = strtok(filename, ".");
    strcpy(file->name, token);
    token = strtok(NULL, ".");
    file->start = strtol(token, NULL, 16);
    token = strtok(NULL, ".");
    file->end = strtol(token, NULL, 16);
    token = strtok(NULL, ".");
    char line[128] = { 0x00 };
    // lowercase for pattern matching
    int i;
    for(i = 0; token[i]; i++)
    {
    	file->type[i] = tolower(token[i]);
    }
    // set last char to null term
    file->type[i] = '\0';
    // debug output
    if(file->debug) {
    	writelineTerminal("===File Load Debug===");
    	writelineTerminal("Parsed File Vars:");
    	sprintf(line, "%s fullName - '%s'", TAB, file->fullName);
    	writelineTerminal(line);
    	sprintf(line, "%s name - '%s'", TAB, file->name);
    	writelineTerminal(line);
    	sprintf(line, "%s start - '%u'", TAB, file->start);
    	writelineTerminal(line);
    	sprintf(line, "%s end - '%u'", TAB, file->end);
    	writelineTerminal(line);
    	sprintf(line, "%s Unstandardized Type - '%s'", TAB, token);
    	writelineTerminal(line);
    	sprintf(line, "%s Standardized Type - '%s'", TAB, file->type);
    	writelineTerminal(line);
    }
    return file;
}

void printFileLoadInfo(tapeFile* file, char* line){
	sprintf(line, "Program: %s", file->name);
	writelineTerminal(line);
	sprintf(line, "Start: %04XR", file->start);
	writelineTerminal(line);
	sprintf(line, "End: %04XR", file->end);
	writelineTerminal(line);
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
        // mounting error catch
	if(result != FR_OK){
		char* error = getResultStatus(result);
		writelineTerminal("Error Mounting File:");
		writelineTerminal(error);
		return -1;
	}
	// open file
	result = f_open(&fileHandle, file->fullName, FA_READ | FA_OPEN_EXISTING);
	// open error catch
	if(result != FR_OK){
		char* error = getResultStatus(result);
		writelineTerminal("Error Opening File:");
		writelineTerminal(error);
		return -1;
	}
		// read the data (up to maximum tape size)
		result = f_read(&fileHandle, buffer, MAX_TAPE_SIZE, &bytesRead);
		// read error catch
		if(result != FR_OK){
			char* error = getResultStatus(result);
			writelineTerminal("Error Reading File:");
			writelineTerminal(error);
			return -1;
		}
		sprintf(line, "Bytes read: %d", bytesRead);
		writelineTerminal(line);
    f_close(&fileHandle);
    // copy first 'BASIC_HEADER_SIZE' worth of data from the basic header start address
    // the rest from the file's provided start address
    memcpy(&RAM[BASIC_HEADER_START], buffer, BASIC_HEADER_SIZE);
    memcpy(&RAM[file->start], &buffer[BASIC_HEADER_SIZE], file->end - file->start + 1);
    // Buzzer beep from buffer using buzzerTone(uint8_t tone)
    for (int i = 0; i < bytesRead; i++) {
        buzzerTone(buffer[i]);
    }
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
    // mounting error catch
	if(result != FR_OK){
		char* error = getResultStatus(result);
		writelineTerminal("Error Mounting File:");
		writelineTerminal(error);
		return -1;
	}

    // open file and read in the data (up to maximum tape size)
    result = f_open(&fileHandle, file->fullName, FA_READ | FA_OPEN_EXISTING);
    // open error catch
	if(result != FR_OK){
		char* error = getResultStatus(result);
		writelineTerminal("Error Opening File:");
		writelineTerminal(error);
		return -1;
	}
		result = f_read(&fileHandle, buffer, MAX_TAPE_SIZE, &bytesRead);
		// read error catch
		if(result != FR_OK){
			char* error = getResultStatus(result);
			writelineTerminal("Error Reading File:");
			writelineTerminal(error);
			return -1;
		}
		sprintf(line, "Bytes read: %d", bytesRead);
		writelineTerminal(line);
    f_close(&fileHandle);
    // copy into ram at the specified start & end address
    memcpy(&RAM[file->start], buffer, file->end - file->start + 1);
    // Buzzer beep from buffer using buzzerTone(uint8_t tone)
    for (int i = 0; i < bytesRead; i++) {
        buzzerTone(buffer[i]);
    }
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
    char* line = malloc(128);
    // 'basic' files
    if (strcmp(file->type, "basic") == 0) {
        writelineTerminal("Loading BASIC program...");
        printFileLoadInfo(file, line);
        int result = loadBasic(file);
        if (result != 0) {
        	writelineTerminal("Terminating File Load...");
            return;
        }
        writelineTerminal("Program loaded.");
        writelineTerminal("To run, type:");
        writelineTerminal("1. E2B3R, 2. RUN");
    // 'bin' files
    } else if (strcmp(file->type, "bin") == 0) {
        writelineTerminal("Loading binary program...");
        printFileLoadInfo(file, line);
        int result = loadBin(file);
        if (result != 0) {
        	writelineTerminal("Terminating File Load...");
            return;
        }
        writelineTerminal("Program loaded.");
        writelineTerminal("To run, type:");
        sprintf(line, "1. %04XR", file->start);
        writelineTerminal(line);
        free(line);
    }
    // unrecognized type
    else{
        sprintf(line, "File Type: '%s' is not supported.", file->type);
    	writelineTerminal(line);
    	writelineTerminal("Terminating File Load...");
    	free(line);
    }
    free(file);
}



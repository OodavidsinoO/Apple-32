// ROM
#include "basic_0xE000.h"
#include "wozMonitor_0xFF00.h"
#include "apple1Assembler_0xE000.h"

// RAM
#define RAM_SIZE 0x8000
// Assembler Selection, 0 = The A1-Assembler, 1 = Original Apple 1 BASIC
#define ASSEMBLER 1
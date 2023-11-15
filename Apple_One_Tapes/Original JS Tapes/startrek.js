/*
C100R
004A.00FFR 0300.0FFFR
E2B3R
RUN
*/

tapes['Star Trek'] = {
    script: 'C100R\n004A.00FFR 0300.0FFFR\nE2B3R\nRUN\n',
    tracks: [[ // 004A.00FF
        0x00,0x03,0x00,0x10,0x3A,0xCC,
        0x05,0xBF,0xFD,0xBF,0xBF,0xBF,0xF8,0xF8,
        0xBF,0xB7,0x7F,0xBF,0xBF,0xB7,0xFF,0x21,
        0xF7,0xBF,0x3F,0x3F,0x3F,0xBF,0x77,0x10,
        0x60,0x3C,0x02,0x0A,0x01,0xFB,0x01,0x3B,
        0xED,0xED,0xED,0xED,0xED,0xEC,0xEC,0xEC,
        0xFF,0xF7,0xB7,0xF7,0xFF,0xB7,0xFF,0x35,
        0x17,0x7F,0xF7,0xF7,0xBD,0xB7,0xDF,0xF7,
        0xB7,0xB7,0xF7,0xF7,0xAE,0xA0,0xD0,0x00,
        0x08,0x08,0x00,0x00,0x05,0x00,0x09,0x03,
        0x59,0x7D,0x82,0x7D,0x80,0x53,0x5B,0x61,
        0x77,0x37,0xF7,0x75,0x35,0x77,0xF1,0x37,
        0x77,0x57,0xF7,0x71,0xCD,0xF7,0xF7,0xB7,
        0xFF,0x97,0xB7,0x97,0xB2,0xB7,0xB7,0x00,
        0xF6,0xF7,0x00,0x00,0x00,0xFF,0x00,0x00,
        0x21,0x08,0x07,0xFF,0xFF,0xFF,0xFF,0xFF,
        0xFF,0x02,0x30,0x04,0x2D,0x04,0x1F,0xFA,
        0x37,0x03,0x2D,0x04,0x00,0xFF,0x03,0x02,
        0x20,0xFE,0x2B,0x05,0x25,0x05,0xFF,0xF7,
        0x37,0x05,0x30,0x04,0x25,0x05,0x54,0x05,
        0x65,0x64,0x0E,0x0C,0x03,0x00,0x00,0x00,
        0x00,0x00,0x63,0x00,0x77,0x75,0xF7,0x25,
        0x00,0x00,0x00,0x00,0x00,0x1F,0x00,0xFF
    ],
    [ // 0300.0FFF
        0x86,0x40,0x0B,0x03,0xA0,0xAD,0xAD,0xAD,
        0x1E,0xAD,0x1E,0x88,0x40,0x1F,0x03,0xA0,
        0xA0,0xA0,0xA0,0xAA,0xA0,0xBE,0xA1,0xBC,
        0xAB,0xAB,0xAB,0xBC,0xAA,0xBE,0x1E,0xA2,
        0x00,0x25,0x03,0x80,0x02,0xA6,0x00,0x2B,
        0x03,0xC0,0x02,0xA4,0x00,0x31,0x03,0x07,
        0x00,0x94,0x00,0x37,0x03,0x09,0x00,0x86,
        0x00,0x3D,0x03,0x63,0x00,0x84,0x00,0x43,
        0x03,0xC0,0x02,0x96,0x00,0x49,0x03,0x02,
        0x00,0xA6,0xB1,0x4F,0x03,0x02,0x00,0x9C,
        0x00,0x55,0x03,0x08,0x00,0xAC,0x00,0x5B,
        0x03,0x02,0x00,0x98,0x00,0x61,0x03,0x03,
        0x00,0x92,0x00,0x67,0x03,0x3A,0x00,0xA0,
        0x00,0x6D,0x03,0x01,0x00,0x84,0xB1,0x73,
        0x03,0x02,0x00,0x96,0xB1,0x79,0x03,0x03,
        0x00,0xA8,0x00,0x7F,0x03,0x06,0x00,0x8A,
        0xB1,0x85,0x03,0x78,0x00,0x8A,0xB2,0x8B,
        0x03,0x01,0x00,0x8A,0xB0,0x91,0x03,0x12,
        0x02,0xB2,0xB0,0x97,0x03,0x08,0x00,0xB0,
        0xB0,0x9D,0x03,0x12,0x00,0xB2,0xB1,0xA3,
        0x03,0x01,0x00,0xB0,0xB1,0xA9,0x03,0x02,
        0x00,0xB2,0xB2,0xAF,0x03,0x00,0x00,0xB0,
        0xB2,0xB5,0x03,0x02,0x00,0xA2,0xB0,0xBB,
        0x03,0x0A,0x00,0xA6,0xB0,0xC1,0x03,0x02,
        0x00,0xB0,0xB5,0xC7,0x03,0x02,0x00,0xB2,
        0xB5,0xCD,0x03,0x07,0x00,0xA2,0xB9,0xD3,
        0x03,0x07,0x00,0x86,0xB9,0xD9,0x03,0x01,
        0x00,0xA6,0xB2,0xDF,0x03,0x01,0x00,0x96,
        0xB2,0xE5,0x03,0x00,0x00,0x84,0xB2,0xEB,
        0x03,0x00,0x00,0x96,0xB5,0xF1,0x03,0xC1,
        0xFF,0xB2,0xB6,0xF7,0x03,0x07,0x00,0xB0,
        0xB6,0xFD,0x03,0x02,0x00,0x84,0xB8,0x03,
        0x04,0x00,0x00,0x96,0xB8,0x09,0x04,0x00,
        0x00,0xB2,0x00,0x0F,0x04,0xFF,0xFF,0x8C,
        0xB2,0x15,0x04,0x00,0x00,0xB0,0x00,0x1B,
        0x04,0x08,0x00,0x8C,0xB1,0x21,0x04,0x00,
        0x00,0xA2,0xB1,0x27,0x04,0x0B,0x00,0x94,
        0xB5,0x2D,0x04,0x02,0x00,0x0A,0x00,0x4E,
        0x14,0x0A,0x00,0x4E,0xC3,0x40,0x22,0xB6,
        0x06,0x00,0x72,0x43,0xC4,0x40,0x22,0xB1,
        0x0F,0x00,0x72,0x01,0x24,0x14,0x00,0xD1,
        0x71,0xB6,0x80,0x02,0x03,0xD3,0x71,0xB7,
        0xC0,0x02,0x03,0xC4,0x40,0x70,0x28,0xA0,
        0xA0,0xA0,0xA0,0xAA,0xA0,0xBE,0xA1,0xBC,
        0xAB,0xAB,0xAB,0xBC,0xAA,0xBE,0x29,0x01,
        0x17,0x1E,0x00,0x53,0x28,0xD4,0xD9,0xD0,
        0xC5,0xA0,0xC1,0xA0,0xCE,0xD5,0xCD,0xC2,
        0xC5,0xD2,0xA0,0x29,0x27,0xD2,0x01,0x18,
        0x28,0x00,0x55,0xCA,0x56,0xB1,0x01,0x00,
        0x57,0xD2,0x03,0xC3,0x71,0x2F,0x3F,0xB2,
        0x00,0x01,0x72,0x03,0x59,0xCA,0x01,0x0C,
        0x32,0x00,0xC2,0x71,0xD1,0x03,0x5C,0xB1,
        0x4C,0x04,0x01,0x20,0x3C,0x00,0xD3,0xB1,
        0x71,0xB7,0x4B,0x00,0x03,0xCE,0x71,0xD3,
        0xB1,0x03,0xD6,0x71,0xB1,0x01,0x00,0x03,
        0xCC,0x71,0xB1,0x0A,0x00,0x03,0x5C,0xB1,
        0xB0,0x04,0x01,0x20,0x46,0x00,0xC2,0xB1,
        0x71,0xB2,0x02,0x00,0x03,0xCE,0x71,0xC2,
        0xB1,0x03,0xD6,0x71,0xB1,0x0A,0x00,0x03,
        0xCC,0x71,0xB2,0x14,0x00,0x03,0x5C,0xB1,
        0xB0,0x04,0x01,0x20,0x50,0x00,0xCB,0xB1,
        0x71,0xB7,0x07,0x00,0x03,0xCE,0x71,0xCB,
        0xB1,0x03,0xD6,0x71,0xB2,0x14,0x00,0x03,
        0xCC,0x71,0xB4,0x28,0x00,0x03,0x5C,0xB1,
        0xB0,0x04,0x01,0x22,0x5A,0x00,0xD4,0x71,
        0xB1,0x0F,0x00,0x03,0x5C,0xB1,0x14,0x05,
        0x03,0xC5,0xB0,0x71,0x2F,0x3F,0xB4,0x00,
        0x10,0x72,0x03,0x5C,0xB1,0x78,0x05,0x03,
        0x5C,0xB1,0xE8,0x03,0x01,0x2F,0x6E,0x00,
        0x63,0x03,0x53,0x28,0xC3,0xCF,0xCD,0xCD,
        0xC1,0xCE,0xC4,0xA0,0x29,0x27,0xC3,0x03,
        0x60,0xC3,0x1C,0xB0,0x00,0x00,0x1E,0xC3,
        0x19,0xB5,0x05,0x00,0x24,0xB1,0x6E,0x00,
        0x03,0x5F,0xB1,0x64,0x00,0x14,0xC3,0x12,
        0xB2,0xC8,0x00,0x01,0x32,0xC8,0x00,0x53,
        0x28,0xD6,0xC5,0xC3,0xD4,0xCF,0xD2,0xA0,
        0x29,0x27,0xD8,0x27,0xD9,0x03,0xD8,0xB0,
        0x71,0xD8,0xB0,0x12,0xD8,0x03,0xD9,0xB0,
        0x71,0xD9,0xB0,0x12,0xD9,0x03,0xC5,0xB1,
        0x71,0xC5,0xB1,0x13,0x31,0x3F,0xD8,0x72,
        0x13,0x31,0x3F,0xD9,0x72,0x01,0x24,0xD2,
        0x00,0x60,0xD8,0xB0,0x1C,0xB0,0x00,0x00,
        0x1E,0xD8,0xB0,0x19,0xB6,0x3F,0x00,0x1E,
        0xD9,0xB0,0x1C,0xB0,0x00,0x00,0x1E,0xD9,
        0xB0,0x19,0xB6,0x3F,0x00,0x24,0xB2,0xFA,
        0x00,0x01,0x2D,0xDC,0x00,0xC5,0xB0,0x71,
        0xC5,0xB0,0x12,0xD8,0x12,0xB6,0x40,0x00,
        0x14,0xD9,0x03,0xD1,0xB1,0x71,0xD1,0xB0,
        0x03,0xD3,0xB1,0x71,0xD3,0xB0,0x03,0x5C,
        0xB1,0x78,0x05,0x03,0x60,0xD1,0xB0,0x16,
        0xD1,0xB1,0x24,0xB2,0xFF,0x00,0x01,0x2A,
        0xE6,0x00,0xC5,0xB1,0x71,0xC5,0xB1,0x13,
        0xB2,0x19,0x00,0x03,0xD4,0x71,0xD4,0x13,
        0xB1,0x01,0x00,0x03,0x5C,0xB1,0xE8,0x03,
        0x03,0x5C,0xB1,0x78,0x05,0x03,0x60,0xD4,
        0x18,0xB0,0x00,0x00,0x24,0xB2,0x04,0x01,
        0x01,0x12,0xF0,0x00,0xC3,0x40,0x70,0x28,
        0xD4,0xC9,0xCD,0xC5,0x29,0x03,0x5F,0xB9,
        0xD4,0x03,0x01,0x14,0xFA,0x00,0xC3,0x40,
        0x70,0x28,0xC7,0xC1,0xCC,0xC1,0xD8,0xD9,
        0x29,0x03,0x5F,0xB9,0xD4,0x03,0x01,0x12,
        0xFF,0x00,0x64,0xD3,0x12,0xD3,0xB1,0x65,
        0xB0,0x00,0x00,0x03,0x5C,0xB1,0x1A,0x04,
        0x01,0x2F,0x04,0x01,0x5C,0xB1,0x08,0x07,
        0x03,0x60,0xC2,0xB2,0x16,0xB0,0x00,0x00,
        0x24,0xB2,0x22,0x01,0x03,0x60,0x31,0x3F,
        0xD8,0xB6,0x13,0xD8,0xB2,0x72,0x12,0x31,
        0x3F,0xD9,0xB6,0x13,0xD9,0xB2,0x72,0x17,
        0xB1,0x01,0x00,0x24,0xB2,0x22,0x01,0x01,
        0x27,0x0E,0x01,0x61,0x28,0xAD,0xA0,0xC4,
        0xCF,0xC3,0xCB,0xC5,0xC4,0xA0,0xAD,0x29,
        0x03,0x5C,0xB1,0x14,0x05,0x03,0xC2,0xB1,
        0x71,0xC2,0xB1,0x13,0xB1,0x01,0x00,0x03,
        0xC2,0xB2,0x71,0xB0,0x00,0x00,0x01,0x2B,
        0x18,0x01,0x64,0xD1,0x12,0xD1,0xB0,0x65,
        0x2E,0x3F,0xD1,0x12,0xD1,0xB0,0x72,0x13,
        0xB1,0x0A,0x00,0x03,0x64,0xD3,0x12,0xD8,
        0xB6,0x12,0xB8,0x08,0x00,0x14,0xD9,0xB6,
        0x65,0xB0,0x00,0x00,0x03,0x5F,0xB1,0x6E,
        0x00,0x01,0x15,0x22,0x01,0x60,0xCB,0xB2,
        0x17,0xB0,0x00,0x00,0x25,0x5C,0xB5,0x26,
        0x02,0x03,0x5F,0xB1,0x6E,0x00,0x01,0x1D,
        0x2C,0x01,0xC3,0x40,0x70,0x28,0xD3,0xC8,
        0xCF,0xD2,0xD4,0x29,0x03,0x5C,0xB3,0x5E,
        0x01,0x03,0x5C,0xB2,0xD0,0x07,0x03,0x5F,
        0xB1,0x6E,0x00,0x01,0x23,0x5E,0x01,0x61,
        0xC3,0x40,0x45,0x28,0xA0,0xD2,0xC1,0xCE,
        0xC7,0xC5,0xA0,0xD3,0xC5,0xCE,0xD3,0xCF,
        0xD2,0xA0,0xD3,0xC3,0xC1,0xCE,0x29,0x03,
        0x5C,0xB3,0x68,0x01,0x03,0x5B,0x01,0x20,
        0x68,0x01,0x61,0x28,0xC6,0xCF,0xD2,0xA0,
        0xD1,0xD5,0xC1,0xC4,0xD2,0xC1,0xCE,0xD4,
        0xA0,0x29,0x46,0xD8,0xB1,0x45,0x28,0xAC,
        0x29,0x46,0xD9,0xB1,0x03,0x5B,0x01,0x1D,
        0x90,0x01,0xC3,0x40,0x70,0x28,0xCC,0xCF,
        0xCE,0xC7,0x29,0x03,0x5C,0xB3,0x5E,0x01,
        0x03,0xCE,0x71,0xB3,0x03,0x00,0x03,0x5C,
        0xB8,0x70,0x03,0x01,0x3E,0x9A,0x01,0x55,
        0xD9,0x56,0xD9,0xB1,0x12,0xB1,0x01,0x00,
        0x57,0xD9,0xB1,0x13,0xB1,0x01,0x00,0x58,
        0x36,0xB1,0x01,0x00,0x03,0xC6,0xB2,0x71,
        0xB0,0x00,0x00,0x03,0x60,0xD9,0x1C,0xB0,
        0x00,0x00,0x1E,0xD9,0x19,0xB7,0x07,0x00,
        0x25,0xC6,0xB2,0x71,0xB1,0x01,0x00,0x03,
        0x5C,0xB4,0xD6,0x01,0x03,0x5C,0xB4,0xE0,
        0x01,0x01,0x2F,0xA4,0x01,0x55,0xD8,0x56,
        0xD8,0xB1,0x13,0xB1,0x01,0x00,0x57,0xD8,
        0xB1,0x12,0xB1,0x01,0x00,0x03,0xC6,0xB1,
        0x71,0xB0,0x00,0x00,0x03,0x60,0xD8,0x1C,
        0xB0,0x00,0x00,0x1E,0xD8,0x19,0xB7,0x07,
        0x00,0x25,0xC6,0xB1,0x71,0xB1,0x01,0x00,
        0x01,0x20,0xAE,0x01,0x60,0xC6,0xB1,0x16,
        0xB0,0x00,0x00,0x1D,0xC6,0xB2,0x16,0xB0,
        0x00,0x00,0x24,0xB4,0xB8,0x01,0x03,0x61,
        0xC3,0x40,0x47,0x03,0x5F,0xB4,0xCC,0x01,
        0x01,0x34,0xB8,0x01,0xD1,0xB9,0x71,0xD8,
        0x12,0xB8,0x08,0x00,0x14,0xD9,0x03,0x5C,
        0xB1,0xDC,0x05,0x03,0x64,0xD1,0x12,0xD1,
        0xB9,0x65,0xC3,0xB9,0x12,0xB4,0x28,0x00,
        0x03,0x61,0x28,0xA1,0xA0,0x29,0x46,0xCB,
        0xB2,0x46,0xC2,0xB2,0x46,0xD3,0xB2,0x45,
        0x28,0xA0,0x29,0x47,0x01,0x22,0xCC,0x01,
        0x59,0xD8,0x03,0x61,0x28,0xA1,0x29,0x03,
        0x5C,0xB4,0xE0,0x01,0x03,0x59,0xD9,0x03,
        0x5C,0xB4,0xD6,0x01,0x03,0x5C,0xB8,0x7A,
        0x03,0x03,0x5F,0xB1,0x6E,0x00,0x01,0x16,
        0xD6,0x01,0xC3,0x40,0x70,0x28,0xAB,0xAD,
        0xAD,0xAD,0xAD,0xAD,0x29,0x03,0x5C,0xB1,
        0x40,0x06,0x03,0x5B,0x01,0x16,0xE0,0x01,
        0xC3,0x40,0x70,0x28,0xA1,0xA0,0xA0,0xA0,
        0xA0,0xA0,0x29,0x03,0x5C,0xB1,0x40,0x06,
        0x03,0x5B,0x01,0x39,0xF4,0x01,0x60,0xCB,
        0xB2,0x16,0xB0,0x00,0x00,0x24,0xB6,0xB2,
        0x02,0x03,0x5C,0xB7,0x16,0x03,0x03,0x61,
        0x28,0xC5,0xCE,0xC5,0xD2,0xC7,0xD9,0xBA,
        0xA0,0x29,0x46,0xC5,0xB1,0x03,0x53,0x28,
        0xC6,0xC9,0xD2,0xC5,0xA0,0x29,0x27,0xC3,
        0x03,0x60,0xC3,0x1C,0xB1,0x01,0x00,0x24,
        0xB1,0x6E,0x00,0x01,0x36,0x08,0x02,0xC5,
        0xB1,0x71,0xC5,0xB1,0x13,0xC3,0x03,0x5C,
        0xB1,0x08,0x07,0x03,0xCB,0xB5,0x71,0xCB,
        0xB5,0x13,0xC3,0x13,0x38,0xD2,0x14,0xB9,
        0x09,0x00,0x72,0x03,0x60,0xCB,0xB5,0x19,
        0xB0,0x00,0x00,0x24,0xB5,0x12,0x02,0x03,
        0x5C,0xB1,0x6C,0x07,0x03,0x5F,0xB1,0x6E,
        0x00,0x01,0x0D,0x12,0x02,0x5C,0xB5,0x26,
        0x02,0x03,0x5F,0xB5,0xF4,0x01,0x01,0x48,
        0x26,0x02,0xCA,0xB5,0x71,0xCB,0xB5,0x15,
        0xB5,0x05,0x00,0x03,0xCB,0xB5,0x71,0xCB,
        0xB5,0x13,0xCA,0xB5,0x03,0xC5,0xB1,0x71,
        0xC5,0xB1,0x13,0xCA,0xB5,0x15,0xD2,0x03,
        0x62,0xCA,0xB5,0x15,0xD2,0x45,0x28,0xA0,
        0xD5,0xCE,0xC9,0xD4,0xD3,0xA0,0xCF,0xC6,
        0xA0,0xD0,0xC8,0xC1,0xD3,0xC5,0xD2,0xA0,
        0xC4,0xC1,0xCD,0xC1,0xC7,0xC5,0x29,0x03,
        0x5C,0xB1,0x08,0x07,0x03,0x5B,0x01,0x35,
        0x58,0x02,0x60,0xCB,0xB2,0x16,0xB0,0x00,
        0x00,0x24,0xB6,0xB2,0x02,0x03,0x60,0xC5,
        0xB2,0x17,0xB0,0x00,0x00,0x24,0xB6,0x62,
        0x02,0x03,0x61,0x28,0xCE,0xCF,0xA0,0xCD,
        0xCF,0xD2,0xC5,0xA0,0xD4,0xCF,0xD2,0xD0,
        0xC5,0xC4,0xCF,0xC5,0xD3,0x29,0x03,0x5F,
        0xB1,0x6E,0x00,0x01,0x25,0x62,0x02,0xC5,
        0xB2,0x71,0xC5,0xB2,0x13,0xB1,0x01,0x00,
        0x03,0x60,0xD2,0x19,0x2F,0x3F,0xB1,0x0F,
        0x00,0x72,0x24,0xB6,0x80,0x02,0x03,0x5C,
        0xB1,0x6C,0x07,0x03,0x5F,0xB1,0x6E,0x00,
        0x01,0x1C,0x80,0x02,0x61,0x28,0xD9,0xCF,
        0xD5,0xA0,0xCD,0xC9,0xD3,0xD3,0xC5,0xC4,
        0xA1,0x29,0x03,0x5C,0xB5,0x26,0x02,0x03,
        0x5F,0xB1,0x6E,0x00,0x01,0x20,0xB2,0x02,
        0x61,0x28,0xCE,0xCF,0xD4,0xC8,0xC9,0xCE,
        0xC7,0xA0,0xD4,0xCF,0xA0,0xD3,0xC8,0xCF,
        0xCF,0xD4,0xA0,0xC1,0xD4,0xA1,0x29,0x03,
        0x5F,0xB1,0x6E,0x00,0x01,0x25,0xBC,0x02,
        0x53,0x28,0xC3,0xCF,0xCD,0xD0,0xD5,0xD4,
        0xC5,0xD2,0xA0,0xD2,0xC5,0xD1,0xD5,0xC5,
        0xD3,0xD4,0xA0,0x29,0x27,0xC3,0x03,0x60,
        0xC3,0x16,0xB0,0x00,0x00,0x24,0xB8,0x20,
        0x03,0x01,0x17,0xC6,0x02,0x61,0x28,0xD3,
        0xD4,0xC1,0xD4,0xD5,0xD3,0xA0,0xD2,0xC5,
        0xD0,0xCF,0xD2,0xD4,0xBA,0x29,0x03,0x63,
        0x01,0x19,0xD0,0x02,0x5C,0xB7,0x16,0x03,
        0x03,0x61,0x28,0xCB,0xCC,0xC9,0xCE,0xC7,
        0xCF,0xCE,0xD3,0xA0,0xBD,0x29,0x49,0xCB,
        0xB1,0x01,0x19,0xDA,0x02,0x5C,0xB7,0x16,
        0x03,0x03,0x61,0x28,0xD3,0xD4,0xC1,0xD2,
        0xC4,0xC1,0xD4,0xC5,0xD3,0xA0,0xBD,0x29,
        0x49,0xD4,0x01,0x1A,0xE4,0x02,0x5C,0xB7,
        0x16,0x03,0x03,0x61,0x28,0xD3,0xD4,0xC1,
        0xD2,0xC2,0xC1,0xD3,0xC5,0xD3,0xA0,0xBD,
        0x29,0x49,0xC2,0xB1,0x01,0x1A,0xEE,0x02,
        0x5C,0xB7,0x16,0x03,0x03,0x61,0x28,0xD4,
        0xCF,0xD2,0xD0,0xC5,0xC4,0xCF,0xC5,0xD3,
        0xA0,0xBD,0x29,0x49,0xC5,0xB2,0x01,0x17,
        0xF8,0x02,0x5C,0xB7,0x16,0x03,0x03,0x61,
        0x28,0xC5,0xCE,0xC5,0xD2,0xC7,0xD9,0xA0,
        0xBD,0x29,0x49,0xC5,0xB1,0x01,0x08,0x02,
        0x03,0x5F,0xB1,0x6E,0x00,0x01,0x14,0x16,
        0x03,0x61,0x28,0xD2,0xC5,0xCD,0xC1,0xC9,
        0xCE,0xC9,0xCE,0xC7,0xA0,0x29,0x47,0x03,
        0x5B,0x01,0x2F,0x20,0x03,0x61,0x28,0xA0,
        0xC7,0xC1,0xCC,0xC1,0xC3,0xD4,0xC9,0xC3,
        0xA0,0xCD,0xC1,0xD0,0x29,0x03,0x61,0x28,
        0xA0,0x29,0x47,0x03,0x5C,0xB3,0x68,0x01,
        0x03,0xC3,0x40,0x70,0x28,0xA0,0xAD,0xAD,
        0xAD,0x29,0x03,0xCE,0x71,0xB8,0x08,0x00,
        0x01,0x1D,0x2A,0x03,0x5C,0xB8,0x70,0x03,
        0x03,0x55,0xD9,0x56,0xB7,0x07,0x00,0x57,
        0xB0,0x00,0x00,0x58,0x36,0xB1,0x01,0x00,
        0x03,0x5C,0xB1,0x40,0x06,0x01,0x28,0x34,
        0x03,0x55,0xD8,0x56,0xB0,0x00,0x00,0x57,
        0xB7,0x07,0x00,0x03,0xD1,0xB9,0x71,0xD8,
        0x12,0xB8,0x08,0x00,0x14,0xD9,0x03,0x5C,
        0xB1,0xDC,0x05,0x03,0xD0,0x71,0x2E,0x3F,
        0xD1,0x12,0xD1,0xB9,0x72,0x01,0x1C,0x3E,
        0x03,0x60,0xD0,0x19,0xB3,0x27,0x00,0x24,
        0xB8,0x48,0x03,0x03,0x61,0x28,0xA0,0xA0,
        0xA0,0xA0,0x29,0x47,0x03,0x5F,0xB8,0x52,
        0x03,0x01,0x12,0x48,0x03,0x61,0x28,0xA0,
        0x29,0x46,0xCB,0xB2,0x46,0xC2,0xB2,0x46,
        0xD3,0xB2,0x47,0x01,0x08,0x52,0x03,0x59,
        0xD8,0x03,0x63,0x01,0x15,0x5C,0x03,0x59,
        0xD9,0x03,0x5C,0xB1,0x40,0x06,0x03,0x5C,
        0xB8,0x7A,0x03,0x03,0x5F,0xB1,0x6E,0x00,
        0x01,0x11,0x70,0x03,0xC2,0xB8,0x71,0xC2,
        0xB2,0x03,0xCB,0xB8,0x71,0xCB,0xB2,0x03,
        0x5B,0x01,0x11,0x7A,0x03,0xC2,0xB2,0x71,
        0xC2,0xB8,0x03,0xCB,0xB2,0x71,0xCB,0xB8,
        0x03,0x5B,0x01,0x1C,0xD4,0x03,0x63,0x03,
        0x61,0x28,0xAD,0xAD,0xA0,0xCF,0xD5,0xD4,
        0xA0,0xCF,0xC6,0xA0,0x29,0x45,0xC3,0x40,
        0x45,0x28,0xA0,0xAD,0xAD,0x29,0x01,0x21,
        0xDE,0x03,0x61,0x28,0xD9,0xCF,0xD5,0xA0,
        0xCC,0xC5,0xC6,0xD4,0xA0,0x29,0x46,0xCB,
        0xB1,0x45,0x28,0xA0,0xCB,0xCC,0xC9,0xCE,
        0xC7,0xCF,0xCE,0xD3,0x29,0x03,0x51,0x01,
        0x24,0xE8,0x03,0xC2,0x71,0xD3,0x03,0x5C,
        0xB1,0x4C,0x04,0x03,0xD1,0xB9,0x71,0xD1,
        0xB0,0x03,0x5C,0xB1,0xDC,0x05,0x03,0x64,
        0xD1,0x12,0xD1,0xB9,0x65,0xC3,0xB9,0x12,
        0xB4,0x28,0x00,0x01,0x19,0xF2,0x03,0xCE,
        0x71,0xD3,0xB2,0x03,0xD6,0x71,0xB1,0x01,
        0x00,0x03,0xCC,0x71,0xB2,0x02,0x00,0x03,
        0x5C,0xB1,0xB0,0x04,0x01,0x35,0xFC,0x03,
        0xCE,0x71,0xCB,0xB2,0x03,0xD6,0x71,0xB3,
        0x03,0x00,0x03,0xCC,0x71,0xB4,0x04,0x00,
        0x03,0x5C,0xB1,0xB0,0x04,0x03,0xCB,0xB5,
        0x71,0xB1,0x64,0x00,0x03,0xD9,0xB5,0x71,
        0xC9,0x15,0xB8,0x08,0x00,0x03,0xD8,0xB5,
        0x71,0xC9,0x13,0xB8,0x08,0x00,0x14,0xD9,
        0xB5,0x01,0x2E,0x06,0x04,0xCE,0x71,0xC2,
        0xB2,0x03,0xD6,0x71,0xB2,0x02,0x00,0x03,
        0xCC,0x71,0xB3,0x03,0x00,0x03,0x5C,0xB1,
        0xB0,0x04,0x03,0xD9,0xB6,0x71,0xC9,0x15,
        0xB8,0x08,0x00,0x03,0xD8,0xB6,0x71,0xC9,
        0x13,0xB8,0x08,0x00,0x14,0xD9,0xB6,0x01,
        0x4A,0x0B,0x04,0xC3,0x40,0x70,0x28,0xC7,
        0xD2,0xC5,0xC5,0xCE,0x29,0x03,0x60,0xCB,
        0xB2,0x17,0xB0,0x00,0x00,0x25,0xC3,0x40,
        0x70,0x28,0xD2,0xC5,0xC4,0xA1,0x29,0x03,
        0x63,0x03,0x61,0x28,0xD3,0xD4,0xC1,0xD2,
        0xC4,0xC1,0xD4,0xC5,0xBA,0xA0,0x29,0x46,
        0xB3,0xCE,0x0C,0x13,0xD4,0x45,0x28,0xAC,
        0xA0,0xA0,0xC3,0xCF,0xCE,0xC4,0xC9,0xD4,
        0xC9,0xCF,0xCE,0xBA,0xA0,0x29,0x45,0xC3,
        0x40,0x01,0x33,0x10,0x04,0x61,0x28,0xD1,
        0xD5,0xC1,0xC4,0xD2,0xC1,0xCE,0xD4,0xA0,
        0x29,0x46,0xD8,0xB1,0x45,0x28,0xAC,0x29,
        0x46,0xD9,0xB1,0x45,0x28,0xA0,0xA0,0xAD,
        0xA0,0xA0,0xD3,0xC5,0xC3,0xD4,0xCF,0xD2,
        0xA0,0x29,0x46,0xD8,0xB2,0x45,0x28,0xAC,
        0x29,0x46,0xD9,0xB2,0x01,0x20,0x1A,0x04,
        0xD0,0x71,0x2E,0x3F,0xD3,0x12,0xD3,0xB0,
        0x72,0x03,0x64,0xD3,0x12,0xD3,0xB0,0x65,
        0xB4,0x04,0x00,0x03,0x60,0xD0,0x16,0xB0,
        0x00,0x00,0x25,0x5B,0x01,0x1F,0x24,0x04,
        0x63,0x03,0x61,0x28,0xAA,0xAA,0xAA,0xA0,
        0xC3,0xCF,0xCC,0xCC,0xC9,0xD3,0xC9,0xCF,
        0xCE,0xA0,0xD7,0xC9,0xD4,0xC8,0xA0,0xC1,
        0xA0,0x29,0x47,0x01,0x3A,0x2E,0x04,0x60,
        0xD0,0x16,0xB1,0x01,0x00,0x25,0x61,0x28,
        0xD3,0xD4,0xC1,0xD2,0x29,0x47,0x03,0x60,
        0xD0,0x16,0xB2,0x02,0x00,0x25,0x61,0x28,
        0xD3,0xD4,0xC1,0xD2,0xC2,0xC1,0xD3,0xC5,
        0x29,0x47,0x03,0x60,0xD0,0x16,0xB3,0x03,
        0x00,0x25,0x61,0x28,0xCB,0xCC,0xC9,0xCE,
        0xC7,0xCF,0xCE,0x29,0x47,0x01,0x21,0x38,
        0x04,0x61,0x28,0xA0,0xAA,0xAA,0xAA,0x29,
        0x03,0x60,0xD0,0x16,0xB3,0x03,0x00,0x25,
        0xCB,0xB1,0x71,0xCB,0xB1,0x13,0xB1,0x01,
        0x00,0x03,0x5F,0xB9,0xDE,0x03,0x01,0x1C,
        0x4C,0x04,0x55,0xCB,0x56,0xB0,0x00,0x00,
        0x57,0xB6,0x3F,0x00,0x03,0x64,0xC2,0x12,
        0xCB,0x65,0xB0,0x00,0x00,0x03,0x59,0xCB,
        0x03,0x5B,0x01,0x15,0xB0,0x04,0x60,0xCE,
        0x16,0xB0,0x00,0x00,0x25,0x5B,0x03,0x55,
        0xCB,0x56,0xB1,0x01,0x00,0x57,0xCE,0x01,
        0x2E,0xBA,0x04,0xC9,0x71,0x2F,0x3F,0xB6,
        0x40,0x00,0x72,0x03,0xD0,0x71,0x2E,0x3F,
        0xC2,0x12,0xC9,0x72,0x03,0x60,0xD0,0x12,
        0xD6,0x18,0xCC,0x24,0xB1,0xBA,0x04,0x03,
        0x64,0xC2,0x12,0xC9,0x65,0xD0,0x12,0xD6,
        0x03,0x59,0xCB,0x03,0x5B,0x01,0x13,0x14,
        0x05,0xC5,0xB1,0x71,0xB5,0xF4,0x01,0x03,
        0xC5,0xB2,0x71,0xB3,0x03,0x00,0x03,0x5B,
        0x01,0x48,0x78,0x05,0xD9,0xB0,0x71,0xC5,
        0xB0,0x15,0xB6,0x40,0x00,0x03,0xD8,0xB0,
        0x71,0xC5,0xB0,0x13,0xB6,0x40,0x00,0x14,
        0xD9,0xB0,0x03,0xD9,0xB1,0x71,0xD9,0xB0,
        0x15,0xB8,0x08,0x00,0x03,0xD8,0xB1,0x71,
        0xD8,0xB0,0x15,0xB8,0x08,0x00,0x03,0xD9,
        0xB2,0x71,0xD9,0xB0,0x13,0xB8,0x08,0x00,
        0x14,0xD9,0xB1,0x03,0xD8,0xB2,0x71,0xD8,
        0xB0,0x13,0xB8,0x08,0x00,0x14,0xD8,0xB1,
        0x01,0x33,0x82,0x05,0xD1,0xB0,0x71,0xD8,
        0xB1,0x12,0xB8,0x08,0x00,0x14,0xD9,0xB1,
        0x03,0xD3,0xB0,0x71,0xD8,0xB2,0x12,0xB8,
        0x08,0x00,0x14,0xD9,0xB2,0x03,0xD2,0x71,
        0x31,0x3F,0xD8,0xB5,0x13,0xD8,0xB2,0x72,
        0x12,0x31,0x3F,0xD9,0xB5,0x13,0xD9,0xB2,
        0x72,0x03,0x5B,0x01,0x20,0xDC,0x05,0xC3,
        0xB9,0x71,0x2E,0x3F,0xD1,0x12,0xD1,0xB9,
        0x72,0x03,0x60,0xC3,0xB9,0x18,0xB4,0x28,
        0x00,0x25,0xC3,0xB9,0x71,0xC3,0xB9,0x13,
        0xB4,0x28,0x00,0x01,0x38,0xE6,0x05,0xD3,
        0xB2,0x71,0xC3,0xB9,0x13,0xB1,0x0A,0x00,
        0x14,0x38,0xC3,0xB9,0x15,0xB1,0x0A,0x00,
        0x72,0x03,0xCB,0xB2,0x71,0xC3,0xB9,0x15,
        0xB2,0x14,0x00,0x03,0xC2,0xB2,0x71,0x38,
        0xC3,0xB9,0x13,0xD3,0xB2,0x13,0xB2,0x14,
        0x00,0x14,0xCB,0xB2,0x72,0x15,0xB1,0x0A,
        0x00,0x03,0x5B,0x01,0x23,0x40,0x06,0x55,
        0xCA,0x56,0xB1,0x01,0x00,0x57,0xCE,0x03,
        0x61,0xC3,0x40,0x47,0x03,0x59,0xCA,0x03,
        0x61,0xC3,0x40,0x2A,0xB1,0x01,0x00,0x23,
        0xB1,0x01,0x00,0x72,0x03,0x5B,0x01,0x0D,
        0x08,0x07,0x60,0xC5,0xB1,0x19,0xB0,0x00,
        0x00,0x25,0x5B,0x01,0x14,0x12,0x07,0xC3,
        0x40,0x70,0x28,0xC5,0xCE,0xC5,0xD2,0xC7,
        0xD9,0x29,0x03,0x5F,0xB9,0xD4,0x03,0x01,
        0x47,0x6C,0x07,0x61,0x28,0xAA,0xAA,0xAA,
        0xA0,0xC2,0xCF,0xCF,0xCD,0xA0,0xAA,0xAA,
        0xAA,0x29,0x03,0xCB,0xB1,0x71,0xCB,0xB1,
        0x13,0xB1,0x01,0x00,0x03,0xCB,0xB2,0x71,
        0xB0,0x00,0x00,0x03,0x64,0xD1,0x12,0xD1,
        0xB0,0x65,0x2E,0x3F,0xD1,0x12,0xD1,0xB0,
        0x72,0x13,0xB2,0x14,0x00,0x03,0x64,0xD3,
        0x12,0xD8,0xB5,0x12,0xB8,0x08,0x00,0x14,
        0xD9,0xB5,0x65,0xB0,0x00,0x00,0x01,0x26,
        0x76,0x07,0x60,0xCB,0xB1,0x25,0x5B,0x03,
        0x63,0x03,0x61,0x28,0xCD,0xC9,0xD3,0xD3,
        0xC9,0xCF,0xCE,0xA0,0xC1,0xC3,0xC3,0xCF,
        0xCD,0xD0,0xCC,0xC9,0xD3,0xC8,0xC5,0xC4,
        0xA1,0x29,0x03,0x51,0x01,0x18,0xD0,0x07,
        0xC3,0x40,0x70,0x28,0xAB,0xAD,0xAD,0xAD,
        0x29,0x03,0xCE,0x71,0xB8,0x08,0x00,0x03,
        0x5C,0xB1,0x40,0x06,0x01,0x1E,0xDA,0x07,
        0x55,0xD9,0x56,0xB7,0x07,0x00,0x57,0xB0,
        0x00,0x00,0x58,0x36,0xB1,0x01,0x00,0x03,
        0x55,0xD8,0x56,0xB0,0x00,0x00,0x57,0xB7,
        0x07,0x00,0x01,0x42,0xE4,0x07,0xD0,0x71,
        0xB3,0x03,0x00,0x14,0x2E,0x3F,0xD3,0x12,
        0xD8,0x12,0xB8,0x08,0x00,0x14,0xD9,0x72,
        0x12,0xB1,0x01,0x00,0x03,0x60,0xD8,0x16,
        0xB0,0x00,0x00,0x25,0x61,0x28,0xA1,0x29,
        0x47,0x03,0x60,0xD8,0x17,0xB0,0x00,0x00,
        0x25,0x61,0x28,0xA0,0x29,0x47,0x03,0x61,
        0xC4,0x40,0x2A,0xD0,0x23,0xD0,0x12,0xB2,
        0x02,0x00,0x72,0x47,0x01,0x26,0xEE,0x07,
        0x59,0xD8,0x03,0x61,0x28,0xA1,0x29,0x03,
        0x60,0xD9,0x16,0xB0,0x00,0x00,0x24,0xB2,
        0xF8,0x07,0x03,0x61,0x28,0xAB,0x29,0x47,
        0x03,0x50,0xB3,0x21,0x00,0x03,0x61,0x28,
        0xAB,0x29,0x01,0x0D,0xF8,0x07,0x59,0xD9,
        0x03,0x5C,0xB1,0x40,0x06,0x03,0x5B,0xFF
    ]]
};
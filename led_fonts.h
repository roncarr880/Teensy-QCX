// fonts from LCD_Basic library copyright Rinky-Dink Electronics

// bit patterns will need to be converted to 4BPP array to print on ILI9341, 1 bit to 4 bits.
// bit zero of two table entries will be packed high nibble, low nibble, for font width,  Then bit 1, etc.
// then repeat for 2nd half of table and third for big numbers.


const uint8_t MediumNumbers[] =
{
0x0c, 0x10, 0x2d, 0x0d,      // pixels wide, pixels high, base character, total in table
0x00, 0x00, 0x80, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00,  // -
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00,  // .
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // / changed to a space or blank
0x00, 0x7E, 0x3D, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x3D, 0x7E, 0x00, 0x00, 0x3F, 0x5E, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x5E, 0x3F, 0x00,  // 0
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x7E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x3F, 0x00, 0x00, 0x00,  // 1
0x00, 0x00, 0x81, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xBD, 0x7E, 0x00, 0x00, 0x3F, 0x5E, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x40, 0x00, 0x00,  // 2
0x00, 0x00, 0x81, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xBD, 0x7E, 0x00, 0x00, 0x00, 0x40, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x5E, 0x3F, 0x00,  // 3
0x00, 0x7E, 0xBC, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xBC, 0x7E, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x1E, 0x3F, 0x00,  // 4
0x00, 0x7E, 0xBD, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0x81, 0x00, 0x00, 0x00, 0x00, 0x40, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x5E, 0x3F, 0x00,  // 5
0x00, 0x7E, 0xBD, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0x81, 0x00, 0x00, 0x00, 0x3F, 0x5E, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x5E, 0x3F, 0x00,  // 6
0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x3D, 0x7E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x3F, 0x00,  // 7
0x00, 0x7E, 0xBD, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xBD, 0x7E, 0x00, 0x00, 0x3F, 0x5E, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x5E, 0x3F, 0x00,  // 8
0x00, 0x7E, 0xBD, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xBD, 0x7E, 0x00, 0x00, 0x00, 0x40, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x5E, 0x3F, 0x00  // 9
};


const uint8_t BigNumbers[]  =
{
0x0e, 0x18, 0x2d, 0x0d,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // -
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0xe0, 0xe0, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00,   // .
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // /  changed to a space
0x00, 0xfc, 0xfa, 0xf6, 0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0xf6, 0xfa, 0xfc, 0x00, 0x00, 0xef, 0xc7, 0x83, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x83, 0xc7, 0xef, 0x00, 0x00, 0x7f, 0xbf, 0xdf, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xdf, 0xbf, 0x7f, 0x00,   // 0
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xf8, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x83, 0xc7, 0xef, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x3f, 0x7f, 0x00,   // 1
0x00, 0x00, 0x02, 0x06, 0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0xf6, 0xfa, 0xfc, 0x00, 0x00, 0xe0, 0xd0, 0xb8, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x3b, 0x17, 0x0f, 0x00, 0x00, 0x7f, 0xbf, 0xdf, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xc0, 0x80, 0x00, 0x00,   // 2
0x00, 0x00, 0x02, 0x06, 0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0xf6, 0xfa, 0xfc, 0x00, 0x00, 0x00, 0x10, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0xbb, 0xd7, 0xef, 0x00, 0x00, 0x00, 0x80, 0xc0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xdf, 0xbf, 0x7f, 0x00,   // 3
0x00, 0xfc, 0xf8, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xf8, 0xfc, 0x00, 0x00, 0x0f, 0x17, 0x3b, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0xbb, 0xd7, 0xef, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x3f, 0x7f, 0x00,   // 4
0x00, 0xfc, 0xfa, 0xf6, 0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0x06, 0x02, 0x00, 0x00, 0x00, 0x0f, 0x17, 0x3b, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0xb8, 0xd0, 0xe0, 0x00, 0x00, 0x00, 0x80, 0xc0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xdf, 0xbf, 0x7f, 0x00,   // 5
0x00, 0xfc, 0xfa, 0xf6, 0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0x06, 0x02, 0x00, 0x00, 0x00, 0xef, 0xd7, 0xbb, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0xb8, 0xd0, 0xe0, 0x00, 0x00, 0x7f, 0xbf, 0xdf, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xdf, 0xbf, 0x7f, 0x00,   // 6
0x00, 0x00, 0x02, 0x06, 0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0xf6, 0xfa, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x83, 0xc7, 0xef, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x3f, 0x7f, 0x00,   // 7
0x00, 0xfc, 0xfa, 0xf6, 0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0xf6, 0xfa, 0xfc, 0x00, 0x00, 0xef, 0xd7, 0xbb, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0xbb, 0xd7, 0xef, 0x00, 0x00, 0x7f, 0xbf, 0xdf, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xdf, 0xbf, 0x7f, 0x00,   // 8
0x00, 0xfc, 0xfa, 0xf6, 0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0xf6, 0xfa, 0xfc, 0x00, 0x00, 0x0f, 0x17, 0x3b, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0xbb, 0xd7, 0xef, 0x00, 0x00, 0x00, 0x80, 0xc0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xdf, 0xbf, 0x7f, 0x00,   // 9
};


/********     bits rotated and reversed for writeRect1BPP but didn't work as would need to be 8 bit aligned and these are
 *      12 bits wide 
const uint8_t MediumNumbers[] = {
0x0c, 0x10, 0x2d, 0x0d,      // pixels wide, pixels high, base character, total in table
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1F, 0x83, 0xFC, 0x1F, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x60, 0x6, 0x0, 0x0, 
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
0x3F, 0xC5, 0xFA, 0x60, 0x66, 0x6, 0x60, 0x66, 0x6, 0x40, 0x20, 0x0, 0x40, 0x26, 0x6, 0x60, 0x66, 0x6, 0x60, 0x65, 0xFA, 0x3F, 0xC0, 0x0, 
0x0, 0x0, 0x8, 0x1, 0x80, 0x18, 0x1, 0x80, 0x18, 0x0, 0x80, 0x0, 0x0, 0x80, 0x18, 0x1, 0x80, 0x18, 0x1, 0x80, 0x8, 0x0, 0x0, 0x0, 
0x3F, 0xC1, 0xFA, 0x0, 0x60, 0x6, 0x0, 0x60, 0x6, 0x1F, 0xA3, 0xFC, 0x5F, 0x86, 0x0, 0x60, 0x6, 0x0, 0x60, 0x5, 0xF8, 0x3F, 0xC0, 0x0, 
0x3F, 0xC1, 0xFA, 0x0, 0x60, 0x6, 0x0, 0x60, 0x6, 0x1F, 0xA3, 0xFC, 0x1F, 0xA0, 0x6, 0x0, 0x60, 0x6, 0x0, 0x61, 0xFA, 0x3F, 0xC0, 0x0, 
0x0, 0x4, 0x2, 0x60, 0x66, 0x6, 0x60, 0x66, 0x6, 0x5F, 0xA3, 0xFC, 0x1F, 0xA0, 0x6, 0x0, 0x60, 0x6, 0x0, 0x60, 0x2, 0x0, 0x0, 0x0, 
0x3F, 0xC5, 0xF8, 0x60, 0x6, 0x0, 0x60, 0x6, 0x0, 0x5F, 0x83, 0xFC, 0x1F, 0xA0, 0x6, 0x0, 0x60, 0x6, 0x0, 0x61, 0xFA, 0x3F, 0xC0, 0x0, 
0x3F, 0xC5, 0xF8, 0x60, 0x6, 0x0, 0x60, 0x6, 0x0, 0x5F, 0x83, 0xFC, 0x5F, 0xA6, 0x6, 0x60, 0x66, 0x6, 0x60, 0x65, 0xFA, 0x3F, 0xC0, 0x0, 
0x3F, 0xC1, 0xFA, 0x0, 0x60, 0x6, 0x0, 0x60, 0x6, 0x0, 0x20, 0x0, 0x0, 0x20, 0x6, 0x0, 0x60, 0x6, 0x0, 0x60, 0x2, 0x0, 0x0, 0x0, 
0x3F, 0xC5, 0xFA, 0x60, 0x66, 0x6, 0x60, 0x66, 0x6, 0x5F, 0xA3, 0xFC, 0x5F, 0xA6, 0x6, 0x60, 0x66, 0x6, 0x60, 0x65, 0xFA, 0x3F, 0xC0, 0x0, 
0x3F, 0xC5, 0xFA, 0x60, 0x66, 0x6, 0x60, 0x66, 0x6, 0x5F, 0xA3, 0xFC, 0x1F, 0xA0, 0x6, 0x0, 0x60, 0x6, 0x0, 0x61, 0xFA, 0x3F, 0xC0, 0x0  
};
************************/

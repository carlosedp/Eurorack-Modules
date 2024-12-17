#include <Arduino.h>

#define splash_width 68
#define splash_height 64

// 'VFM_Splash', 68x64px
const unsigned char VFM_Splash[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0x88, 0x80, 0x00, 0x22, 0x1f, 0xff, 0x80, 0x3f, 0xff, 0xcc,
    0xc6, 0x02, 0x66, 0x3f, 0xff, 0x80, 0x1c, 0xfc, 0xc6, 0x66, 0x02, 0x66, 0x67, 0xf7, 0x00, 0x0c,
    0xfc, 0xe6, 0x60, 0x40, 0xcc, 0x67, 0xe6, 0x00, 0x0f, 0xff, 0xe3, 0x30, 0x40, 0xd8, 0xff, 0xfe,
    0x00, 0x07, 0xc3, 0xf3, 0x30, 0x41, 0x98, 0xfc, 0x7c, 0x00, 0x07, 0xe3, 0xf1, 0x98, 0x41, 0x31,
    0xf8, 0xfc, 0x00, 0x03, 0xe1, 0xf8, 0x88, 0xc3, 0x31, 0xf0, 0xf8, 0x00, 0x03, 0xf1, 0xf8, 0xcc,
    0xc2, 0x63, 0xf1, 0xf8, 0x00, 0x01, 0xf8, 0xfc, 0x60, 0xc0, 0x67, 0xe1, 0xf0, 0x00, 0x01, 0xf8,
    0xfc, 0x61, 0xc0, 0xc7, 0xe3, 0xf0, 0x00, 0x00, 0xdc, 0x66, 0x31, 0xc0, 0xcc, 0xc7, 0x60, 0x00,
    0x00, 0x7c, 0x66, 0x33, 0xc0, 0x8f, 0xc7, 0xe0, 0x00, 0x00, 0x7e, 0x3f, 0x03, 0xc0, 0x1f, 0x8f,
    0xc0, 0x00, 0x00, 0x3e, 0x1f, 0x83, 0xc0, 0x1f, 0x8f, 0xc0, 0x00, 0x00, 0x3f, 0x1f, 0x87, 0xc0,
    0x13, 0x1f, 0x80, 0x00, 0x00, 0x1f, 0x0f, 0x87, 0xc0, 0x07, 0x1f, 0x80, 0x00, 0x00, 0x1f, 0x8f,
    0x87, 0xc0, 0x26, 0x3f, 0x00, 0x00, 0x00, 0x0c, 0xc7, 0x8f, 0xc1, 0xe6, 0x37, 0x00, 0x00, 0x00,
    0x0c, 0xc7, 0x8f, 0xcf, 0xcc, 0x66, 0x00, 0x00, 0x00, 0x07, 0xe3, 0x1f, 0xff, 0xcc, 0x7c, 0x00,
    0x00, 0x00, 0x07, 0xe3, 0x1f, 0xff, 0x88, 0xfc, 0x00, 0x00, 0x00, 0x03, 0xf1, 0x1f, 0xff, 0x98,
    0xf8, 0x00, 0x00, 0x00, 0x03, 0xf0, 0x3f, 0xff, 0x11, 0xf8, 0x00, 0x00, 0x00, 0x01, 0xf8, 0x3f,
    0xff, 0x11, 0xf0, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x3e, 0x3f, 0x03, 0xf0, 0x00, 0x00, 0x00, 0x00,
    0xfc, 0x78, 0x3e, 0x03, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x00, 0x3e, 0x07, 0xe0, 0x00, 0x00,
    0x00, 0x00, 0x76, 0x00, 0x3c, 0x07, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x23, 0x00, 0x3c, 0x0c, 0xc0,
    0x00, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x38, 0x1f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x80, 0x38,
    0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x80, 0x38, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f,
    0xc0, 0x30, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xc0, 0x30, 0x7e, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x07, 0xe0, 0x30, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x60, 0x20, 0xcc, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x03, 0x70, 0x20, 0xc8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf0, 0x01, 0xf8,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf8, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8,
    0x03, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x03, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x7c, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x07, 0xc0, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x33, 0x0d, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0x1c, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x9f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xbf,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x07, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x03, 0x9c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x98, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Array of all bitmaps for convenience. (Total bytes used to store images in PROGMEM = 592)
const int vfm_bitmap_allArray_LEN = 1;
const unsigned char *vfm_bitmap_allArray[1] = {
    VFM_Splash};

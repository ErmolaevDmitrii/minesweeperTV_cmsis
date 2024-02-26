#ifndef GDI_H
#define GDI_H

#include <stdint.h>

typedef struct {
    uint8_t *image;
    uint16_t width;
    uint16_t height;
} gdi_handle;

void set_pixel(uint16_t x, uint16_t y, uint8_t color,
                      gdi_handle *handle);

void draw_picture(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                  const uint8_t *picture, gdi_handle *handle);

void draw_picture_blend(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                        const uint8_t *picture, gdi_handle *handle);

#endif

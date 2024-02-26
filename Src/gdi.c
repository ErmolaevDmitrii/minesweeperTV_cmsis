#include "gdi.h"

void set_pixel(uint16_t x, uint16_t y, uint8_t color, gdi_handle *handle) {
    if(handle->width <= x || handle->height <= y) {
        return;
    }

    uint8_t *cell = handle->image + y * (handle->width / 8) + x / 8;
    *cell ^= (- color ^ *cell) & (1 << (x % 8));

    return;
}

void draw_picture(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *picture, gdi_handle *handle) {
    if(x + width >= handle->width || y + height >= handle->height) {
        return;
    }

    uint16_t bytes_in_line = width / 8 + (width % 8 == 0 ? 0 : 1);

    for(uint16_t i = 0; i < height; ++i) {
        for(uint16_t j = 0; j < width; ++j) {
            set_pixel(x + j, y + i, (picture[i * bytes_in_line + j / 8] >> (7 - j % 8)) & 1, handle);
        }
    }

    return;
}

void draw_picture_blend(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *picture, gdi_handle *handle) {
    if(x + width >= handle->width || y + height >= handle->height) {
        return;
    }

    uint16_t bytes_in_line = width / 8 + (width % 8 == 0 ? 0 : 1);

    for(uint16_t i = 0; i < height; ++i) {
        for(uint16_t j = 0; j < width; ++j) {
            //uint8_t cell = (handle->image[(y + i) * (handle->width / 8) + (x + j) / 8]
             //              >> (7 - j % 8)) & 1;
            uint8_t cell = (picture[i * bytes_in_line + j / 8] >> (7 - j % 8)) & 1;
            if(cell) {
                set_pixel(x + j, y + i, /*(picture[i * bytes_in_line + j / 8] >> (7 - j % 8)) & 1*/ cell, handle);
            }
        }
    }

    return;
}

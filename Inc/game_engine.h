#ifndef GAME_ENGINE_H
#define GAME_ENGINE_H

#include <stdlib.h>
#include "gdi.h"
#include "game_symbols.h"

typedef struct {
    uint8_t field_width;
    uint8_t field_height;
    uint8_t *field;
    gdi_handle *graphics;
    uint32_t (*seed_function) (void);
    uint8_t player_pos_x;
    uint8_t player_pos_y;
    uint16_t mines_count;
    uint16_t flags_count;
    uint16_t openings;
    uint8_t game_state;
} game_handle;

void game_set_cell_value(uint8_t x, uint8_t y, uint8_t value, game_handle *handle);
uint8_t game_get_cell_value(uint8_t x, uint8_t y, game_handle *handle);
uint8_t game_is_cell_opened(uint8_t x, uint8_t y, game_handle *handle);
uint8_t game_is_cell_around_opened_neighbours(uint8_t x, uint8_t y, game_handle *handle);
void game_lose(game_handle *handle);
void game_win(game_handle *handle);
void game_show_bombs(game_handle *handle);
void game_set_cell_opened(uint8_t x, uint8_t y, game_handle *handle);
uint8_t game_is_cell_under_flag(uint8_t x, uint8_t y, game_handle *handle);
uint8_t game_calculate_flags_around_cell(uint8_t x, uint8_t y, game_handle *handle);
void game_open_cells_recursive(uint8_t x, uint8_t y, game_handle *handle);
void draw_cell(uint8_t x, uint8_t y, game_handle *handle);
void game_player_move(int8_t x, int8_t y, game_handle *handle);

void game_start(game_handle *handle);
void game_player_move_up(game_handle *handle);
void game_player_move_down(game_handle *handle);
void game_player_move_left(game_handle *handle);
void game_player_move_right(game_handle *handle);
void game_player_put_flag(game_handle *handle);
void game_player_open_cell(game_handle *handle);

#endif

#include "game_engine.h"

void game_start(game_handle *handle) {
    handle->player_pos_x = handle->field_width / 2;
    handle->player_pos_y = handle->field_height / 2;
    handle->mines_count = handle->field_width * handle->field_height / 7 * 1;
    handle->flags_count = handle->mines_count;
    handle->openings = 0;
    handle->game_state = 0;

    for(uint16_t i = 0; i < 64 / 8; ++i) {
        draw_picture(i *16, (handle->field_height + 1) * 16, 16, 16, zero_cell, handle->graphics);
    }

    for(uint16_t i = 0; i < handle->field_width * handle->field_height; ++i) {
        handle->field[i] = 0b00000001;
    }

    for(uint8_t i = 0; i < handle->field_height; ++i) {
        for(uint8_t j = 0; j < handle->field_width; ++j) {
            draw_cell(j, i, handle);
        }
    }
}

void game_player_move(int8_t x, int8_t y, game_handle *handle) {
    uint8_t old_x = handle->player_pos_x;
    uint8_t old_y = handle->player_pos_y;
    if(x) {
        if(x < 0 && handle->player_pos_x == 0) {
            handle->player_pos_x = handle->field_width - 1;
        }
        else {
            handle->player_pos_x += x;
            if(handle->player_pos_x >= handle->field_width) {
                handle->player_pos_x = 0;
            }
        }
    }

    if(y) {
        if(y < 0 && handle->player_pos_y == 0) {
            handle->player_pos_y = handle->field_height - 1;
        }
        else {
            handle->player_pos_y += y;
            if(handle->player_pos_y >= handle->field_height) {
                handle->player_pos_y = 0;
            }
        }
    }

    draw_cell(old_x, old_y, handle);
    draw_cell(handle->player_pos_x, handle->player_pos_y, handle);
    return;
}

void game_player_move_up(game_handle *handle) {
    game_player_move(0, -1, handle);
    return;
}

void game_player_move_down(game_handle *handle) {
    game_player_move(0, 1, handle);
    return;
}

void game_player_move_left(game_handle *handle) {
    game_player_move(-1, 0, handle);
    return;
}

void game_player_move_right(game_handle *handle) {
    game_player_move(1, 0, handle);
    return;
}

void game_player_put_flag(game_handle *handle) {
    if(!game_is_cell_opened(handle->player_pos_x, handle->player_pos_y, handle)) {
        if(game_is_cell_under_flag(handle->player_pos_x, handle->player_pos_y, handle)) {
            handle->field[handle->player_pos_x + handle->player_pos_y * handle->field_width] -= 2;
            ++handle->flags_count;
        }
        else {
            if(!handle->flags_count) {
                return;
            }
            --handle->flags_count;
            handle->field[handle->player_pos_x + handle->player_pos_y * handle->field_width] += 2;
        }
        draw_cell(handle->player_pos_x, handle->player_pos_y, handle);
    }
    return;
}

void game_player_open_cell(game_handle *handle) {
    if(handle->game_state != 0) {
        game_start(handle);
        return;
    }

    if(handle->openings == 0) {
        srand(handle->seed_function());
        uint16_t mines_count = 0;

        while(mines_count <= handle->mines_count) {
            uint8_t mine_x = (rand() % (handle->field_width));
            uint8_t mine_y = (rand() % (handle->field_height));

            if(game_get_cell_value(mine_x, mine_y, handle) == 10 ||
              (mine_x == handle->player_pos_x && mine_y == handle->player_pos_y)) {
                continue;
            }

            ++mines_count;
            game_set_cell_value(mine_x, mine_y, 10, handle);

            for(int16_t i = mine_y - 1; i < (mine_y + 2); ++i) {
                for(int16_t j = mine_x - 1;  j < (mine_x + 2); ++j) {
                    if((i < 0) || (j < 0) || (j >= handle->field_width) ||
                       (i >= handle->field_height) || (i == mine_y && j == mine_x)) {
                        continue;
                    }
                    if(game_get_cell_value(j, i, handle) == 10) {
                        continue;
                    }
                    game_set_cell_value(j, i, game_get_cell_value(j, i, handle) + 1, handle);
                }
            }
        }

    }

    uint8_t x = handle->player_pos_x, y = handle->player_pos_y;

    if(x >= handle->field_width || y >= handle->field_height) {
        return;
    }

    if(game_is_cell_under_flag(x, y, handle)) {
        return;
    }

    if(game_is_cell_around_opened_neighbours(x, y, handle)) {
        return;
    }

    if(game_get_cell_value(x, y, handle) == 10) {
        handle->game_state = 2;
        //game_show_bombs(handle);
        game_lose(handle);
        return;
    }

    if(!game_is_cell_opened(x, y, handle)) {
        game_set_cell_opened(x, y, handle);
        ++handle->openings;
    }


    if(handle->openings >= (handle->field_width * handle->field_height - handle->mines_count - 1)) {
        handle->game_state = 1;
        game_win(handle);
        return;
    }

    if(game_get_cell_value(x, y, handle) == 0 ||
       game_get_cell_value(x, y, handle) ==
       game_calculate_flags_around_cell(x, y, handle))
    {
        for(int16_t i = y - 1; i < (y + 2); ++i) {
            for(int16_t j = x - 1;  j < (x + 2); ++j) {
                if((i < 0) || (j < 0) || (j >= handle->field_width) ||
                    (i >= handle->field_height) || (i == y && j == x)) {
                    continue;
                }

                game_open_cells_recursive(j, i, handle);
            }
        }
    }



    return;
}

void game_lose(game_handle *handle) {
    game_show_bombs(handle);
    draw_picture(0, (handle->field_height + 1) * 16, 64, 8, lose_print, handle->graphics);
}

void game_win(game_handle *handle) {
    draw_picture(0, (handle->field_height + 1) * 16, 56, 8, win_print, handle->graphics);
}

void game_show_bombs(game_handle *handle) {
    for(uint8_t i = 0; i < handle->field_height; ++i) {
        for(uint8_t j = 0; j < handle->field_width; ++j) {
            if(game_get_cell_value(j, i, handle) == 10) {
                game_set_cell_opened(j, i, handle);
            }
        }
    }

    return;
}

uint8_t game_is_cell_around_opened_neighbours(uint8_t x, uint8_t y, game_handle *handle) {
    uint8_t opened_neighbours = 0;

    for(uint16_t i = y - 1; i < (y + 2); ++i) {
        for(uint16_t j = x - 1;  j < (x + 2); ++j) {
            if((i < 0) || (j < 0) || (j >= handle->field_width) ||
               (i >= handle->field_height) || (i == y && j == x)) {
                continue;
            }
            opened_neighbours += game_is_cell_opened(j, i, handle);
        }
    }

    if((y == 0) || (y == (handle->field_height - 1))) {
        if((x == 0) || (x == (handle->field_width - 1))) {
            return (opened_neighbours == 3);
        }

        return (opened_neighbours == 5);
    }

    if((x == 0) || (x == (handle->field_width - 1))) {
        return (opened_neighbours == 5);
    }

    return (opened_neighbours == 8);
}

void game_open_cells_recursive(uint8_t x, uint8_t y, game_handle *handle) {
    if(x >= handle->field_width || y >= handle->field_height) {
        return;
    }

    if(game_is_cell_under_flag(x, y, handle)) {
        return;
    }

    if(game_is_cell_opened(x, y, handle)) {
        return;
    }

    if(game_get_cell_value(x, y, handle) == 10) {
        handle->game_state = 2;
        game_lose(handle);
        //game_show_bombs(handle);
        return;
    }

    ++handle->openings;
    game_set_cell_opened(x, y, handle);
    /*if(handle->openings == (handle->field_width * handle->field_height - handle->mines_count)) {
        handle->game_state = 1;
        game_win(handle);
        return;
    }*/

    if(game_get_cell_value(x, y, handle) == 0 ||
       game_get_cell_value(x, y, handle) ==
       game_calculate_flags_around_cell(x, y, handle))
    {
        for(int16_t i = y - 1; i < (y + 2); ++i) {
            for(int16_t j = x - 1;  j < (x + 2); ++j) {
                if((i < 0) || (j < 0) || (j >= handle->field_width) ||
                    (i >= handle->field_height) || (i == y && j == x)) {
                    continue;
                }

                game_open_cells_recursive(j, i, handle);
            }
        }
    }

    return;
}

uint8_t game_calculate_flags_around_cell(uint8_t x, uint8_t y, game_handle *handle) {
    uint8_t flags_count = 0;

    for(int16_t i = y - 1; i < (y + 2); ++i) {
        for(int16_t j = x - 1;  j < (x + 2); ++j) {
            if((i < 0) || (j < 0) || (j >= handle->field_width) ||
               (i >= handle->field_height) || (i == y && j == x)) {
                continue;
            }
            flags_count += game_is_cell_under_flag(j, i, handle);
        }
    }

    return flags_count;
}

void game_set_cell_value(uint8_t x, uint8_t y, uint8_t value, game_handle *handle) {
    handle->field[y * handle->field_width + x] <<= 4;
    handle->field[y * handle->field_width + x] >>= 4;
    handle->field[y * handle->field_width + x] += (uint8_t)(value << (uint8_t)4);
}

uint8_t game_get_cell_value(uint8_t x, uint8_t y, game_handle *handle) {
    return (uint8_t)(handle->field[y * handle->field_width + x]) >> (uint8_t)4;
}

uint8_t game_is_cell_opened(uint8_t x, uint8_t y, game_handle *handle) {
    return ((uint8_t)(handle->field[y * handle->field_width + x] << 4) == (uint8_t)0);
}

void game_set_cell_opened(uint8_t x, uint8_t y, game_handle *handle) {
    if(!game_is_cell_opened(x, y, handle)) {
        if(game_is_cell_under_flag(x, y, handle)) {
            handle->field[y * handle->field_width + x] -= 2;
        }
        handle->field[y * handle->field_width + x] -= 1;
        draw_cell(x, y, handle);
    }
    return;
}

uint8_t game_is_cell_under_flag(uint8_t x, uint8_t y, game_handle *handle) {
    return ((uint8_t)(handle->field[y * handle->field_width + x] << 4)) == (uint8_t)0b00110000;
}

void draw_cell(uint8_t x, uint8_t y, game_handle *handle) {
    if(game_is_cell_opened(x, y, handle)) {
        switch(game_get_cell_value(x, y, handle)) {
            case 1:
                draw_picture(x * 16, y * 16, 16, 16, one, handle->graphics);
                break;
            case 2:
                draw_picture(x * 16, y * 16, 16, 16, two, handle->graphics);
                break;
            case 3:
                draw_picture(x * 16, y * 16, 16, 16, three, handle->graphics);
                break;
            case 4:
                draw_picture(x * 16, y * 16, 16, 16, four, handle->graphics);
                break;
            case 5:
                draw_picture(x * 16, y * 16, 16, 16, five, handle->graphics);
                break;
            case 6:
                draw_picture(x * 16, y * 16, 16, 16, six, handle->graphics);
                break;
            case 7:
                draw_picture(x * 16, y * 16, 16, 16, seven, handle->graphics);
                break;
            case 8:
                draw_picture(x * 16, y * 16, 16, 16, eight, handle->graphics);
                break;
            case 0:
                draw_picture(x * 16, y * 16, 16, 16, zero_cell, handle->graphics);
                break;
            case 10:
                draw_picture(x * 16, y * 16, 16, 16, bomb, handle->graphics);
                break;
        }
    }
    else {
        if(game_is_cell_under_flag(x, y, handle)) {
            draw_picture(x * 16, y * 16, 16, 16, flag, handle->graphics);
        }
        else {
            draw_picture(x * 16, y * 16, 16, 16, not_opened_cell, handle->graphics);
        }
    }

    if(handle->player_pos_x == x && handle->player_pos_y == y) {
        draw_picture_blend(x * 16, y * 16, 16, 16, selected_cell, handle->graphics);
    }
}

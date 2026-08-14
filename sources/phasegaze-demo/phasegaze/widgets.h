// widgets.h
// GUI widget structures and functions

#ifndef WIDGETS_H
#define WIDGETS_H

#include <stdbool.h>
#include <stdint.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include "config.h"

// ------------------------------------------------------------
// Widget Structures
// ------------------------------------------------------------

typedef struct {
    double value;
    double min, max;
    int x, y, width, height;
    bool dragging;
} slider_t;

typedef struct {
    char text[32];
    int x, y, width, height;
    bool active;
    int cursor_pos;
    int select_start;
    int select_end;
    uint32_t last_click_time;
    int last_click_x, last_click_y;
    uint32_t cursor_blink_time;
} textbox_t;

typedef struct {
    double value;
    double min, max;
    int x, y, width, height;
    bool dragging;
} vertical_slider_t;

typedef struct {
    int x, y, width, height;
    bool hover;
} button_t;

typedef struct {
    textbox_t textbox;
    double min, max;
    double *linked_value;  // pointer to value to sync with (e.g., slider value)
    bool is_out_of_range;  // true when value is outside min/max
} editable_number_t;

// ------------------------------------------------------------
// Inline Utility Functions
// ------------------------------------------------------------

static inline bool point_in_rect(int x, int y, int rx, int ry, int rw, int rh)
{
    return (x >= rx && x < rx + rw && y >= ry && y < ry + rh);
}

// ------------------------------------------------------------
// Widget Functions (implemented in widgets.c)
// ------------------------------------------------------------

// Text rendering
void render_text(SDL_Renderer *ren, TTF_Font *font, int x, int y, const char *text, SDL_Color color);
void render_tooltip(SDL_Renderer *ren, TTF_Font *font, int x, int y, const char *text);

// Slider functions
void draw_slider(SDL_Renderer *ren, slider_t *s);
void update_slider_from_value(slider_t *s, double value);
double get_slider_value_from_pos(slider_t *s, int mouse_x);

// Vertical slider functions
void draw_vertical_slider(SDL_Renderer *ren, vertical_slider_t *s, TTF_Font *font);
double get_vertical_slider_value_from_pos(vertical_slider_t *s, int mouse_y);
void update_vertical_slider_from_value(vertical_slider_t *s, double value);

// Textbox functions
void draw_textbox(SDL_Renderer *ren, textbox_t *tb, SDL_Color bg, SDL_Color fg, SDL_Color border, TTF_Font *font);
void update_textbox_from_slider(textbox_t *tb, slider_t *s);
void textbox_select_all(textbox_t *tb);
void textbox_clear_selection(textbox_t *tb);
void textbox_delete_selected(textbox_t *tb);
void textbox_insert_char(textbox_t *tb, char c);
int textbox_get_cursor_pos_from_x(textbox_t *tb, int click_x, TTF_Font *font);

// Button functions
void draw_button(SDL_Renderer *ren, button_t *b, const char *label, TTF_Font *font);

// Editable number functions
void init_editable_number(editable_number_t *en, double value, double min_val, double max_val,
                          int x, int y, int width, int height, double *linked_value);
void update_editable_number_from_slider(editable_number_t *en, double slider_value);
bool validate_editable_number_range(editable_number_t *en, double *out_value);
void draw_editable_number(SDL_Renderer *ren, editable_number_t *en, TTF_Font *font);

// Frequency validation
bool validate_frequency_range(double start, double stop);

#endif // WIDGETS_H


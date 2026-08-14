// widgets.c
// GUI widget implementations

#include "widgets.h"
#include "config.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// ------------------------------------------------------------
// Text Rendering
// ------------------------------------------------------------

void render_text(SDL_Renderer *ren, TTF_Font *font, int x, int y, const char *text, SDL_Color color)
{
    if (!font || !text || !*text) return;
    
    SDL_Surface *surface = TTF_RenderText_Solid(font, text, color);
    if (!surface) return;
    
    SDL_Texture *texture = SDL_CreateTextureFromSurface(ren, surface);
    if (texture)
    {
        SDL_Rect dst = {x, y, surface->w, surface->h};
        SDL_RenderCopy(ren, texture, NULL, &dst);
        SDL_DestroyTexture(texture);
    }
    
    SDL_FreeSurface(surface);
}

void render_tooltip(SDL_Renderer *ren, TTF_Font *font, int x, int y, const char *text)
{
    if (!font || !text || !*text) return;
    
    // Split text by newlines and render each line
    int line_y = y;
    int max_width = 0;
    
    // First pass: measure to determine tooltip size
    const char *p = text;
    while (*p)
    {
        const char *line_end = p;
        while (*line_end && *line_end != '\n') line_end++;
        
        char line[256];
        int line_len = (int)(line_end - p);
        if (line_len >= (int)sizeof(line)) line_len = (int)sizeof(line) - 1;
        memcpy(line, p, (size_t)line_len);
        line[line_len] = '\0';
        
        SDL_Surface *surf = TTF_RenderText_Solid(font, line, (SDL_Color){255, 255, 255, 255});
        if (surf)
        {
            if (surf->w > max_width) max_width = surf->w;
            SDL_FreeSurface(surf);
        }
        
        if (!*line_end) break;
        p = line_end + 1;
    }
    
    // Draw tooltip background
    int line_count = 1;
    for (const char *p2 = text; *p2; p2++) if (*p2 == '\n') line_count++;
    int tooltip_height = line_count * 16 + 8;
    int tooltip_width = max_width + 16;
    
    SDL_SetRenderDrawColor(ren, 40, 40, 40, 240);
    SDL_Rect bg_rect = {x - 4, y - 4, tooltip_width, tooltip_height};
    SDL_RenderFillRect(ren, &bg_rect);
    
    SDL_SetRenderDrawColor(ren, 200, 200, 200, 255);
    SDL_RenderDrawRect(ren, &bg_rect);
    
    // Second pass: render lines
    p = text;
    line_y = y;
    while (*p)
    {
        const char *line_end = p;
        while (*line_end && *line_end != '\n') line_end++;
        
        char line[256];
        int line_len = (int)(line_end - p);
        if (line_len >= (int)sizeof(line)) line_len = (int)sizeof(line) - 1;
        memcpy(line, p, (size_t)line_len);
        line[line_len] = '\0';
        
        render_text(ren, font, x, line_y, line, (SDL_Color){255, 255, 255, 255});
        line_y += 16;
        
        if (!*line_end) break;
        p = line_end + 1;
    }
}

// ------------------------------------------------------------
// Slider Functions
// ------------------------------------------------------------

void draw_slider(SDL_Renderer *ren, slider_t *s)
{
    // Draw track
    SDL_SetRenderDrawColor(ren, 100, 100, 100, 255);
    SDL_Rect track = {s->x, s->y + s->height/2 - 2, s->width, 4};
    SDL_RenderFillRect(ren, &track);

    // Calculate handle position
    double ratio = (s->value - s->min) / (s->max - s->min);
    int handle_x = s->x + (int)(ratio * s->width) - 5;
    int handle_y = s->y;

    // Draw handle
    SDL_SetRenderDrawColor(ren, 200, 200, 200, 255);
    SDL_Rect handle = {handle_x, handle_y, 10, s->height};
    SDL_RenderFillRect(ren, &handle);

    // Draw handle border
    SDL_SetRenderDrawColor(ren, 50, 50, 50, 255);
    SDL_RenderDrawRect(ren, &handle);
}

void update_slider_from_value(slider_t *s, double value)
{
    if (value < s->min) value = s->min;
    if (value > s->max) value = s->max;
    s->value = value;
}

double get_slider_value_from_pos(slider_t *s, int mouse_x)
{
    int rel_x = mouse_x - s->x;
    double ratio = (double)rel_x / (double)s->width;
    if (ratio < 0.0) ratio = 0.0;
    if (ratio > 1.0) ratio = 1.0;
    return s->min + ratio * (s->max - s->min);
}

// ------------------------------------------------------------
// Vertical Slider Functions
// ------------------------------------------------------------

void draw_vertical_slider(SDL_Renderer *ren, vertical_slider_t *s, TTF_Font *font)
{
    // Draw track
    SDL_SetRenderDrawColor(ren, 100, 100, 100, 255);
    SDL_Rect track = {s->x + s->width/2 - 2, s->y, 4, s->height};
    SDL_RenderFillRect(ren, &track);

    // Calculate handle position (from bottom)
    // Clamp ratio for out-of-range values
    double ratio = (s->value - s->min) / (s->max - s->min);
    if (ratio < 0.0) ratio = 0.0;
    if (ratio > 1.0) ratio = 1.0;
    
    int handle_y = s->y + s->height - (int)(ratio * s->height) - 5;
    int handle_x = s->x;

    // Draw handle
    SDL_SetRenderDrawColor(ren, 200, 200, 200, 255);
    SDL_Rect handle = {handle_x, handle_y, s->width, 10};
    SDL_RenderFillRect(ren, &handle);

    // Draw handle border
    SDL_SetRenderDrawColor(ren, 50, 50, 50, 255);
    SDL_RenderDrawRect(ren, &handle);

    // Draw min/max labels if font is available
    if (font)
    {
        SDL_Color label_color = {180, 180, 180, 255}; // Light gray
        char min_label[32], max_label[32];
        snprintf(min_label, sizeof(min_label), "%.2f", s->min);
        snprintf(max_label, sizeof(max_label), "%.2f", s->max);
        
        // Draw max label at top (centered horizontally) - above slider
        SDL_Surface *max_surf = TTF_RenderText_Solid(font, max_label, label_color);
        if (max_surf)
        {
            SDL_Texture *max_tex = SDL_CreateTextureFromSurface(ren, max_surf);
            if (max_tex)
            {
                int label_x = s->x + (s->width - max_surf->w) / 2;
                int label_y = s->y - 15; // Above slider
                SDL_Rect dst = {label_x, label_y, max_surf->w, max_surf->h};
                SDL_RenderCopy(ren, max_tex, NULL, &dst);
                SDL_DestroyTexture(max_tex);
            }
            SDL_FreeSurface(max_surf);
        }
        
        // Draw min label at bottom (centered horizontally) - below slider
        SDL_Surface *min_surf = TTF_RenderText_Solid(font, min_label, label_color);
        if (min_surf)
        {
            SDL_Texture *min_tex = SDL_CreateTextureFromSurface(ren, min_surf);
            if (min_tex)
            {
                int label_x = s->x + (s->width - min_surf->w) / 2;
                int label_y = s->y + s->height + 2; // Below slider
                SDL_Rect dst = {label_x, label_y, min_surf->w, min_surf->h};
                SDL_RenderCopy(ren, min_tex, NULL, &dst);
                SDL_DestroyTexture(min_tex);
            }
            SDL_FreeSurface(min_surf);
        }
    }
}

double get_vertical_slider_value_from_pos(vertical_slider_t *s, int mouse_y)
{
    int rel_y = mouse_y - s->y;
    double ratio = 1.0 - ((double)rel_y / (double)s->height);
    if (ratio < 0.0) ratio = 0.0;
    if (ratio > 1.0) ratio = 1.0;
    return s->min + ratio * (s->max - s->min);
}

void update_vertical_slider_from_value(vertical_slider_t *s, double value)
{
    if (value < s->min) value = s->min;
    if (value > s->max) value = s->max;
    s->value = value;
}

// ------------------------------------------------------------
// Textbox Functions
// ------------------------------------------------------------

void draw_textbox(SDL_Renderer *ren, textbox_t *tb, SDL_Color bg, SDL_Color fg, SDL_Color border, TTF_Font *font)
{
    // Draw background
    SDL_SetRenderDrawColor(ren, bg.r, bg.g, bg.b, bg.a);
    SDL_Rect rect = {tb->x, tb->y, tb->width, tb->height};
    SDL_RenderFillRect(ren, &rect);

    // Draw border (thicker when active)
    SDL_SetRenderDrawColor(ren, border.r, border.g, border.b, border.a);
    if (tb->active)
    {
        // Thicker border when active
        SDL_Rect border_rect = {tb->x - 2, tb->y - 2, tb->width + 4, tb->height + 4};
        SDL_RenderDrawRect(ren, &border_rect);
        border_rect.x++; border_rect.y++; border_rect.w -= 2; border_rect.h -= 2;
        SDL_RenderDrawRect(ren, &border_rect);
    }
    else
    {
        SDL_RenderDrawRect(ren, &rect);
    }

    if (!font) return;

    int text_y = tb->y + (tb->height - 16) / 2;
    int text_x = tb->x + 4;
    int len = (int)strlen(tb->text);

    // Draw selection highlight if there's a selection
    if (tb->select_start != tb->select_end && tb->active)
    {
        int sel_start = (tb->select_start < tb->select_end) ? tb->select_start : tb->select_end;
        int sel_end = (tb->select_start > tb->select_end) ? tb->select_start : tb->select_end;
        
        char temp[32];
        int pre_len = sel_start;
        if (pre_len > 0 && pre_len <= len)
        {
            memcpy(temp, tb->text, (size_t)pre_len);
            temp[pre_len] = '\0';
            SDL_Surface *pre_surf = TTF_RenderText_Solid(font, temp, fg);
            if (pre_surf)
            {
                int sel_start_x = text_x + pre_surf->w;
                SDL_FreeSurface(pre_surf);
                
                int sel_len = sel_end - sel_start;
                if (sel_len > 0 && sel_len <= len - sel_start)
                {
                    memcpy(temp, tb->text + sel_start, (size_t)sel_len);
                    temp[sel_len] = '\0';
                    SDL_Surface *sel_surf = TTF_RenderText_Solid(font, temp, fg);
                    if (sel_surf)
                    {
                        SDL_SetRenderDrawColor(ren, 50, 100, 200, 255);
                        SDL_Rect sel_rect = {sel_start_x, text_y, sel_surf->w, 16};
                        SDL_RenderFillRect(ren, &sel_rect);
                        SDL_FreeSurface(sel_surf);
                    }
                }
            }
        }
        else if (sel_start == 0)
        {
            int sel_len = sel_end;
            if (sel_len > 0 && sel_len <= len)
            {
                memcpy(temp, tb->text, (size_t)sel_len);
                temp[sel_len] = '\0';
                SDL_Surface *sel_surf = TTF_RenderText_Solid(font, temp, fg);
                if (sel_surf)
                {
                    SDL_SetRenderDrawColor(ren, 50, 100, 200, 255);
                    SDL_Rect sel_rect = {text_x, text_y, sel_surf->w, 16};
                    SDL_RenderFillRect(ren, &sel_rect);
                    SDL_FreeSurface(sel_surf);
                }
            }
        }
    }

    // Render text efficiently - only do character-by-character if there's a selection
    if (tb->text[0])
    {
        bool has_selection = tb->active && (tb->select_start != tb->select_end);
        
        if (has_selection)
        {
            SDL_Color text_color = fg;
            SDL_Color sel_color = {255, 255, 255, 255};
            
            int current_x = text_x;
            for (int i = 0; i < len; ++i)
            {
                bool in_selection = (i >= tb->select_start && i < tb->select_end) ||
                                    (i >= tb->select_end && i < tb->select_start);
                
                char c[2] = {tb->text[i], '\0'};
                SDL_Color color = in_selection ? sel_color : text_color;
                
                SDL_Surface *char_surf = TTF_RenderText_Solid(font, c, color);
                if (char_surf)
                {
                    SDL_Texture *char_tex = SDL_CreateTextureFromSurface(ren, char_surf);
                    if (char_tex)
                    {
                        SDL_Rect dst = {current_x, text_y, char_surf->w, char_surf->h};
                        SDL_RenderCopy(ren, char_tex, NULL, &dst);
                        SDL_DestroyTexture(char_tex);
                        current_x += char_surf->w;
                    }
                    SDL_FreeSurface(char_surf);
                }
            }
        }
        else
        {
            // Fast path: render entire text at once when no selection
            // Clip text if it exceeds textbox bounds (when not editing)
            if (!tb->active)
            {
                int text_width = 0;
                TTF_SizeText(font, tb->text, &text_width, NULL);
                int max_width = tb->width - 8; // 4px padding on each side
                
                if (text_width > max_width)
                {
                    // Set clipping rectangle to prevent text overflow
                    SDL_Rect clip_rect = {tb->x + 4, tb->y, max_width, tb->height};
                    SDL_RenderSetClipRect(ren, &clip_rect);
                    render_text(ren, font, text_x, text_y, tb->text, fg);
                    SDL_RenderSetClipRect(ren, NULL); // Reset clipping
                }
                else
                {
                    render_text(ren, font, text_x, text_y, tb->text, fg);
                }
            }
            else
            {
                render_text(ren, font, text_x, text_y, tb->text, fg);
            }
        }
    }

    // Draw blinking cursor if active
    if (tb->active)
    {
        uint32_t current_time = SDL_GetTicks();
        uint32_t time_since_blink = current_time - tb->cursor_blink_time;
        bool cursor_visible = (time_since_blink < 1000) || ((time_since_blink / 500) % 2 == 0);
        
        int cursor_x = text_x;
        if (tb->cursor_pos >= 0 && tb->cursor_pos <= len)
        {
            if (tb->cursor_pos > 0)
            {
                char temp[32];
                int pre_len = tb->cursor_pos;
                if (pre_len > len) pre_len = len;
                memcpy(temp, tb->text, (size_t)pre_len);
                temp[pre_len] = '\0';
                SDL_Surface *pre_surf = TTF_RenderText_Solid(font, temp, fg);
                if (pre_surf)
                {
                    cursor_x = text_x + pre_surf->w;
                    SDL_FreeSurface(pre_surf);
                }
            }
            
            if (cursor_visible)
            {
                SDL_SetRenderDrawColor(ren, 255, 255, 0, 255);
                SDL_Rect cursor_rect = {cursor_x, text_y, 3, 16};
                SDL_RenderFillRect(ren, &cursor_rect);
            }
        }
    }
}

void update_textbox_from_slider(textbox_t *tb, slider_t *s)
{
    snprintf(tb->text, sizeof(tb->text), "%.1f", s->value);
    tb->cursor_pos = (int)strlen(tb->text);
    tb->select_start = 0;
    tb->select_end = 0;
}

void textbox_select_all(textbox_t *tb)
{
    tb->select_start = 0;
    tb->select_end = (int)strlen(tb->text);
    tb->cursor_pos = tb->select_end;
}

void textbox_clear_selection(textbox_t *tb)
{
    tb->select_start = tb->cursor_pos;
    tb->select_end = tb->cursor_pos;
}

void textbox_delete_selected(textbox_t *tb)
{
    if (tb->select_start == tb->select_end) return;
    
    int start = (tb->select_start < tb->select_end) ? tb->select_start : tb->select_end;
    int end = (tb->select_start > tb->select_end) ? tb->select_start : tb->select_end;
    int len = (int)strlen(tb->text);
    
    memmove(tb->text + start, tb->text + end, (size_t)(len - end + 1));
    tb->cursor_pos = start;
    tb->select_start = start;
    tb->select_end = start;
    tb->cursor_blink_time = SDL_GetTicks();
}

void textbox_insert_char(textbox_t *tb, char c)
{
    int len = (int)strlen(tb->text);
    if (tb->select_start != tb->select_end)
    {
        textbox_delete_selected(tb);
        len = (int)strlen(tb->text);
    }
    
    if (len < (int)sizeof(tb->text) - 1)
    {
        memmove(tb->text + tb->cursor_pos + 1, tb->text + tb->cursor_pos, 
                (size_t)(len - tb->cursor_pos + 1));
        tb->text[tb->cursor_pos] = c;
        tb->cursor_pos++;
        tb->select_start = tb->cursor_pos;
        tb->select_end = tb->cursor_pos;
        tb->cursor_blink_time = SDL_GetTicks();
    }
}

int textbox_get_cursor_pos_from_x(textbox_t *tb, int click_x, TTF_Font *font)
{
    if (!font) return tb->cursor_pos;
    
    int text_x = tb->x + 4; // Same padding as in draw_textbox
    int rel_x = click_x - text_x;
    if (rel_x < 0) return 0;
    
    int len = (int)strlen(tb->text);
    if (len == 0) return 0;
    
    int current_x = 0;
    for (int i = 0; i < len; ++i)
    {
        char c[2] = {tb->text[i], '\0'};
        SDL_Surface *char_surf = TTF_RenderText_Solid(font, c, (SDL_Color){255, 255, 255, 255});
        if (char_surf)
        {
            int char_width = char_surf->w;
            SDL_FreeSurface(char_surf);
            
            if (rel_x < current_x + char_width / 2)
            {
                return i;
            }
            current_x += char_width;
        }
    }
    
    return len;
}

// ------------------------------------------------------------
// Button Functions
// ------------------------------------------------------------

void draw_button(SDL_Renderer *ren, button_t *b, const char *label, TTF_Font *font)
{
    // Draw button background
    SDL_SetRenderDrawColor(ren, b->hover ? 220 : 180, 50, 50, 255);
    SDL_Rect rect = {b->x, b->y, b->width, b->height};
    SDL_RenderFillRect(ren, &rect);

    // Draw border
    SDL_SetRenderDrawColor(ren, 100, 0, 0, 255);
    SDL_RenderDrawRect(ren, &rect);

    // Render text centered
    if (font && label && *label)
    {
        SDL_Color text_color = {255, 255, 255, 255};
        SDL_Surface *surface = TTF_RenderText_Solid(font, label, text_color);
        if (surface)
        {
            SDL_Texture *texture = SDL_CreateTextureFromSurface(ren, surface);
            if (texture)
            {
                int text_x = b->x + (b->width - surface->w) / 2;
                int text_y = b->y + (b->height - surface->h) / 2;
                SDL_Rect dst = {text_x, text_y, surface->w, surface->h};
                SDL_RenderCopy(ren, texture, NULL, &dst);
                SDL_DestroyTexture(texture);
            }
            SDL_FreeSurface(surface);
        }
    }
}

// ------------------------------------------------------------
// Editable Number Functions
// ------------------------------------------------------------

void init_editable_number(editable_number_t *en, double value, double min_val, double max_val,
                          int x, int y, int width, int height, double *linked_value)
{
    memset(&en->textbox, 0, sizeof(textbox_t));
    en->textbox.x = x;
    en->textbox.y = y;
    en->textbox.width = width;
    en->textbox.height = height;
    en->textbox.active = false;
    en->textbox.cursor_pos = 0;
    en->textbox.select_start = 0;
    en->textbox.select_end = 0;
    en->textbox.last_click_time = 0;
    en->textbox.last_click_x = 0;
    en->textbox.last_click_y = 0;
    en->textbox.cursor_blink_time = 0;
    en->min = min_val;
    en->max = max_val;
    en->linked_value = linked_value;
    en->is_out_of_range = false;
    snprintf(en->textbox.text, sizeof(en->textbox.text), "%.3f", value);
    en->textbox.cursor_pos = (int)strlen(en->textbox.text);
}

void update_editable_number_from_slider(editable_number_t *en, double slider_value)
{
    if (en->linked_value) *en->linked_value = slider_value;
    snprintf(en->textbox.text, sizeof(en->textbox.text), "%.3f", slider_value);
    en->textbox.cursor_pos = (int)strlen(en->textbox.text);
    en->textbox.select_start = 0;
    en->textbox.select_end = 0;
    en->is_out_of_range = (slider_value < en->min || slider_value > en->max);
}

bool validate_editable_number_range(editable_number_t *en, double *out_value)
{
    double val = atof(en->textbox.text);
    en->is_out_of_range = (val < en->min || val > en->max);
    if (out_value) *out_value = val;
    return true; // Always allow editing, even if out of range
}

void draw_editable_number(SDL_Renderer *ren, editable_number_t *en, TTF_Font *font)
{
    SDL_Color bg_color = {60, 60, 60, 255};
    SDL_Color fg_color = en->is_out_of_range ? (SDL_Color){255, 150, 150, 255} : (SDL_Color){255, 255, 255, 255};
    SDL_Color border_color = en->textbox.active ? (SDL_Color){100, 150, 255, 255} : (SDL_Color){150, 150, 150, 255};
    draw_textbox(ren, &en->textbox, bg_color, fg_color, border_color, font);
}

// ------------------------------------------------------------
// Frequency Validation
// ------------------------------------------------------------

bool validate_frequency_range(double start, double stop)
{
    return (start >= FREQ_MIN_MHZ && start <= FREQ_MAX_MHZ &&
            stop >= FREQ_MIN_MHZ && stop <= FREQ_MAX_MHZ &&
            start < stop);
}


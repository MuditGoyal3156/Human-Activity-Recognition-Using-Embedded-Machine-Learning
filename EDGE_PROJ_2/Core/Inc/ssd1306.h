#ifndef SSD1306_H
#define SSD1306_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define SSD1306_WIDTH   128U
#define SSD1306_HEIGHT  64U
#define SSD1306_FB_SIZE ((SSD1306_WIDTH * SSD1306_HEIGHT) / 8U)

#define FONT_SMALL  1
#define FONT_LARGE  2

void ssd1306_init(void);
void ssd1306_clear(void);
void ssd1306_update(void);
void ssd1306_draw_text(int x, int y, int font_size, const char *text);

#ifdef __cplusplus
}
#endif

#endif /* SSD1306_H */

#ifndef INC_GC9A01_H_
#define INC_GC9A01_H_

#include "main.h"
#include <stdint.h>

// ==== USER CONFIGURATIONS ====
#define GC9A01_SPI           hspi1
#define GC9A01_SPI_TIMEOUT   100
#define USE_DMA              0  // Enable/disable DMA

#define GC9A01_CS_PORT       GPIOA
#define GC9A01_CS_PIN        GPIO_PIN_4

#define GC9A01_DC_PORT       GPIOB
#define GC9A01_DC_PIN        GPIO_PIN_0

#define GC9A01_RST_PORT      GPIOB
#define GC9A01_RST_PIN       GPIO_PIN_1

// Optional user callback for flush complete
void GC9A01_FlushReady(void);

// ==== API ====
void GC9A01_Init(void);
void GC9A01_Reset(void);
void GC9A01_WriteCommand(uint8_t cmd);
void GC9A01_WriteData(uint8_t data);
void GC9A01_WriteDataBuffer(uint8_t *data, uint32_t size);
void GC9A01_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void GC9A01_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void GC9A01_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void GC9A01_Flush(const void *color_map, int32_t x1, int32_t y1, int32_t x2, int32_t y2);

#endif

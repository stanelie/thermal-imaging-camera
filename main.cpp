/**
 * Copyright 2023 André Weinand
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <cstdio>
#include <ctime>
#include <math.h>
#include <pico/stdlib.h>
#include "pico/multicore.h"
#include "hardware/i2c.h"

extern "C"{
#include <MLX90640_I2C_Driver.h>
#include <MLX90640_API.h>
void MLX90640_CalculateTo_fast(uint16_t *frameData, const paramsMLX90640 *params,
                               float emissivity, float tr, float *result);
}

#include "SSD1351_SPI_DRIVER.h"
#include "SSD1351_API.h"

// ---- configuration ----

// MLX90640 32 x 24 Thermopile Array
constexpr uint8_t REFRESH_RATE = 6;             // 0: 0.5 Hz, 1: 1 Hz, 2: 2 Hz, 3: 4 Hz, 4: 8 Hz, 5: 16 Hz, 6: 32 Hz, 7: 64 Hz
constexpr float EMISSIVITY = 0.95;              // the emissivity of the measured object (1.0 = black body)
constexpr float OPENAIR_TA_SHIFT = -8.0;        // for a MLX90640 in the open air the shift is -8 deg Celsius

// SSD1351 128 x 128 OLED Display
constexpr uint16_t HEAT_MAP_SIZE = 256;         // the number of colors in the heat map (must be <= 256)
constexpr bool BILINEAR_INTERPOLATION = true;   // if true use bilinear interpolation and nearest neighbor interpolation otherwise

#define FLIP_GRAPH_HORIZONTAL 1
#define FLIP_GRAPH_VERTICAL 0

// ---- profiling ----

#define PROFILE 1

#if PROFILE
// per-stage microsecond accumulators, summed over a 1 second window
static volatile uint32_t p_wait, p_read, p_calc, p_scale, p_fifo0;  // core0 stages
static volatile uint32_t p_render, p_text, p_spi, p_fifo1;          // core1 stages
static volatile bool p_report = false;
static uint32_t p_loops = 0;
static uint32_t p_window_start = 0;
static volatile uint32_t p_latency = 0;   // sensor-ready to pixels-on-screen
static volatile uint32_t p_l1 = 0, p_l2 = 0, p_l3 = 0;  // core0 / queue / core1 shares
static int p_trace = 3;     // trace the first few iterations stage by stage
#define TRACE(msg) do { if (p_trace > 0) printf("  trace: " msg "\n"); } while (0)
#define PROF_T(v)        uint32_t v = time_us_32()
#define PROF_ACC(acc, v) acc += time_us_32() - (v)
#else
#define PROF_T(v)        do {} while (0)
#define PROF_ACC(acc, v) do {} while (0)
#endif

// ---- other constants (don't change) ----

constexpr uint8_t MLX_I2C_ADDR = 0x33;          // I2C address of the MLX90640

constexpr uint16_t BANNER_HEIGHT = 16;                                  // bottom strip holding min / fps / max
constexpr uint16_t GRAPH_WIDTH = DISPLAY_WIDTH;                         // width of the thermal camera graph on the OLED display
constexpr uint16_t GRAPH_HEIGHT = DISPLAY_HEIGHT - BANNER_HEIGHT;       // height of the thermal camera graph on the OLED display

// interpolation lookup tables, built once: source index and fractional weight
// for every display column and row. Mapping the last display pixel exactly onto
// the last sensor pixel needs the (DST-1) denominator, so the outermost sensor
// row and column reach full weight instead of being truncated away.
static uint8_t  ipx0[GRAPH_WIDTH],  ipy0[GRAPH_HEIGHT];
static uint16_t ipxf[GRAPH_WIDTH],  ipyf[GRAPH_HEIGHT];

static void interp_tables_init() {
    for (int x = 0; x < GRAPH_WIDTH; x++) {
        int32_t pos = (x * ((MLX90640_COLUMN_NUM - 1) << 8)) / (GRAPH_WIDTH - 1);
        int32_t i = pos >> 8, f = pos & 0xff;
        if (i >= MLX90640_COLUMN_NUM - 1) { i = MLX90640_COLUMN_NUM - 2; f = 256; }
        ipx0[x] = (uint8_t)i;
        ipxf[x] = (uint16_t)f;
    }
    for (int y = 0; y < GRAPH_HEIGHT; y++) {
        int32_t pos = (y * ((MLX90640_LINE_NUM - 1) << 8)) / (GRAPH_HEIGHT - 1);
        int32_t i = pos >> 8, f = pos & 0xff;
        if (i >= MLX90640_LINE_NUM - 1) { i = MLX90640_LINE_NUM - 2; f = 256; }
        ipy0[y] = (uint8_t)i;
        ipyf[y] = (uint16_t)f;
    }
}

// ---- implementation ----

// measure display frame rate with a timer interrupt
static repeating_timer_t timer;
static int frame_cnt = 0;
static int fps = 0;

bool timer_callback(repeating_timer_t *rt)
{   
    fps = frame_cnt;
    frame_cnt = 0;
#if PROFILE
    p_report = true;
#endif
    return true;
}

// a "heat" color map inspired by: http://www.andrewnoske.com/wiki/Code_-_heatmaps_and_color_gradients

typedef struct {
    uint8_t r, g, b;
} RGBColor;

#define RGB_BLACK {0, 0, 0} 
#define RGB_RED {255, 38, 0} 
#define RGB_YELLOW {255, 255, 0} 
#define RGB_GREEN {0, 249, 0} 
#define RGB_BLUE {4, 51, 255}
#define RGB_ORANGE {255, 147, 0}
#define RGB_CYAN {0, 253, 255}
#define RGB_MAGENTA {255, 64, 255}
#define RGB_PURPLE {148, 33, 146}
#define RGB_WHITE {255, 255, 255} 

// palette from "version stan bilinear smoothing", May 2025: the repeated black
// entries hold the bottom third of the range at black, which keeps low-end
// sensor noise from showing as colour
constexpr RGBColor colors[] = {RGB_BLACK, RGB_BLACK, RGB_BLACK, RGB_BLACK,
                               RGB_BLUE, RGB_CYAN, RGB_GREEN, RGB_YELLOW, RGB_RED, RGB_WHITE};
//constexpr RGBColor colors[] = {RGB_BLUE, RGB_CYAN, RGB_GREEN, RGB_YELLOW, RGB_RED};
//constexpr RGBColor colors[] = {RGB_BLUE, RGB_RED};
//constexpr RGBColor colors[] = {RGB_VIOLET, RGB_ORANGE};
//constexpr RGBColor colors[] = {RGB_BLACK, RGB_WHITE};

static uint16_t palette[HEAT_MAP_SIZE];

void heatmap_init() {
    constexpr int numColors = sizeof(colors) / sizeof(RGBColor);
    for (int c = 0; c < HEAT_MAP_SIZE; c++) {
        float value = c * (numColors-1) / float(HEAT_MAP_SIZE-1);
        int idx1 = int(value);
        uint8_t red, green, blue;
        if (idx1 == value) {
            red = colors[idx1].r;
            green = colors[idx1].g;
            blue = colors[idx1].b;
        } else {
            int idx2 = idx1 + 1;
            float fractBetween = value - idx1;
            red = int(round((colors[idx2].r - colors[idx1].r) * fractBetween + colors[idx1].r));
            green = int(round((colors[idx2].g - colors[idx1].g) * fractBetween + colors[idx1].g));
            blue = int(round((colors[idx2].b - colors[idx1].b) * fractBetween + colors[idx1].b));
        }
        palette[c] = SSD1351_color(red, green, blue);
    }
}

#if FLIP_GRAPH_HORIZONTAL
#define TR_X(x) (GRAPH_WIDTH-1-(x))
#else
#define TR_X(x) (x)
#endif
#if FLIP_GRAPH_VERTICAL
#define TR_Y(y) (GRAPH_HEIGHT-1-(y))
#else
#define TR_Y(y) (y)
#endif

// fill a rectangle in graph coordinates, applying the flips to the whole rect.
// Passing a flipped origin straight to SSD1351_fillrect would shift the block by
// its own width and run past the right edge of the framebuffer.
static inline void graph_fillrect(int x, int y, int w, int h, uint16_t color) {
#if FLIP_GRAPH_HORIZONTAL
    x = GRAPH_WIDTH - x - w;
#endif
#if FLIP_GRAPH_VERTICAL
    y = GRAPH_HEIGHT - y - h;
#endif
    SSD1351_fillrect(x, y, w, h, color);
}

typedef struct {
    uint32_t t_ready;                   // time_us_32() when the sensor flagged this subpage ready
    uint32_t t_push;                    // time_us_32() when core0 handed it over
    float min, max;                     // min and max temperature values
    uint8_t values[MLX90640_PIXEL_NUM]; // temperature values from [min .. max] scaled to [0 .. HEAT_MAP_SIZE-1]

    inline uint8_t value(int16_t x, int16_t y) {
        return values[(y << 5) + x];
    }
} FrameDTO;

void renderer() {

    SSD1351_SPIInit();

    heatmap_init();
    interp_tables_init();

    add_repeating_timer_ms(1000, &timer_callback, NULL, &timer);

    // Two frame buffers. Core0 fills one while core1 renders the other, and a
    // buffer is not released until its frame has actually been pushed to the
    // display: releasing it earlier lets core0 run ahead and read the sensor
    // sooner than needed, which only makes the displayed frame staler.
    multicore_fifo_push_blocking((uint32_t)new FrameDTO());
    multicore_fifo_push_blocking((uint32_t)new FrameDTO());

    // process DTOs received from core0
    while (true) {

        // wait for a frame DTO received from core0
        PROF_T(t_f1);
        FrameDTO *dto = (FrameDTO*) multicore_fifo_pop_blocking();
        PROF_ACC(p_fifo1, t_f1);

        PROF_T(t_render);
#if PROFILE
        const uint32_t t_pop = time_us_32();
        const uint32_t frame_t_ready = dto->t_ready;
        const uint32_t frame_t_push = dto->t_push;
#endif

        if constexpr (BILINEAR_INTERPOLATION) {
            // integer bilinear interpolation off the precomputed tables
            for (int y = 0; y < GRAPH_HEIGHT; y++) {

                const int32_t y0 = ipy0[y];
                const int32_t ty = ipyf[y];
                const uint8_t *row0 = &dto->values[y0 << 5];
                const uint8_t *row1 = row0 + MLX90640_COLUMN_NUM;

                for (int x = 0; x < GRAPH_WIDTH; x++) {

                    const int32_t x0 = ipx0[x];
                    const int32_t tx = ipxf[x];

                    int32_t v00 = row0[x0], v10 = row0[x0+1];
                    int32_t v01 = row1[x0], v11 = row1[x0+1];

                    int32_t s = v00 + ((tx * (v10-v00)) >> 8);
                    int32_t e = v01 + ((tx * (v11-v01)) >> 8);
                    int32_t v = s + ((ty * (e-s)) >> 8);

                    SSD1351_pixel(TR_X(x), TR_Y(y), palette[v]);
                }
            }
        } else {
            // nearest neighbor: each sensor pixel fills its own block of the graph
            for (int y = 0; y < MLX90640_LINE_NUM; y++) {
                int y_top = (y * GRAPH_HEIGHT) / MLX90640_LINE_NUM;
                int h = ((y + 1) * GRAPH_HEIGHT) / MLX90640_LINE_NUM - y_top;
                for (int x = 0; x < MLX90640_COLUMN_NUM; x++) {
                    int x_lft = (x * GRAPH_WIDTH) / MLX90640_COLUMN_NUM;
                    int w = ((x + 1) * GRAPH_WIDTH) / MLX90640_COLUMN_NUM - x_lft;
                    graph_fillrect(x_lft, y_top, w, h, palette[dto->value(x, y)]);
                }
            }
        }

        PROF_ACC(p_render, t_render);

        float min = dto->min;
        float max = dto->max;

        PROF_T(t_text);

        int y = GRAPH_HEIGHT;
        SSD1351_fillrect(0, y, DISPLAY_WIDTH, 16, BLACK);

        y += 4;
        char buf[32];
        sprintf(buf, "%.0f", min);
        SSD1351_text(1, y, buf, WHITE);

        sprintf(buf, "%d fps", fps);
        int w = SSD1351_textwidth(buf);
        SSD1351_text((DISPLAY_WIDTH-w) / 2, y, buf, WHITE);

        sprintf(buf, "%.0f", max);
        w = SSD1351_textwidth(buf);
        SSD1351_text(DISPLAY_WIDTH-w-1, y, buf, WHITE);

        PROF_ACC(p_text, t_text);

        PROF_T(t_spi);
        SSD1351_update();
        PROF_ACC(p_spi, t_spi);

        // the frame is on the display now, so this buffer can be refilled
        multicore_fifo_push_blocking((uint32_t)dto);
#if PROFILE
        {
            const uint32_t now = time_us_32();
            p_l1 += frame_t_push - frame_t_ready;
            p_l2 += t_pop - frame_t_push;
            p_l3 += now - t_pop;
            p_latency += now - frame_t_ready;
        }
#endif

        frame_cnt++;
    }
}

int main() {

    stdio_init_all();

#if PROFILE
    // give a host serial monitor time to attach before the boot markers go out
    for (int i = 0; i < 8; i++) {
        printf("boot: waiting for host %d\n", i);
        sleep_ms(250);
    }
    printf("boot: stdio up\n");
#endif

    multicore_launch_core1(renderer);
    printf("boot: core1 launched\n");

    sleep_ms(40 + 500); // after Power-On wait a bit for the MLX90640 to initialize

    MLX90640_I2CInit();
    MLX90640_SetResolution(MLX_I2C_ADDR, 3);    // 0: 16 bit, 1: 17 bit, 2 = 18 bit, 3 = 19 bit
    MLX90640_SetRefreshRate(MLX_I2C_ADDR, REFRESH_RATE);
    MLX90640_SetChessMode(MLX_I2C_ADDR);

    printf("boot: sensor configured\n");

    uint16_t *eeMLX90640 = new uint16_t[832];       // too large for allocating on stack
    int ee_err = MLX90640_DumpEE(MLX_I2C_ADDR, eeMLX90640);
    printf("boot: DumpEE -> %d\n", ee_err);
    paramsMLX90640 *params = new paramsMLX90640;    // too large for allocating on stack
    int pe_err = MLX90640_ExtractParameters(eeMLX90640, params);
    printf("boot: ExtractParameters -> %d\n", pe_err);
    delete eeMLX90640;

    uint16_t *captureFrame = new uint16_t[834];     // too large for allocating on stack 
    float *values = new float[MLX90640_PIXEL_NUM];  // too large for allocating on stack 
    int patternMode = MLX90640_GetCurMode(MLX_I2C_ADDR);
    FrameDTO *dto = NULL;   // held across iterations so a skipped frame never loses it
    printf("boot: patternMode=%d, entering main loop\n", patternMode);
    {
        extern volatile uint32_t g_spi_baud, g_clk_peri, g_dma_pushes, g_pio_pushes;
        printf("boot: clk_peri=%lu Hz, spi baud=%lu Hz, dma_pushes=%lu blocking_pushes=%lu\n",
               g_clk_peri, g_spi_baud, g_dma_pushes, g_pio_pushes);
    }


    while (true) {

#if PROFILE
        // report every N frames from core0 itself, so the numbers do not depend
        // on the repeating timer (which is registered from core1) ever firing
        if (++p_loops >= 32) {
            uint32_t now = time_us_32();
            uint32_t elapsed = now - p_window_start;
            p_window_start = now;
            const uint32_t n = 32;
            printf("%2lu fps | core0 wait=%5lu read=%5lu calc=%5lu scale=%4lu fifo=%5lu"
                   " | core1 wait=%5lu render=%5lu spi=%5lu | lat=%6lu = %5lu + %5lu + %5lu\n",
                   (uint32_t)(32000000UL / (elapsed ? elapsed : 1)),
                   p_wait/n, p_read/n, p_calc/n, p_scale/n, p_fifo0/n,
                   p_fifo1/n, p_render/n, p_spi/n, p_latency/n, p_l1/n, p_l2/n, p_l3/n);
            p_loops = 0;
            p_wait = p_read = p_calc = p_scale = p_fifo0 = 0;
            p_fifo1 = p_render = p_text = p_spi = p_latency = 0;
            p_l1 = p_l2 = p_l3 = 0;
        }
#endif

        // take a free buffer first: blocking here rather than after the capture
        // keeps the sensor read as late as possible, so the data handed over is
        // as fresh as it can be. A frame skipped below keeps its buffer, since
        // dropping one here would permanently shorten the pipeline.
        if (dto == NULL) {
            PROF_T(t_f0);
            dto = (FrameDTO*)multicore_fifo_pop_blocking();
            PROF_ACC(p_fifo0, t_f0);
        }

        // wait until the MLX90640 has a new subpage ready. Polling here rather than
        // letting MLX90640_GetFrameData do it separates sensor slack from I2C cost.
        PROF_T(t_wait);
        uint16_t statusReg = 0;
        while (!MLX90640_GET_DATA_READY(statusReg)) {
            if (MLX90640_I2CRead(MLX_I2C_ADDR, MLX90640_STATUS_REG, 1, &statusReg) != 0) {
                break;
            }
        }
        PROF_ACC(p_wait, t_wait);
        const uint32_t t_ready = time_us_32();
        TRACE("sensor data ready");

        // read pages (half frames) from the MLX90640
        PROF_T(t_read);
        int status = MLX90640_GetFrameData(MLX_I2C_ADDR, captureFrame);
        PROF_ACC(p_read, t_read);
        TRACE("frame read");
        if (status < 0) {
            printf("Error: MLX90640_GetFrameData returned %d\n", status);
            continue;   // skip this frame
        }
        PROF_T(t_calc);
        float eTa = MLX90640_GetTa(captureFrame, params) + OPENAIR_TA_SHIFT;
        MLX90640_CalculateTo_fast(captureFrame, params, EMISSIVITY, eTa, values);
        MLX90640_BadPixelsCorrection(params->brokenPixels, values, patternMode, params);
        MLX90640_BadPixelsCorrection(params->outlierPixels, values, patternMode, params);
        PROF_ACC(p_calc, t_calc);
        TRACE("CalculateTo done");

        // find min and max temperature values of the frame
        PROF_T(t_scale);
        float min, max;
        min = max = values[0];
        for (int i = 1; i < MLX90640_PIXEL_NUM; i++) {
            float value = values[i];
            if (value > max)
                max = value;
            if (value < min)
                min = value;
        }

        PROF_ACC(p_scale, t_scale);

        dto->t_ready = t_ready;
        dto->min = min;
        dto->max = max;

        PROF_T(t_scale2);
        float step = (ceil(max + 1.0) - floor(min - 1.0)) / float(HEAT_MAP_SIZE-1);
        for (int i = 0; i < MLX90640_PIXEL_NUM; i++) {
            dto->values[i] = uint8_t((values[i] - min) / step);
        }
        PROF_ACC(p_scale, t_scale2);
        dto->t_push = time_us_32();
        multicore_fifo_push_blocking((uint32_t)dto);
        dto = NULL;
        TRACE("pushed DTO to core1");
        if (p_trace > 0) p_trace--;
    }

    delete values;
    delete captureFrame;
    delete params;
    return 0;
}
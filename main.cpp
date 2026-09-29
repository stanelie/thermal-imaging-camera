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

constexpr uint8_t PIXEL_SIZE = DISPLAY_WIDTH / MLX90640_COLUMN_NUM;     // width and height of a single thermal camera pixel on the OLED display
constexpr uint16_t GRAPH_WIDTH = MLX90640_COLUMN_NUM * PIXEL_SIZE;      // width of the thermal camera graph on the OLED display
constexpr uint16_t GRAPH_HEIGHT = MLX90640_LINE_NUM * PIXEL_SIZE;       // height of the thermal camera graph on the OLED display

// ---- implementation ----

// handle touch button interrupts
static bool bilinear_interpolation = BILINEAR_INTERPOLATION;
static bool freeze_image = false;

void gpio_callback(uint gpio, uint32_t events) {
    switch (gpio) {
        case 14:    // touch button disables bilinear interpolation
            if (events & GPIO_IRQ_EDGE_RISE) {
                bilinear_interpolation = false;
            }
            if (events & GPIO_IRQ_EDGE_FALL) {
                bilinear_interpolation = true;
            }
            break;
        case 18:   // touch button "freeze image"
            if (events & GPIO_IRQ_EDGE_RISE) {
                freeze_image = true;
            }
            if (events & GPIO_IRQ_EDGE_FALL) {
                freeze_image = false;
            }
            break;
    }
}

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

constexpr RGBColor colors[] = {RGB_BLACK, RGB_BLUE, RGB_CYAN, RGB_GREEN, RGB_YELLOW, RGB_RED, RGB_WHITE};
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

typedef struct {
    float min, max;                     // min and max temperature values
    uint8_t values[MLX90640_PIXEL_NUM]; // temperature values from [min .. max] scaled to [0 .. HEAT_MAP_SIZE-1]

    inline uint8_t value(int16_t x, int16_t y) {
        return values[(y << 5) + x];
    }
} FrameDTO;

void renderer() {

    SSD1351_SPIInit();

    heatmap_init();

    for (int x = 0; x < DISPLAY_WIDTH; x++) {
        int l = (x * HEAT_MAP_SIZE) / DISPLAY_WIDTH;
        SSD1351_fillrect(x, 112, 1, 16, palette[l]);
    }

    add_repeating_timer_ms(1000, &timer_callback, NULL, &timer);

    gpio_set_irq_enabled(14, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(18, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_callback(&gpio_callback);
    irq_set_enabled(IO_IRQ_BANK0, true);

    // create a single DTO for the communication between main thread (core0) and renderer thread (core1)
    multicore_fifo_push_blocking((uint32_t)new FrameDTO());

    // process DTOs received from core0
    while (true) {

        // wait for a frame DTO received from core0
        PROF_T(t_f1);
        FrameDTO *dto = (FrameDTO*) multicore_fifo_pop_blocking();
        PROF_ACC(p_fifo1, t_f1);

        PROF_T(t_render);

        if (freeze_image) {
            // just send back the DTO without rendering it
            multicore_fifo_push_blocking((uint32_t)dto);
            continue;
        }

        if (bilinear_interpolation) {
            // integer bilinear interpolation
            int32_t mx = ((MLX90640_COLUMN_NUM - 1) << 7) / GRAPH_WIDTH;
            int32_t my = ((MLX90640_LINE_NUM - 1) << 7) / GRAPH_HEIGHT;

            for (int y = 0; y < GRAPH_HEIGHT; y++) {
                
                int32_t y0 = (y * my) >> 7;
                int32_t ty = (y * my) & 0x7f;
 
                for (int x = 0; x < GRAPH_WIDTH; x++) {

                    int32_t x0 = (x * mx) >> 7;
                    int32_t tx = (x * mx) & 0x7f;

                    int16_t v00 = dto->value(x0,   y0);
                    int16_t v10 = dto->value(x0+1, y0);
                    int16_t v01 = dto->value(x0,   y0+1);
                    int16_t v11 = dto->value(x0+1, y0+1);

                    int32_t s = v00 + ((tx * (v10-v00)) >> 7);
                    int32_t e = v01 + ((tx * (v11-v01)) >> 7);
                    int32_t v = s + ((ty * (e-s)) >> 7);
          
                    SSD1351_pixel(TR_X(x), TR_Y(y), palette[v]);
                }
            }
        } else {
            // nearest neighbor interpolation
            for (int y = 0; y < MLX90640_LINE_NUM; y++) {
                for (int x = 0; x < MLX90640_COLUMN_NUM; x++) {
                    SSD1351_fillrect(TR_X(x*PIXEL_SIZE), TR_Y(y*PIXEL_SIZE), PIXEL_SIZE, PIXEL_SIZE, palette[dto->value(x, y)]);
                }
            }
        }

        PROF_ACC(p_render, t_render);

        float min = dto->min;
        float max = dto->max;

        // send frame back to core0
        multicore_fifo_push_blocking((uint32_t)dto);

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

        frame_cnt++;
    }
}

int main() {

    stdio_init_all();

#if PROFILE
    // give a host serial monitor time to attach before the boot markers go out
    for (int i = 0; i < 20; i++) {
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
    printf("boot: patternMode=%d, entering main loop\n", patternMode);

#if PROFILE
    // scan the I2C bus, checking the SDK return codes the MLX driver throws away
    printf("diag: scanning I2C bus...\n");
    int found = 0;
    for (int addr = 0x08; addr < 0x78; addr++) {
        uint8_t dummy;
        int r = i2c_read_blocking(i2c0, addr, &dummy, 1, false);
        if (r >= 0) {
            printf("diag:   device responding at 0x%02x\n", addr);
            found++;
        }
    }
    printf("diag: scan done, %d device(s) found\n", found);

    // and show what the raw SDK calls say for the MLX specifically
    uint8_t cmd[2] = { MLX90640_STATUS_REG >> 8, MLX90640_STATUS_REG & 0xff };
    int wr = i2c_write_blocking(i2c0, MLX_I2C_ADDR, cmd, 2, true);
    uint8_t rx[2] = { 0, 0 };
    int rd = i2c_read_blocking(i2c0, MLX_I2C_ADDR, rx, 2, false);
    printf("diag: status reg: write->%d read->%d bytes=%02x%02x\n", wr, rd, rx[0], rx[1]);

    // read the idle bus levels: both high means an idle bus with nothing answering,
    // a low line means something is holding the bus down
    gpio_set_function(16, GPIO_FUNC_SIO);
    gpio_set_function(17, GPIO_FUNC_SIO);
    gpio_set_dir(16, GPIO_IN);
    gpio_set_dir(17, GPIO_IN);
    gpio_pull_up(16);
    gpio_pull_up(17);
    sleep_ms(2);
    printf("diag: with pullups   SDA(16)=%d SCL(17)=%d\n", gpio_get(16), gpio_get(17));
    gpio_disable_pulls(16);
    gpio_disable_pulls(17);
    sleep_ms(2);
    printf("diag: without pullups SDA(16)=%d SCL(17)=%d\n", gpio_get(16), gpio_get(17));
#endif

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
                   " | core1 fifo=%5lu render=%5lu text=%4lu spi=%5lu  (us/frame)\n",
                   (uint32_t)(32000000UL / (elapsed ? elapsed : 1)),
                   p_wait/n, p_read/n, p_calc/n, p_scale/n, p_fifo0/n,
                   p_fifo1/n, p_render/n, p_text/n, p_spi/n);
            p_loops = 0;
            p_wait = p_read = p_calc = p_scale = p_fifo0 = 0;
            p_fifo1 = p_render = p_text = p_spi = 0;
        }
#endif

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
        MLX90640_CalculateTo(captureFrame, params, EMISSIVITY, eTa, values);
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

        // create a DTO for the renderer
        PROF_T(t_f0);
        FrameDTO *dto = (FrameDTO*)multicore_fifo_pop_blocking();
        PROF_ACC(p_fifo0, t_f0);
        TRACE("got DTO from core1");
        dto->min = min;
        dto->max = max;

        PROF_T(t_scale2);
        float step = (ceil(max + 1.0) - floor(min - 1.0)) / float(HEAT_MAP_SIZE-1);
        for (int i = 0; i < MLX90640_PIXEL_NUM; i++) {
            dto->values[i] = uint8_t((values[i] - min) / step);
        }
        PROF_ACC(p_scale, t_scale2);
        multicore_fifo_push_blocking((uint32_t)dto);
        TRACE("pushed DTO to core1");
        if (p_trace > 0) p_trace--;
    }

    delete values;
    delete captureFrame;
    delete params;
    return 0;
}
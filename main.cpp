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
#include <string.h>
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

// temporal average over this many frames, to damp sensor noise. 1 disables it.
constexpr int SMOOTH_FRAMES = 1;

// The palette was rescaled to the scene every frame from the raw min and max,
// the two most noise-sensitive statistics available. Measured unsmoothed, the
// range was rebuilt ~20 times a second and the span wandered over 9..12C, which
// rescales every pixel by up to 30% a frame and makes the image pulse.
// The endpoints are smoothed, but only when they contract: anything new in the
// scene widens the range immediately, so nothing is ever clipped and there is
// no per-pixel clamp to pay for.
constexpr float RANGE_SMOOTH = 0.25f;   // ~0.13s to settle back in at 31fps
constexpr float RANGE_MARGIN = 1.0f;    // degrees of headroom each side

// The sensor carries a column pattern of period 4 -- every 4th column shares an
// ADC channel -- whose amplitude and phase wander at a few Hz. Measured live it
// averages 1179mK and peaks at 5383mK, while its multi-frame average is only
// 367mK, so a static correction cancels itself out and does nothing. It is only
// two numbers per frame though (the cosine and sine amplitude at period 4), so
// it can be measured and notched out frame by frame. Two degrees of freedom at
// exactly 4-column period, which real scenes essentially never contain.
#define NOTCH4 1
static float notch4[4] = {0, 0, 0, 0};
#define NOTCH(i) (notch4[(i) & 3])

#define FLIP_GRAPH_HORIZONTAL 0
#define FLIP_GRAPH_VERTICAL 1

// ---- profiling ----

#define PROFILE 1

#if PROFILE
// per-stage microsecond accumulators, summed over a 1 second window
static volatile uint32_t p_wait, p_read, p_calc, p_scale, p_fifo0;  // core0 stages
static volatile uint32_t p_render, p_text, p_spi, p_fifo1;          // core1 stages
static uint32_t p_loops = 0;
static uint32_t p_window_start = 0;
static volatile uint32_t p_latency = 0;   // sensor-ready to pixels-on-screen
static volatile uint32_t p_l1 = 0, p_l2 = 0, p_l3 = 0;  // core0 / queue / core1 shares
static char p_boot_line[192];
static int p_boot_repeats = 3;
#define PROF_T(v)        uint32_t v = time_us_32()
#define PROF_ACC(acc, v) acc += time_us_32() - (v)
#else
#define PROF_T(v)        do {} while (0)
#define PROF_ACC(acc, v) do {} while (0)
#endif

// ---- other constants (don't change) ----

constexpr uint8_t MLX_I2C_ADDR = 0x33;          // I2C address of the MLX90640

constexpr uint16_t BANNER_HEIGHT = 16;                                  // bottom strip holding min / fps / max
constexpr uint32_t BANNER_INTERVAL_US = 350000;                         // ~3 banner refreshes a second
constexpr uint16_t BANNER_TOP = DISPLAY_HEIGHT - BANNER_HEIGHT;         // info bar along the bottom edge
constexpr uint16_t GRAPH_WIDTH = DISPLAY_WIDTH;                         // width of the thermal camera graph on the OLED display
// the largest true-aspect image a 128 wide area can hold: the sensor is 32x24,
// so width is always the binding constraint and 96 is the tallest 4:3 fit
constexpr uint16_t GRAPH_HEIGHT = (GRAPH_WIDTH * MLX90640_LINE_NUM) / MLX90640_COLUMN_NUM;

// centre the image on the panel: the strip left above it carries the status
// icons, the strip below it is the info bar
constexpr uint16_t IMAGE_TOP = (DISPLAY_HEIGHT - GRAPH_HEIGHT) / 2;

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

// Derived from the May 2025 "version stan" palette. The gradient spans
// (stops - 1) segments and the leading blacks consume the first of them, so
// three blacks out of nine stops hold the coldest quarter of the range at
// black. Fewer blacks show more detail in cool parts of the scene; more hide
// low-end noise.
constexpr RGBColor colors[] = {RGB_BLACK, RGB_BLACK, RGB_BLACK,
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

    SSD1351_update();   // clear the whole panel once; rows below the banner are never pushed again

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

        // Average the palette indices over the last few frames. The MLX90640 is
        // noisy enough per-frame that this reads as a clear improvement, and core1
        // has the idle time for it. Costs roughly one frame of extra motion lag.
        const uint8_t *src = dto->values;
        if constexpr (SMOOTH_FRAMES > 1) {
            static uint8_t hist[SMOOTH_FRAMES][MLX90640_PIXEL_NUM];
            static uint8_t smoothed[MLX90640_PIXEL_NUM];
            static int hist_pos = 0;
            static bool hist_primed = false;

            if (!hist_primed) {
                // seed every slot from the first frame, so the divisor below is
                // always the constant SMOOTH_FRAMES and never a runtime divide
                for (int f = 0; f < SMOOTH_FRAMES; f++) {
                    memcpy(hist[f], dto->values, MLX90640_PIXEL_NUM);
                }
                hist_primed = true;
            } else {
                memcpy(hist[hist_pos], dto->values, MLX90640_PIXEL_NUM);
                hist_pos = (hist_pos + 1) % SMOOTH_FRAMES;
            }

            for (int i = 0; i < MLX90640_PIXEL_NUM; i++) {
                unsigned sum = 0;
                for (int f = 0; f < SMOOTH_FRAMES; f++) {
                    sum += hist[f][i];
                }
                smoothed[i] = (uint8_t)(sum / SMOOTH_FRAMES);
            }
            src = smoothed;
        }

        // integer bilinear interpolation off the precomputed tables
        for (int y = 0; y < GRAPH_HEIGHT; y++) {

            const int32_t y0 = ipy0[y];
            const int32_t ty = ipyf[y];
            const uint8_t *row0 = &src[y0 << 5];
            const uint8_t *row1 = row0 + MLX90640_COLUMN_NUM;

            for (int x = 0; x < GRAPH_WIDTH; x++) {

                const int32_t x0 = ipx0[x];
                const int32_t tx = ipxf[x];

                int32_t v00 = row0[x0], v10 = row0[x0+1];
                int32_t v01 = row1[x0], v11 = row1[x0+1];

                int32_t s = v00 + ((tx * (v10-v00)) >> 8);
                int32_t e = v01 + ((tx * (v11-v01)) >> 8);
                int32_t v = s + ((ty * (e-s)) >> 8);

                SSD1351_pixel(TR_X(x), IMAGE_TOP + TR_Y(y), palette[v]);
            }
        }

        PROF_ACC(p_render, t_render);

        float min = dto->min;
        float max = dto->max;

        PROF_T(t_text);

        // the banner is only readable at a few updates a second, so redrawing and
        // re-sending it every frame would spend ~2.5ms of SPI on nothing
        static uint32_t banner_due = 0;
        const uint32_t now_us = time_us_32();
        const bool banner_dirty = (int32_t)(now_us - banner_due) >= 0;
        if (banner_dirty) {
            banner_due = now_us + BANNER_INTERVAL_US;

            int y = BANNER_TOP;
            SSD1351_fillrect(0, y, DISPLAY_WIDTH, BANNER_HEIGHT, BLACK);

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
        }

        PROF_ACC(p_text, t_text);

        PROF_T(t_spi);
        if (banner_dirty) {
            SSD1351_update_rows(0, DISPLAY_HEIGHT - 1);
        } else {
            SSD1351_update_rows(IMAGE_TOP, IMAGE_TOP + GRAPH_HEIGHT - 1);
        }
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

    const uint32_t t_boot_stdio = time_us_32();

    multicore_launch_core1(renderer);
    const uint32_t t_boot_core1 = time_us_32();

    MLX90640_I2CInit();

    // Wait for the MLX90640 to answer rather than sleeping a fixed 540ms. On a
    // warm reset the sensor is still powered and replies at once; on a cold start
    // this waits exactly as long as it actually needs. The settle delay after the
    // first ACK is margin for the sensor's own internal init.
    uint32_t probe_us = time_us_32();
    uint8_t probe;
    while (i2c_read_blocking(i2c0, MLX_I2C_ADDR, &probe, 1, false) < 0) {
        if (time_us_32() - probe_us > 1000000) {
            break;      // give up waiting and let the driver report the failure
        }
        sleep_ms(2);
    }
    const uint32_t t_boot_ack = time_us_32();
    sleep_ms(25);
    MLX90640_SetResolution(MLX_I2C_ADDR, 3);    // 0: 16 bit, 1: 17 bit, 2 = 18 bit, 3 = 19 bit
    MLX90640_SetRefreshRate(MLX_I2C_ADDR, REFRESH_RATE);
    MLX90640_SetChessMode(MLX_I2C_ADDR);

    const uint32_t t_boot_cfg = time_us_32();

    uint16_t *eeMLX90640 = new uint16_t[832];       // too large for allocating on stack
    int ee_err = MLX90640_DumpEE(MLX_I2C_ADDR, eeMLX90640);
    const uint32_t t_boot_ee = time_us_32();
    paramsMLX90640 *params = new paramsMLX90640;    // too large for allocating on stack
    int pe_err = MLX90640_ExtractParameters(eeMLX90640, params);
    const uint32_t t_boot_params = time_us_32();
    delete eeMLX90640;

    uint16_t *captureFrame = new uint16_t[834];     // too large for allocating on stack 
    // value-initialised: in chess mode CalculateTo only writes half the pixels, so
    // on the first frame the rest would otherwise be uninitialised heap
    float *values = new float[MLX90640_PIXEL_NUM]();
    int patternMode = MLX90640_GetCurMode(MLX_I2C_ADDR);
    FrameDTO *dto = NULL;   // held across iterations so a skipped frame never loses it
    snprintf(p_boot_line, sizeof(p_boot_line),
             "boot (ms from power-on): stdio=%lu core1=%lu sensorcfg=%lu dumpEE=%lu"
             " params=%lu loop=%lu  [sensorACK=%lu EE->%d params->%d mode=%d]",
             t_boot_stdio/1000, t_boot_core1/1000, t_boot_cfg/1000,
             t_boot_ee/1000, t_boot_params/1000, time_us_32()/1000,
             (t_boot_ack - probe_us)/1000, ee_err, pe_err, patternMode);
    printf("%s\n", p_boot_line);


    while (true) {

#if PROFILE
        // report every N frames from core0 itself, so the numbers do not depend
        // on the repeating timer (which is registered from core1) ever firing
        if (++p_loops >= 32) {
            uint32_t now = time_us_32();
            uint32_t elapsed = now - p_window_start;
            p_window_start = now;
            const uint32_t n = 32;
            if (p_boot_repeats > 0) { p_boot_repeats--; printf("%s\n", p_boot_line); }
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

        // read pages (half frames) from the MLX90640
        PROF_T(t_read);
        int status = MLX90640_GetFrameData(MLX_I2C_ADDR, captureFrame);
        PROF_ACC(p_read, t_read);
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

#if NOTCH4
        // Measure the period-4 column component of this frame. At period 4 the
        // basis is cos = 1,0,-1,0 and sin = 0,1,0,-1, so the projection is just
        // four running sums. The column means are high-passed first so that real
        // scene gradient does not leak into the estimate.
        {
            float cm[MLX90640_COLUMN_NUM];
            for (int x = 0; x < MLX90640_COLUMN_NUM; x++) {
                float acc = 0;
                for (int y = 0; y < MLX90640_LINE_NUM; y++) acc += values[y * MLX90640_COLUMN_NUM + x];
                cm[x] = acc * (1.0f / MLX90640_LINE_NUM);
            }
            float c = 0, sn = 0;
            for (int x = 0; x < MLX90640_COLUMN_NUM; x++) {
                int lo = x - 3, hi = x + 3;
                if (lo < 0) lo = 0;
                if (hi > MLX90640_COLUMN_NUM - 1) hi = MLX90640_COLUMN_NUM - 1;
                float a = 0;
                for (int i = lo; i <= hi; i++) a += cm[i];
                const float hp = cm[x] - a / (hi - lo + 1);
                switch (x & 3) {
                    case 0: c  += hp; break;
                    case 1: sn += hp; break;
                    case 2: c  -= hp; break;
                    default: sn -= hp; break;
                }
            }
            c *= 0.125f; sn *= 0.125f;      // -> 2x the cosine/sine amplitude
            notch4[0] =  c * 0.5f;
            notch4[1] =  sn * 0.5f;
            notch4[2] = -c * 0.5f;
            notch4[3] = -sn * 0.5f;
        }
#endif

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


        static float smin = 0, smax = 0;
        static bool range_init = false;
        // only seed from a sane frame: seeding from a NaN would poison the
        // smoothed range permanently
        if (!range_init && min <= max && min > -200.0f && max < 1000.0f) {
            smin = min; smax = max; range_init = true;
        }
        smin += (min - smin) * RANGE_SMOOTH;
        smax += (max - smax) * RANGE_SMOOTH;
        // Deliberately NOT widened to the raw min/max: a single outlier pixel
        // would then stretch the range for that frame and compress the whole
        // scene into the dark end of the palette, which shows as a black frame.
        // Out-of-range pixels are clamped to the end colours instead.
        const float lo = smin - RANGE_MARGIN;
        const float hi = smax + RANGE_MARGIN;

        PROF_ACC(p_scale, t_scale);

        dto->t_ready = t_ready;
        dto->min = lo;
        dto->max = hi;

        PROF_T(t_scale2);
        const float step = (hi - lo) / float(HEAT_MAP_SIZE-1);
        const float inv_step = 1.0f / step;
        const float off[4] = { lo + notch4[0], lo + notch4[1],
                               lo + notch4[2], lo + notch4[3] };
        for (int i = 0; i < MLX90640_PIXEL_NUM; i++) {
            int idx = (int)((values[i] - off[i & 3]) * inv_step);
            if (idx < 0) idx = 0; else if (idx > HEAT_MAP_SIZE-1) idx = HEAT_MAP_SIZE-1;
            dto->values[i] = (uint8_t)idx;
        }
        PROF_ACC(p_scale, t_scale2);
        dto->t_push = time_us_32();
        multicore_fifo_push_blocking((uint32_t)dto);
        dto = NULL;
    }

    delete values;
    delete captureFrame;
    delete params;
    return 0;
}
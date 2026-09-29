/**
 * Single-precision replacement for MLX90640_CalculateTo.
 *
 * The upstream version is written against double literals (SCALEALPHA is
 * 0.000001, the Kelvin offsets are 273.15) and calls the double sqrt, so the
 * whole per-pixel chain runs in double on a core with no FPU. The result is
 * then quantised to one of 256 palette steps, so that precision is discarded
 * anyway. This computes the same expressions entirely in float.
 */
#include <math.h>
#include <stdint.h>

extern "C" {
#include <MLX90640_API.h>
}

#define SCALEALPHA_F 0.000001f

static inline float quadrt(float x) {
    return sqrtf(sqrtf(x));
}

extern "C"
void MLX90640_CalculateTo_fast(uint16_t *frameData, const paramsMLX90640 *params,
                               float emissivity, float tr, float *result)
{
    const uint16_t subPage = frameData[833];
    const float vdd = MLX90640_GetVdd(frameData, params);
    const float ta  = MLX90640_GetTa(frameData, params);

    float ta4 = ta + 273.15f;
    ta4 = ta4 * ta4;
    ta4 = ta4 * ta4;
    float tr4 = tr + 273.15f;
    tr4 = tr4 * tr4;
    tr4 = tr4 * tr4;
    const float taTr = tr4 - (tr4 - ta4) / emissivity;

    const float ktaScale   = ldexpf(1.0f, params->ktaScale);
    const float kvScale    = ldexpf(1.0f, params->kvScale);
    const float alphaScale = ldexpf(1.0f, params->alphaScale);

    // loop-invariant reciprocals, so the per-pixel path has no divides except
    // the one by alpha[], which varies per pixel
    const float invKtaScale   = 1.0f / ktaScale;
    const float invKvScale    = 1.0f / kvScale;
    const float invEmissivity = 1.0f / emissivity;
    const float alphaNum      = SCALEALPHA_F * alphaScale;

    const float taM25   = ta - 25.0f;
    const float vddM33  = vdd - 3.3f;
    const float ksTo1   = params->ksTo[1];
    const float ksTo1K  = 1.0f - ksTo1 * 273.15f;
    const float ksTaTerm = 1.0f + params->KsTa * taM25;

    float alphaCorrR[4];
    alphaCorrR[0] = 1.0f / (1.0f + params->ksTo[0] * 40.0f);
    alphaCorrR[1] = 1.0f;
    alphaCorrR[2] = 1.0f + ksTo1 * params->ct[2];
    alphaCorrR[3] = alphaCorrR[2] * (1.0f + params->ksTo[2] * (params->ct[3] - params->ct[2]));

    const float gain = (float)params->gainEE / (float)(int16_t)frameData[778];

    const uint8_t mode = (frameData[832] & MLX90640_CTRL_MEAS_MODE_MASK) >> 5;

    const float cpCorr = (1.0f + params->cpKta * taM25) * (1.0f + params->cpKv * vddM33);
    float irDataCP[2];
    irDataCP[0] = (int16_t)frameData[776] * gain - params->cpOffset[0] * cpCorr;
    if (mode == params->calibrationModeEE) {
        irDataCP[1] = (int16_t)frameData[808] * gain - params->cpOffset[1] * cpCorr;
    } else {
        irDataCP[1] = (int16_t)frameData[808] * gain
                    - (params->cpOffset[1] + params->ilChessC[0]) * cpCorr;
    }
    const float tgcCP = params->tgc * irDataCP[subPage];

    const int8_t frameSub = (int8_t)frameData[833];

    for (int pixelNumber = 0; pixelNumber < 768; pixelNumber++) {

        const int8_t ilPattern = pixelNumber / 32 - (pixelNumber / 64) * 2;
        const int8_t chessPattern = ilPattern ^ (pixelNumber - (pixelNumber / 2) * 2);
        const int8_t pattern = (mode == 0) ? ilPattern : chessPattern;

        if (pattern != frameSub) {
            continue;
        }

        const int8_t conversionPattern =
            ((pixelNumber + 2) / 4 - (pixelNumber + 3) / 4
           + (pixelNumber + 1) / 4 - pixelNumber / 4) * (1 - 2 * ilPattern);

        float irData = (int16_t)frameData[pixelNumber] * gain;

        const float kta = params->kta[pixelNumber] * invKtaScale;
        const float kv  = params->kv[pixelNumber] * invKvScale;
        irData -= params->offset[pixelNumber] * (1.0f + kta * taM25) * (1.0f + kv * vddM33);

        if (mode != params->calibrationModeEE) {
            irData += params->ilChessC[2] * (2 * ilPattern - 1)
                    - params->ilChessC[1] * conversionPattern;
        }

        irData = (irData - tgcCP) * invEmissivity;

        float alphaCompensated = (alphaNum / params->alpha[pixelNumber]) * ksTaTerm;

        float Sx = alphaCompensated * alphaCompensated * alphaCompensated
                 * (irData + alphaCompensated * taTr);
        Sx = quadrt(Sx) * ksTo1;

        float To = quadrt(irData / (alphaCompensated * ksTo1K + Sx) + taTr) - 273.15f;

        int8_t range;
        if (To < params->ct[1])      range = 0;
        else if (To < params->ct[2]) range = 1;
        else if (To < params->ct[3]) range = 2;
        else                         range = 3;

        To = quadrt(irData / (alphaCompensated * alphaCorrR[range]
                              * (1.0f + params->ksTo[range] * (To - params->ct[range])))
                    + taTr) - 273.15f;

        result[pixelNumber] = To;
    }
}

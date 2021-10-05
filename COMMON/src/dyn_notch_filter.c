/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/* original work by Rav
 * 
 * 2018_07 updated by ctzsnooze to post filter, wider Q, different peak detection
 * coding assistance and advice from DieHertz, Rav, eTracer
 * test pilots icr4sh, UAV Tech, Flint723
 * 
 * 2021_02 updated by KarateBrot: switched FFT with SDFT, multiple notches per axis
 * test pilots: Sugar K, bizmar
 */

#include <math.h>
#include "config.h"



#ifdef USE_DYN_NOTCH_FILTER


#include "axis.h"
#include "filter.h"
#include "maths.h"
#include "sdft.h"
#include "myqueue.h"
#include "time.h"

#include "dyn_notch_filter.h"
#include "stabilizer.h"

// SDFT_SAMPLE_SIZE defaults to 72 (common/sdft.h).
// We get 36 frequency bins from 72 consecutive data values, called SDFT_BIN_COUNT (common/sdft.h)
// Bin 0 is DC and can't be used.
// Only bins 1 to 35 are usable.

// A gyro sample is collected every PID loop.
// sampleCount recent gyro values are accumulated and averaged
// to ensure that 72 samples are collected at the right rate for the required SDFT bandwidth.

// For an 8k PID loop, at default 600hz max, 6 sequential gyro data points are averaged, SDFT runs 1333Hz.
// Upper limit of SDFT is half that frequency, eg 666Hz by default.
// At 8k, if user sets a max of 300Hz, int(8000/600) = 13, sdftSampleRateHz = 615Hz, range 307Hz.
// Note that lower max requires more samples to be averaged, increasing precision but taking longer to get enough samples.
// For Bosch at 3200Hz gyro, max of 600, int(3200/1200) = 2, sdftSampleRateHz = 1600, range to 800hz.
// For Bosch on XClass, better to set a max of 300, int(3200/600) = 5, sdftSampleRateHz = 640, range to 320Hz.

// When sampleIndex reaches sampleCount, the averaged gyro value is put into the corresponding SDFT.
// At 8k, with 600Hz max, sampleCount = 6, this happens every 6 * 0.125us, or every 0.75ms.
// Hence to completely replace all 72 samples of the SDFT input buffer with clean new data takes 54ms.

// The SDFT code is split into steps. It takes 4 PID loops to calculate the SDFT, track peaks and update the filters for one axis.
// Since there are three axes, it takes 12 PID loops to completely update all axes.
// At 8k, any one axis gets updated at 8000 / 12 or 666hz or every 1.5ms
// In this time, 2 points in the SDFT buffer will have changed.
// At 4k, it takes twice as long to update an axis, i.e. each axis updates only every 3ms.
// Four points in the buffer will have changed in that time, and each point will be the average of three samples.
// Hence output jitter at 4k is about four times worse than at 8k. At 2k output jitter is quite bad.

// Each SDFT output bin has width sdftSampleRateHz/72, ie 18.5Hz per bin at 1333Hz.
// Usable bandwidth is half this, ie 666Hz if sdftSampleRateHz is 1333Hz, i.e. bin 1 is 18.5Hz, bin 2 is 37.0Hz etc.

#define DYN_NOTCH_SMOOTH_HZ        4
#define NOTCH_AXIS_COUNT 1
#define DYN_NOTCH_CALC_TICKS       (NOTCH_AXIS_COUNT * STEP_COUNT) // 3 axes and 4 steps per axis
#define DYN_NOTCH_OSD_MIN_THROTTLE 20
#define PEAKS_SEARCH_STARTBIN   15  //从30Hz以上开始搜索
#define DYN_COMB_QUEUE_SIZE 50

typedef enum {

    STEP_WINDOW,
    STEP_DETECT_PEAKS,
    STEP_CALC_FREQUENCIES,
    STEP_UPDATE_FILTERS,
    STEP_COUNT
} step_e;



typedef struct state_s {

    // state machine step information
    int tick;
    int step;
    int axis;

} Notchupdate_state_t;

typedef struct dynNotch_s {

    float q;
    float minHz;
    float maxHz;
    int count;

    uint16_t maxCenterFreq;
    float centerFreq[XYZ_AXIS_COUNT][DYN_NOTCH_COUNT_MAX];
    
    timeUs_t looptimeUs;
    biquadFilter_t notch[XYZ_AXIS_COUNT][DYN_NOTCH_COUNT_MAX];

} dynNotch_t;

typedef struct dynComb_s {
    float minHz;
    float maxHz;
    float centerFreq;
    uint16_t sampleHz;
    combFilter_t combfilter;
} dynComb_t;

// dynamic notch instance (singleton)
static dynNotch_t dynNotch;
static dynComb_t dynComb;
static QUEUE rollrate_queue;


// accumulator for oversampled data => no aliasing and less noise
static  int   sampleIndex;
static  int   sampleCount;
static  float sampleCountRcp;
static  float sampleAccumulator[XYZ_AXIS_COUNT];

// downsampled data for frequency analysis
static  float sampleAvg[XYZ_AXIS_COUNT];

// parameters for peak detection and frequency analysis
static  Notchupdate_state_t state;
static  sdft_t  sdft[XYZ_AXIS_COUNT];
static  peak_t  peaks[DYN_NOTCH_COUNT_MAX];
static  float   sdftData[SDFT_BIN_COUNT];
static  float   sdftSampleRateHz;
static  float   sdftResolutionHz;
static  int     sdftStartBin;
static  int     sdftEndBin;
static  float   sdftMeanSq;
static  float   gain;
uint8_t times = 0;
uint8_t error[3] = {0};
static uint8_t flag = 0;
static uint8_t base_freq = 0;

void dynNotchInit(const dynNotchConfig_t *config, const timeUs_t targetLooptimeUs)
{
    // initialise even if FEATURE_DYNAMIC_FILTER not set, since it may be set later
    dynNotch.q = config->dyn_notch_q / 100.0f;
    dynNotch.minHz = config->dyn_notch_min_hz;
    dynNotch.maxHz = MAX(2 * dynNotch.minHz, config->dyn_notch_max_hz);
    dynNotch.count = config->dyn_notch_count;  //the number of dynnotch filter in every axis
    dynNotch.looptimeUs = targetLooptimeUs;
    dynNotch.maxCenterFreq = 0;

    // dynNotchUpdate() is running at looprateHz (which is PID looprate aka. 1e6f / gyro.targetLooptime)
    const float looprateHz = 1.0f / dynNotch.looptimeUs * 1e6f;
    sampleCount = MAX(1, looprateHz / (2 * dynNotch.maxHz)); // sanmplerate between every downsample step: 4.00 when maxHz=125 hz, gyro sample = 1k,  
    sampleCountRcp = 1.0f / sampleCount;

    sdftSampleRateHz = looprateHz / sampleCount; // dynNotch downsampling looprateHz: 250Hz when maxHz=125 hz, gyro sample = 1k,
    // eg 8k, user max 250hz, int(1000/250) = 4 , sdftSampleRateHz = 250hz, range 125Hz
    // the upper limit of DN is always going to be the Nyquist frequency (= sampleRate / 2)

    sdftResolutionHz = sdftSampleRateHz / SDFT_SAMPLE_SIZE; // 1.95hz per bin at 1k and 250Hz maxHz
    sdftStartBin = MAX(1, dynNotch.minHz / sdftResolutionHz + 0.5f); // can't use bin 0 because it is DC.
    sdftEndBin = MIN(SDFT_BIN_COUNT - 1, dynNotch.maxHz / sdftResolutionHz + 0.5f); // can't use more than SDFT_BIN_COUNT bins.
    gain = pt1FilterGain(DYN_NOTCH_SMOOTH_HZ, DYN_NOTCH_CALC_TICKS / looprateHz); // minimum PT1 k value

    sdftInit(&sdft[0], sdftStartBin, sdftEndBin, sampleCount);
    
    for (int p = 0; p < dynNotch.count; p++) {
         // any init value is fine, but evenly spreading centerFreqs across frequency range makes notch filters stick to peaks quicker
        dynNotch.centerFreq[0][p] = (p + 0.5f) * (dynNotch.maxHz - dynNotch.minHz) / (float)dynNotch.count + dynNotch.minHz;
        biquadFilterInit(&dynNotch.notch[0][p], dynNotch.centerFreq[0][p], dynNotch.looptimeUs, dynNotch.q, FILTER_NOTCH, 1.0f);
    }
}
void dynCombInit(const dynCombConfig_t *config, const uint16_t sampleHZ)
{
    // initialise even if FEATURE_DYNAMIC_FILTER not set, since it may be set later
    dynComb.minHz = config->dyn_comb_min_hz;
    dynComb.maxHz = MAX(2 * dynComb.minHz, config->dyn_comb_max_hz);
    //dynComb.looptimeUs = targetLooptimeUs;
    // dynNotchUpdate() is running at looprateHz (which is PID looprate aka. 1e6f / gyro.targetLooptime)
    const uint16_t looprateHz = sampleHZ;
    sampleCount = MAX(1, looprateHz / (2 * dynComb.maxHz)); // sanmplerate between every downsample step: 4.00 when maxHz=125 hz, gyro sample = 1k,  
    sampleCountRcp = 1.0f / sampleCount;

    sdftSampleRateHz = looprateHz / sampleCount; // dynComb downsampling looprateHz: 250Hz when maxHz=125 hz, gyro sample = 1k,
    // eg 8k, user max 250hz, int(1000/250) = 4 , sdftSampleRateHz = 250hz, range 125Hz
    // the upper limit of DN is always going to be the Nyquist frequency (= sampleRate / 2)

    sdftResolutionHz = sdftSampleRateHz / SDFT_SAMPLE_SIZE; // 1.95hz per bin at 1k and 250Hz maxHz
    sdftStartBin = MAX(1, dynComb.minHz / sdftResolutionHz + 0.5f); // can't use bin 0 because it is DC.
    sdftEndBin = MIN(SDFT_BIN_COUNT - 1, dynComb.maxHz / sdftResolutionHz + 0.5f); // can't use more than SDFT_BIN_COUNT bins.
    gain = pt1FilterGain(DYN_NOTCH_SMOOTH_HZ, DYN_NOTCH_CALC_TICKS / looprateHz); // minimum PT1 k value

    sdftInit(&sdft[0], sdftStartBin, sdftEndBin, sampleCount);
    dynComb.centerFreq = config->centerFreq;
    combFilterInit(&dynComb.combfilter, looprateHz / dynComb.centerFreq , config->coefficient);
    initQueue(&rollrate_queue,DYN_COMB_QUEUE_SIZE);
}
// Collect gyro data, to be downsampled and analysed in dynNotchUpdate() function
void dynNotchPush(const int axis, const float sample)
{
    sampleAccumulator[axis] += sample;
    In_Queue(&rollrate_queue , sample);
}

static void dynNotchProcess(void);

// Downsample and analyse gyro data
void dynNotchUpdate(axis_e axis)
{
    state.axis  = axis;
    // samples should have been pushed by `dynNotchPush`
    // if gyro sampling is > 1kHz, accumulate and average multiple gyro samples
    if (sampleIndex == sampleCount) {
        sampleIndex = 0;

        // calculate mean value of accumulated samples
        sampleAvg[axis] = sampleAccumulator[axis] * sampleCountRcp;
        sampleAccumulator[axis] = 0;

        // We need DYN_NOTCH_CALC_TICKS ticks to update all axes with newly sampled value
        // recalculation of filters takes 4 calls per axis => each filter gets updated every DYN_NOTCH_CALC_TICKS calls
        // at 8kHz PID loop rate this means 8kHz / 4 / 3 = 666Hz => update every 1.5ms
        // at 4kHz PID loop rate this means 4kHz / 4 / 3 = 333Hz => update every 3ms
        state.tick = DYN_NOTCH_CALC_TICKS;
    }

    // 2us @ F722
    // SDFT processing in batches to synchronize with incoming downsampled data
    sdftPushBatch(&sdft[axis], sampleAvg[axis], sampleIndex);
    sampleIndex++;

    // Find frequency peaks and update filters
    if (state.tick > 0) {
        dynNotchProcess();
        --state.tick;
    }
}

// Find frequency peaks and update filters
static  void dynNotchProcess(void)
{
    switch (state.step) {
        case STEP_WINDOW: // 6us @ F722
        {
            sdftWinSq(&sdft[state.axis], sdftData);
            
            // Calculate mean square over frequency range (= average power of vibrations)
            sdftMeanSq = 0.0f;
            for (int bin = (sdftStartBin + 1); bin < sdftEndBin; bin++) {   // don't use startBin or endBin because they are not windowed properly
                sdftMeanSq += sdftData[bin];                                // sdftData is already squared (see sdftWinSq)
            }
            sdftMeanSq /= sdftEndBin - sdftStartBin - 1;
            if(sdftMeanSq <= 8.0f)
                sdftMeanSq = 8.0f;
            break;
        }
        case STEP_DETECT_PEAKS: // 6us @ F722
        {
            // Get memory ready for new peak data on current axis
            for (int p = 0; p < DYN_NOTCH_COUNT_MAX; p++) {
                peaks[p].bin = 0;
                peaks[p].value = 0.0f;
            }

            // Search for N biggest peaks in frequency spectrum
            for (int bin = PEAKS_SEARCH_STARTBIN; bin < sdftEndBin; bin++) {
                // Check if bin is peak
                if((sdftData[bin] > sdftData[bin - 1]) && (sdftData[bin] > sdftData[bin + 1])&& 
                   (sdftData[bin] > sdftData[bin - 2]) && (sdftData[bin] > sdftData[bin + 2])&& 
                   (sdftData[bin] > sdftMeanSq)) {
                    // Check if peak is big enough to be one of N biggest peaks.
                    // If so, insert peak and sort peaks in descending height order
                    for (int p = 0; p < DYN_NOTCH_COUNT_MAX; p++) {
                        if (sdftData[bin] > peaks[p].value) {
                            for (int k = DYN_NOTCH_COUNT_MAX - 1; k > p; k--) {
                                peaks[k] = peaks[k - 1];
                            }
                            peaks[p].bin = bin;
                            peaks[p].value = sdftData[bin];
                            break;
                        }
                    }
                    bin= bin + 4;// If bin is peak, next four bins can't be peak => jump it
                }
            }

            // Sort N biggest peaks in ascending bin order (example: 3, 8, 25, 0, 0, ..., 0)
            for (int p = DYN_NOTCH_COUNT_MAX- 1; p > 0; p--) {
                for (int k = 0; k < p; k++) {
                    // Swap peaks but ignore swapping void peaks (bin = 0). This leaves
                    // void peaks at the end of peaks array without moving them
                    if (peaks[k].bin > peaks[k + 1].bin && peaks[k + 1].bin != 0) {
                        peak_t temp = peaks[k];
                        peaks[k] = peaks[k + 1];
                        peaks[k + 1] = temp;
                    }
                }
            }
            break;
        }
        case STEP_CALC_FREQUENCIES: // 4us @ F722
        {
            for (int i = 13; i >= 7; i-- ){
                for(int p = 0; p < DYN_NOTCH_COUNT_MAX; p++){
                    if(peaks[p].bin != 0){
                        times = ROUND((float)(peaks[p].bin - 1) / i);
                        error[p] = ABS((peaks[p].bin -1) - i * times);
                        if(error[p] <= times * 0.75)
                            flag++;
                    }
                    else{
                        flag = 0;
                    }
                } 
                if(flag == 3)
                {
                    base_freq = i;
                    break;
                }
                if((flag != 3)&&(i == 7))
                    base_freq = 0;
            }

            if(flag == 3){
                if (peaks[2].bin != 0 && peaks[2].value > sdftMeanSq) {

                    float meanBin = 0;

                    // Height of peak bin (y1) and shoulder bins (y0, y2)
                    const float y0 = sdftData[peaks[2].bin - 1];
                    const float y1 = sdftData[peaks[2].bin];
                    const float y2 = sdftData[peaks[2].bin + 1];

                    // Estimate true peak position aka. meanBin (fit parabola y(x) over y0, y1 and y2, solve dy/dx=0 for x)
                    const float denom = 2.0f * (y0 - 2 * y1 + y2);
                    if (denom != 0.0f) {
                        meanBin = ROUND((((y0 - y2) / denom)+ peaks[2].bin) * sdftSampleRateHz/ times);
                    }else{
                        meanBin = ROUND(peaks[2].bin * sdftSampleRateHz/ times);
                    }

                    // Convert bin to frequency: freq = bin * binResoultion (bin 0 is 0Hz)
                    const float centerFreq = constrainf(meanBin, dynNotch.minHz, dynNotch.maxHz);

                    // // PT1 style smoothing moves notch center freqs rapidly towards big peaks and slowly away, up to 8x faster 
                    // // DYN_NOTCH_SMOOTH_HZ = 4 & gainMultiplier = 1 .. 8  =>  PT1 -3dB cutoff frequency = 4Hz .. 41Hz
                    // const float gainMultiplier = constrainf(peaks[p].value / sdftMeanSq, 1.0f, 8.0f);

                    // // Finally update notch center frequency p on current axis
                    // float err_centerfreq = centerFreq - dynNotch.centerFreq[state.axis][p];
                    // if(ABS(err_centerfreq) > 0.5f)
                    // {
                        if(centerFreq != dynComb.centerFreq){
                            dynComb.centerFreq = centerFreq;
                            dynComb.combfilter.change_flag = true;
                        }

                        // dynNotch.notch[state.axis][p].change_flag = true;
                    // }
                }
                // if(calculateThrottlePercentAbs() > DYN_NOTCH_OSD_MIN_THROTTLE) {
                //     for (int p = 0; p < dynNotch.count; p++) {
                //         if(dynNotch.notch[state.axis][p].change_flag == true)
                //             dynNotch.maxCenterFreq = MAX(dynNotch.maxCenterFreq, dynNotch.centerFreq[state.axis][p]);
                //     }
                // }
            }
            break;
        }
        case STEP_UPDATE_FILTERS: // 7us @ F722
        {
            // for (int p = 0; p < dynNotch.count; p++) {
                // Only update notch filter coefficients if the corresponding peak got its center frequency updated in the previous step
                // if(dynNotch.notch[state.axis][0].change_flag == true)
                    // if (peaks[p].bin != 0 && peaks[p].value > sdftMeanSq) {
                    //     biquadFilterUpdate(&dynNotch.notch[state.axis][p], dynNotch.centerFreq[state.axis][p], dynNotch.looptimeUs, dynNotch.q, FILTER_NOTCH, 1.0f);
                    //     dynNotch.notch[state.axis][p].x1 = dynNotch.notch[state.axis][p].x2 = 0;
                    //     dynNotch.notch[state.axis][p].y1 = dynNotch.notch[state.axis][p].y2 = 0;
                    // }   
            // }

            // state.axis = (state.axis + 1) % XYZ_AXIS_COUNT;

            if(dynComb.combfilter.change_flag == true)
                combFilterupdate(&dynComb.combfilter, dynComb.sampleHz / dynComb.centerFreq);

        }
    }

    state.step = (state.step + 1) % STEP_COUNT;
}

float dynNotchFilter(const int axis, float value) 
{
    float _temp = 0;
    for (uint8_t p = 0; p < dynNotch.count; p++) {
        if(dynNotch.notch[axis][p].change_flag == true){  //if coefficient of filter was changed, firstly update the filter buffer(x1, x2.y1,y2) using the previous input data
            for(u8 i = 0; i < BUF_SIZE; i++)
            {
                out_Queue(&dynNotch.notch[axis][p].queue_buffer, &_temp);
                _temp = biquadFilterApplyDF1(&dynNotch.notch[axis][p], _temp);
            }
            dynNotch.notch[axis][p].change_flag = false;
        }
        In_Queue(&dynNotch.notch[axis][p].queue_buffer,value);
        value = biquadFilterApplyDF1(&dynNotch.notch[axis][p], value);
    }

    return value;
}
float dynCombFilter(const int axis, float value)
{
    return combFilterApply(&dynComb.combfilter, value, &rollrate_queue);
}

uint16_t getMaxFFT(void)
{
    return dynNotch.maxCenterFreq;
}

void resetMaxFFT(void)
{
    dynNotch.maxCenterFreq = 0;
}

//float * getdynNotchcenterfreq(axis_e axis)
//{
//    return dynNotch.centerFreq[axis];
//}

float (*getdynNotchcenterfreq(void))[DYN_NOTCH_COUNT_MAX]
{
   return dynNotch.centerFreq;
}

peak_t *getdynNotchpeak(void)
{
    return peaks;
}
#endif


#include <string>
#include "daisy_patch_sm.h"
#include "daisysp.h"


using namespace daisy;
using namespace daisysp;
using namespace patch_sm;

#define NUM_VOICES 32

// Hardware
DaisyPatchSM hw;
Switch button, toggle;

// Synthesis
PolyPluck<NUM_VOICES> synth;
ReverbSc              verb;

double mapInputToRange(double input, double input_start, double input_end, double output_start, double output_end)
{
    double slope = 1.0 * (output_end - output_start) / (input_end - input_start);
    return output_start + round(slope * (input - input_start));
}

void updateControls();

void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    float sig;           // Mono Audio Vars
    float trig, nn, decay;       // Pluck Vars
    float verbfbk;
    float dry, send, wetl, wetr; // Effects Vars

    button.Debounce();
    toggle.Debounce();

    // Assign Output Buffers
    float *out_left, *out_right;
    out_left  = out[0];
    out_right = out[1];

    hw.ProcessAllControls();

    // Handle Triggering the Plucks
    trig = 0.0f;
    if(hw.gate_in_1.Trig())
        trig = 1.0f;

    // Set MIDI Note for new Pluck notes.
    nn = 24.0f + hw.GetAdcValue(CV_5) * 60.0f;
    nn = static_cast<int32_t>(nn); // Quantize to semitones

    // Read knobs for decay;
    decay = 0.5f + (hw.GetAdcValue(CV_1) * 0.5f);
    synth.SetDecay(decay);

    verbfbk = hw.GetAdcValue(CV_2);

    // Synthesis.
    for(size_t i = 0; i < size; i++)
    {
        // Synthesize Plucks
        sig = synth.Process(trig, nn);

        // Create Reverb Send
        dry  = sig;
        send = dry * 0.6f;
        verb.SetFeedback(verbfbk);
        verb.Process(send, send, &wetl, &wetr);

        // Output
        out_left[i]  = dry + wetl;
        out_right[i] = dry + wetr;
    }
}

int main(void)
{
    // Init everything.
    float samplerate;
    hw.Init();
    samplerate = hw.AudioSampleRate();
    synth.Init(samplerate);

    verb.Init(samplerate);
    verb.SetFeedback(0.85f);

    button.Init(DaisyPatchSM::B7, hw.AudioCallbackRate());
    toggle.Init(DaisyPatchSM::B8, hw.AudioCallbackRate());

    // Start the ADC and Audio Peripherals on the Hardware
    hw.StartAdc();
    hw.StartAudio(AudioCallback);
    /** Loop forever */
    while(1)
    {
        /** Update all cv inputs */
        updateControls();
    }
}

void updateControls(){
    hw.ProcessAllControls();
}
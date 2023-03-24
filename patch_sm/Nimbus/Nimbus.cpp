#include "daisy_patch_sm.h"
#include "daisysp.h"
#include "dsp/granular_processor.h"

#define NUM_PARAMS 9
#define NUM_PAGES 2
using namespace daisysp;
using namespace daisy;
using namespace patch_sm;

GranularProcessorClouds processor;
DaisyPatchSM            hw;

// Pre-allocate big blocks in main memory and CCM. No malloc here.
uint8_t block_mem[118784];
uint8_t block_ccm[65536 - 128];

int mymod(int a, int b)
{
    return (b + (a % b)) % b;
}

class ParamControl
{
  public:
    ParamControl() {}
    ~ParamControl() {}

    void Init(AnalogControl* control, Parameters* params)
    {
        params_    = params;
        control_   = control;
        param_num_ = 0;
        oldval_    = 0.f;
    }

    void incParamNum(int inc)
    {
        param_num_ += inc;
        param_num_ = mymod(param_num_, NUM_PARAMS);
    }

    bool knobTouched(float newval)
    {
        bool ret = fabsf(newval - oldval_) > .001f;
        oldval_  = newval;
        return ret;
    }

    void Process()
    {
        float val = control_->Process();
        if(!knobTouched(val))
        {
            return;
        }

        switch(param_num_)
        {
            case 0: params_->position = val; break;
            case 1: params_->size = val; break;
            case 2:
                params_->pitch = powf(9.798f * (val - .5f), 2.f);
                params_->pitch *= val < .5f ? -1.f : 1.f;
                break;
            case 3: params_->density = val; break;
            case 4: params_->texture = val; break;
            case 5: params_->dry_wet = val; break;
            case 6: params_->stereo_spread = val; break;
            case 7: params_->feedback = val; break;
            case 8: params_->reverb = val; break;
        }
    }

  private:
    AnalogControl* control_;
    Parameters*    params_;
    int            param_num_;
    float          oldval_;
};

ParamControl paramControls[4];
Switch toggle, button;

bool freeze_btn;
int  pbMode;
int  quality;
int  increment;

Parameters* parameters;

void Controls();

void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    Controls();

    button.Debounce();
    toggle.Debounce();

    FloatFrame input[size];
    FloatFrame output[size];

    for(size_t i = 0; i < size; i++)
    {
        input[i].l  = in[0][i];
        input[i].r  = in[1][i];
        output[i].l = output[i].r = 0.f;
    }

    processor.Process(input, output, size);

    for(size_t i = 0; i < size; i++)
    {
        out[0][i] = output[i].l;
        out[1][i] = output[i].r;
    }
}

int main(void)
{
    hw.Init();
    hw.SetAudioBlockSize(32); // clouds won't work with blocks bigger than 32
    float sample_rate = hw.AudioSampleRate();

    //init the luts
    InitResources(sample_rate);

    processor.Init(sample_rate,
                   block_mem,
                   sizeof(block_mem),
                   block_ccm,
                   sizeof(block_ccm));

    parameters = processor.mutable_parameters();

    for(int i = 0; i < 4; i++)
    {
        paramControls[i].Init(&hw.controls[i], parameters);
    }

    paramControls[1].incParamNum(1);
    paramControls[2].incParamNum(3);
    paramControls[3].incParamNum(5);

    button.Init(DaisyPatchSM::B7, hw.AudioCallbackRate());
    toggle.Init(DaisyPatchSM::B8, hw.AudioCallbackRate());
    increment  = 0;

    hw.StartAdc();
    hw.StartAudio(AudioCallback);
    while(1)
    {
        processor.Prepare();
    }
}

void Controls()
{
    hw.ProcessAllControls();

    //process knobs
    for(int i = 0; i < 4; i++)
    {
        paramControls[i].Process();
    }

    freeze_btn = toggle.Pressed();
    pbMode += button.Pressed() ? 1 : 0;
    pbMode = mymod(pbMode, 4);
    processor.set_playback_mode((PlaybackMode)pbMode);
    processor.set_quality(quality);

    // gate ins
    parameters->freeze  = hw.gate_in_1.State() || freeze_btn;
    parameters->trigger = hw.gate_in_2.Trig();
    parameters->gate    = hw.gate_in_2.State();
}

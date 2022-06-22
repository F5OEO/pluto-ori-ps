#include <arm_neon.h>
static inline void volk_16i_s32f_convert_32f_neon(float* outputVector,
                                                  const int16_t* inputVector,
                                                  const float scalar,
                                                  unsigned int num_points)
{
    float* outputPtr = outputVector;
    const int16_t* inputPtr = inputVector;
    unsigned int number = 0;
    unsigned int eighth_points = num_points / 8;

    int16x4x2_t input16;
    int32x4_t input32_0, input32_1;
    float32x4_t input_float_0, input_float_1;
    float32x4x2_t output_float;
    float32x4_t inv_scale;

    inv_scale = vdupq_n_f32(1.0 / scalar);

    // the generic disassembles to a 128-bit load
    // and duplicates every instruction to operate on 64-bits
    // at a time. This is only possible with lanes, which is faster
    // than just doing a vld1_s16, but still slower.
    for (number = 0; number < eighth_points; number++) {
        input16 = vld2_s16(inputPtr);
        // widen 16-bit int to 32-bit int
        input32_0 = vmovl_s16(input16.val[0]);
        input32_1 = vmovl_s16(input16.val[1]);
        // convert 32-bit int to float with scale
        input_float_0 = vcvtq_f32_s32(input32_0);
        input_float_1 = vcvtq_f32_s32(input32_1);
        output_float.val[0] = vmulq_f32(input_float_0, inv_scale);
        output_float.val[1] = vmulq_f32(input_float_1, inv_scale);
        vst2q_f32(outputPtr, output_float);
        inputPtr += 8;
        outputPtr += 8;
    }

    for (number = eighth_points * 8; number < num_points; number++) {
        *outputPtr++ = ((float)(*inputPtr++)) / scalar;
    }
}
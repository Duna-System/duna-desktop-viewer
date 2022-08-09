#ifndef COLORMAP_HPP
#define COLORMAP_HPP

/* Author : Marcus Forte
# Update History
* 22-04-2022 


*/

// This method MAPS intensities to RGB colors
#include <math.h>

static void getRainbowColor(float value, float &r, float &g, float &b)
{
    // this is HSV color palette with hue values going only from 0.0 to 0.833333.

    value = std::min(value, 1.0f);
    value = std::max(value, 0.0f);

    float h = value * 5.0f + 1.0f;
    int i = floor(h);
    float f = h - i;
    if (!(i & 1))
        f = 1 - f; // if i is even
    float n = 1 - f;

    if (i <= 1)
        r = n, g = 0, b = 1;
    else if (i == 2)
        r = 0, g = n, b = 1;
    else if (i == 3)
        r = 0, g = 1, b = n;
    else if (i == 4)
        r = n, g = 1, b = 0;
    else if (i >= 5)
        r = 1, g = n, b = 0;
}

static void Int2RGB_rviz(float intensity, uint8_t &r, uint8_t &g, uint8_t &b)
{
    // const float min_c = 0;
    // const float max_c = 255;

    const float min_i = 10;
    const float max_i = 150;

    // Normalize
    float norm_i = 1.0 - ((intensity - min_i)/ (max_i - min_i));

    // Invert ? 
    // norm_i =  1.0 - norm_i;

    // float r_f = (norm_i * max_c) + ( (1-norm_i) * min_c);
    // float g_f = (norm_i * max_c) + ( (1-norm_i) * min_c);
    // float b_f = (norm_i * max_c) + ( (1-norm_i) * min_c);

    // Convert to HSV (rainbow)
    float r_f, g_f, b_f;
    getRainbowColor(norm_i, r_f, g_f, b_f);

    r = (uint8_t)(r_f * 255.0f);
    g = (uint8_t)(g_f * 255.0f);
    b = (uint8_t)(b_f * 255.0f);
}

static uint8_t color_from_interval(const uint8_t intensity, const uint8_t min, const uint8_t max)
{
    // Check if within range
    if(intensity < min || intensity > max)
        return 0;

    const uint8_t half = (max - min) / 2;
    {
        if (intensity < half)
            return (intensity - min) * (255 / half);
        else
            return (max - intensity) * (255 / half);
    }
}

// Assume intensity range [0 , 150]
void Int2RGB(float intensity, uint8_t &r, uint8_t &g, uint8_t &b)
{
    uint8_t intensity_i = (uint8_t) intensity;
    // std::cout << intensity << "<=>" << +intensity_i << std::endl;
    // Define Ranges
    const uint8_t b0 = 0;
    const uint8_t b1 = 50;

    const uint8_t g0 = 25;
    const uint8_t g1 = 90;

    const uint8_t r0 = 80;
    const uint8_t r1 = 150;
    r = color_from_interval(intensity_i, r0,r1);
    g = color_from_interval(intensity_i, g0,g1);
    b = color_from_interval(intensity_i, b0,b1);
}

#endif
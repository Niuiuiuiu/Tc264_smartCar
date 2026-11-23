#include "line_detect.h"
#include "zf_device_mt9v03x_double.h"

#define LINE_DETECT_ROW_STEP 5
#define LINE_DETECT_MIN_WIDTH_PX  1U
#define LINE_DETECT_MAX_WIDTH_PX  UINT16_MAX
#define GRAYSCALE_THRESHOLD 150

line_point_t points[MT9V03X_1_H / LINE_DETECT_ROW_STEP];

#include <stdlib.h>  // abs

size_t line_detect(uint8_t image[MT9V03X_1_H][MT9V03X_1_W],uint8_t gray_threshold,
                                            line_point_t *points,size_t capacity)
{
    size_t count = 0;
    int reference_col = MT9V03X_1_W / 2;

    for (int row = MT9V03X_1_H - 1;row >= 0 && count < capacity;row -= LINE_DETECT_ROW_STEP)
    {
        const uint8_t *pixels = image[row];
        int best_col = -1;
        int best_distance = MT9V03X_1_W;
        int col = 0;

        while (col < MT9V03X_1_W)
        {
            if (pixels[col] <= gray_threshold)
            {
                ++col;
                continue;
            }

            int start = col;
            while (col < MT9V03X_1_W && pixels[col] > gray_threshold)
            {
                ++col;
            }

            // col 已经在亮段后面，所以亮段范围是 start 到 col - 1
            int width = col - start;
            if (width < (int)LINE_DETECT_MIN_WIDTH_PX || width > (int)LINE_DETECT_MAX_WIDTH_PX)
            {
                continue;
            }

            int center = start + (width - 1) / 2;
            int distance = abs(center - reference_col);
            if (distance < best_distance)
            {
                best_col = center;
                best_distance = distance;
            }
        }

        if (best_col >= 0)
        {
            points[count].col = (uint16_t)best_col;
            points[count].row = (uint16_t)row;
            ++count;
            reference_col = best_col;
        }
    }

    return count;
}
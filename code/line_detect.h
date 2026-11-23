#ifndef _LINE_DETECT_H
#define _LINE_DETECT_H

#include <stddef.h>
#include <stdint.h>
#include "zf_device_mt9v03x_double.h"

typedef struct
{
    uint16_t col;  // 白线中心的列
    uint16_t row;  // 所在的行
} line_point_t;

size_t line_detect(uint8_t image[MT9V03X_1_H][MT9V03X_1_W],uint8_t gray_threshold,
                                            line_point_t *points,size_t capacity);






#endif /* _LINE_DETECT_H */
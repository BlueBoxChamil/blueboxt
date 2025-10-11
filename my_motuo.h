/*
 * @Author: BlueboxChamil
 * @Date: 2025-09-05 16:28:26
 * @LastEditTime: 2025-09-08 10:38:07
 * @FilePath: \examples\application\btdm_audio\Src\my_motuo.h
 * @Description:
 * Copyright (c) 2025 by BlueboxChamil, All Rights Reserved.
 */
#ifndef MY_MOTUO_H
#define MY_MOTUO_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "stdbool.h"
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include <time.h>

#include "driver_adc.h"

typedef struct
{
    // index部分
    uint16_t min_idx;   // 音频index最小值
    uint16_t max_idx;   // 音频index最大值
    uint8_t max_offset; // 音频index上下偏移值范围
    uint16_t last_idx;  // 上一个音频index
    uint16_t now_idx;   // 当前音频index

    // 音频数据部分
    float *tail_wav;   // 上一帧尾部音频
    uint16_t tail_len; // 上一帧尾部音频长度
    // 环形数组缓存音频
    float *ring_wav;
    uint16_t ring_read;
    uint16_t ring_write;

    uint16_t max_overlap; // 允许最大重叠长度
    float vol;            // 音量控制，范围0到1
} stitch_t;

typedef struct
{
    float *mix_wav; // 获取到的混音音频

    uint8_t *mix1_start_addr; // 混音1起点
    uint8_t *mix1_end_addr;   // 混音1终点
    uint32_t mix1_len;        // 混音1长度
    uint32_t mix1_now_len;    // 当前混音1长度
    bool is_mix1;             // 是否进行混音1
} mix_wav_t;

typedef struct
{
    float throttle;     // 油门
    float throttle_max; // 油门最大值
    float speed;        // 速度
    float speed_max;    // 速度最大值
    float air;          // 空气阻力
    float brake;        // 刹车
    float speed_dt;
} motuo_t;

static int16_t float_2_int16(float x);
static float int16_2_float(int16_t x);
static uint16_t find_zero_crossing(float *samples, size_t n, uint16_t search_from_start, uint16_t tolerance);
static uint16_t stitch_one_frame_realtime(float *tail_buffer, uint16_t *tail_len,
                                          float *curr_frame, uint16_t curr_len,
                                          uint16_t sr, float tolerance_ms, uint16_t max_overlap,
                                          float *output_part);
static void process_same_index(uint16_t *idx, uint16_t max_idx, uint16_t min_idx, uint8_t max_offset);

static uint16_t write_float_buff();
static void get_float_wav(uint16_t index, float *ouput_dat);
static uint16_t get_float_mix1_wav(float *ouput_dat);

void motor_voice_init();
void my_freertos_task(void *arg);
void my_freertos_init();
void start_shache();
void my_audio_callback(int16_t *ptr);

#endif
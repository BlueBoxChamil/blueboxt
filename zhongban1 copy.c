#include "zhongban1.h"

// sin的方式来运行代码，主要为了不对等映射和处理两段相同index的音频的方法
// 需要优化，在实时运行中不能遍历每个index来判断是否重复的，重复的话+1，越界回到offset最小值，不用计算很方便，声音结果也差不多
// 真正的实时处理音频！！！不是拿到所有音频再处理！！还需要优化和简化代码，现在写的比较混乱，主要是调用那一块,差不多了
#if 0
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#define DR_WAV_IMPLEMENTATION
#include "dr_wav.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define DR_WAV_IMPLEMENTATION
#include "dr_wav.h"

#define SR_EXPECTED 44100
#define NUM_SEGMENTS 1000
#define FRAME_MS 50

// ---------------- 工具函数 ----------------

// 找零交点
size_t find_zero_crossing(float *samples, size_t n, int search_from_start, size_t tolerance)
{
    if (search_from_start)
    {
        size_t end = tolerance < n ? tolerance : n;
        for (size_t i = 1; i < end; i++)
        {
            if ((samples[i - 1] <= 0 && samples[i] > 0) || (samples[i - 1] >= 0 && samples[i] < 0))
            {
                return i;
            }
        }
        return 0;
    }
    else
    {
        size_t start = n > tolerance ? n - tolerance : 1;
        for (size_t i = n - 1; i > start; i--)
        {
            if ((samples[i - 1] <= 0 && samples[i] > 0) || (samples[i - 1] >= 0 && samples[i] < 0))
            {
                return i;
            }
        }
        return n;
    }
}

// 交叉淡入淡出拼接
float *stitch_frames(float **frames, int num_frames, int frame_len, int sr, float tolerance_ms, size_t *out_len)
{
    size_t tolerance = (size_t)(sr * tolerance_ms / 1000.0);
    // 最坏情况：直接拼接所有
    size_t cap = num_frames * frame_len * 2;
    float *output = (float *)malloc(cap * sizeof(float));
    size_t out_pos = 0;

    memcpy(output, frames[0], frame_len * sizeof(float));
    out_pos += frame_len;

    for (int i = 1; i < num_frames; i++)
    {
        float *prev = output;
        float *curr = frames[i];

        size_t idx_prev = find_zero_crossing(output, out_pos, 0, tolerance);
        size_t idx_curr = find_zero_crossing(curr, frame_len, 1, tolerance);

        if (idx_prev < out_pos && idx_curr < frame_len)
        {
            size_t overlap_len = (out_pos - idx_prev < frame_len - idx_curr) ? (out_pos - idx_prev) : (frame_len - idx_curr);
            if (overlap_len > 64)
                overlap_len = 64;
            if (overlap_len > 0)
            {
                for (size_t j = 0; j < overlap_len; j++)
                {
                    float fade_out = 1.0f - (float)j / (overlap_len - 1);
                    float fade_in = (float)j / (overlap_len - 1);
                    output[idx_prev + j] = output[idx_prev + j] * fade_out + curr[idx_curr + j] * fade_in;
                }
                memcpy(output + idx_prev + overlap_len, curr + idx_curr + overlap_len,
                       (frame_len - idx_curr - overlap_len) * sizeof(float));
                out_pos = idx_prev + overlap_len + (frame_len - idx_curr - overlap_len);
            }
            else
            {
                memcpy(output + out_pos, curr + idx_curr, (frame_len - idx_curr) * sizeof(float));
                out_pos += (frame_len - idx_curr);
            }
        }
        else
        {
            memcpy(output + out_pos, curr, frame_len * sizeof(float));
            out_pos += frame_len;
        }
    }

    *out_len = out_pos;
    return output;
}

// 实时拼接：返回 output_part 长度，同时输出新的尾部长度
int stitch_one_frame_realtime(float *tail_buffer, int tail_len,
                              float *curr_frame, int frame_len,
                              float *output_part, float *new_tail,
                              int sr, float tolerance_ms, int max_overlap,
                              int *new_tail_len)
{
    int tolerance = (int)(sr * tolerance_ms / 1000.0f);
    int idx_prev = find_zero_crossing(tail_buffer, tail_len, 0, tolerance);
    int idx_curr = find_zero_crossing(curr_frame, frame_len, 1, tolerance);

    // 计算 overlap_len（重叠长度）
    int overlap_len = 0;
    if (idx_prev > 0 && idx_prev < tail_len && idx_curr > 0 && idx_curr < frame_len)
    {
        // 先用尾部尾端长度
        overlap_len = tail_len - idx_prev;
        // 限制不能超过当前帧剩余长度
        int curr_remain = frame_len - idx_curr;
        if (overlap_len > curr_remain)
            overlap_len = curr_remain;
        if (overlap_len > max_overlap)
            overlap_len = max_overlap;
    }

    int out_len = 0;

    if (overlap_len > 0)
    {
        // 前半部分
        for (int i = 0; i < idx_prev; i++)
            output_part[out_len++] = tail_buffer[i]; // 前半段不重叠部分
        // 淡入淡出
        for (int i = 1; i < overlap_len; i++)
        {
            float fade_out = 1.0f - (float)i / (overlap_len - 1);
            float fade_in = (float)i / (overlap_len - 1);
            output_part[out_len++] = tail_buffer[idx_prev + i] * fade_out + curr_frame[idx_curr + i] * fade_in;
        }
        // 更新尾部缓冲区
        int tail_new_len = frame_len - (idx_curr + overlap_len);
        for (int i = 0; i < tail_new_len; i++)
            new_tail[i] = curr_frame[idx_curr + overlap_len + i];
        *new_tail_len = tail_new_len;
    }
    else
    {
        memcpy(output_part, tail_buffer, tail_len * sizeof(float));
        memcpy(new_tail, curr_frame, frame_len * sizeof(float));
        *new_tail_len = frame_len;
        out_len = tail_len;
    }

    return out_len;
}

// ---------------- 主流程 ----------------
int main_zhongban1()
{
    srand((unsigned int)time(NULL));

    drwav wav;
    if (!drwav_init_file(&wav, "./1/zuo_441.wav", NULL))
    {
        printf("无法打开 total.wav\n");
        return -1;
    }
    if (wav.channels != 1 || wav.sampleRate != SR_EXPECTED)
    {
        printf("需要单声道、%dHz，但实际是 %dHz, %d 通道\n", SR_EXPECTED, wav.sampleRate, wav.channels);
        drwav_uninit(&wav);
        return -1;
    }

    size_t total_samples = wav.totalPCMFrameCount;
    float *audio = (float *)malloc(total_samples * sizeof(float));
    drwav_read_pcm_frames_f32(&wav, wav.totalPCMFrameCount, audio);
    drwav_uninit(&wav);

    // 均匀切成 1000 段
    size_t seg_len = total_samples / NUM_SEGMENTS;
    int frame_len = (int)(FRAME_MS * SR_EXPECTED / 1000); // 结果为320  20ms @ 16kHz

    float **frames = (float **)malloc(NUM_SEGMENTS * sizeof(float *));
    for (int i = 0; i < NUM_SEGMENTS; i++)
    {
        frames[i] = (float *)calloc(frame_len, sizeof(float));
        memcpy(frames[i], audio + i * seg_len, frame_len * sizeof(float));
    }

    int all_count = 0;  // 重置计数
    int max_offset = 5; // ±5 帧浮动范围
    int min_idx = 0;
    int max_idx = 999;
    int range = max_idx - min_idx; // 489

    float **all_frames = (float **)malloc(NUM_SEGMENTS * 3 * sizeof(float *)); // 预分配足够空间

    int last_idx = -1; // 记录上一次的索引
    for (int i = 0; i < NUM_SEGMENTS; i++)
    {
        // 当前 sin 映射索引
        float theta = 2.0f * M_PI * i / NUM_SEGMENTS * 4.0f; // 频率调快 2x
        int idx = (int)(min_idx + fabs(sin(theta)) * range + 0.5f);
        if (idx > max_idx)
            idx = max_idx;

        // 如果与上次相同，执行二次修改
        if (idx == last_idx)
        {
            int range_start = idx - max_offset;
            int range_end = idx + max_offset;
            if (range_start < min_idx)
                range_start = min_idx;
            if (range_end > max_idx)
                range_end = max_idx;

            idx++;
            if (idx > max_idx)
                idx = range_start; // 防止越界
        }
        // printf("%d ", idx);

        // 保存帧
        // all_frames[all_count++] = frames[idx];
        all_frames[all_count++] = frames[i];

        // 更新 last_idx
        last_idx = idx;
    }

    // 分界线，往上是对音频的分割和映射处理，往下是对音频拼接的处理

    // 重新分配精确大小
    all_frames = (float **)realloc(all_frames, all_count * sizeof(float *));

    drwav out_wav;
    drwav_data_format format;
    format.container = drwav_container_riff;
    format.format = DR_WAVE_FORMAT_PCM;
    format.channels = 1;
    format.sampleRate = 16000;
    format.bitsPerSample = 16;

    if (!drwav_init_file_write(&out_wav, "./1/20250911_0952.wav", &format, NULL))
    {
        printf("无法写入 total_20250829-1741.wav\n");
        return -1;
    }

    int max_overlap = 64;
    // PCM 临时缓冲
    int16_t *pcm16_out = (int16_t *)malloc((frame_len + max_overlap) * sizeof(int16_t));

    float tail_buffer[frame_len + max_overlap];
    float new_tail[frame_len + max_overlap];
    float output_part[frame_len + max_overlap];
    int tail_len = frame_len;
    int out_len, new_tail_len;
    // 初始化尾部缓冲区

    frame_len = (int)(40 * SR_EXPECTED / 1000);
    tail_len = frame_len;
    memcpy(tail_buffer, all_frames[0], frame_len * sizeof(float));

    for (int i = 1; i < NUM_SEGMENTS; i++)
    {
        if (i < 750)
        {
            frame_len = (int)(40 * SR_EXPECTED / 1000);
        }
        else
        {
            frame_len = (int)(FRAME_MS * SR_EXPECTED / 1000);
        }
        out_len = stitch_one_frame_realtime(
            tail_buffer, tail_len,
            all_frames[i], frame_len,
            output_part, new_tail,
            SR_EXPECTED, 20.0f, max_overlap,
            &new_tail_len);

        // 写出 PCM
        for (int j = 0; j < out_len; j++)
        {
            float v = output_part[j];
            if (v > 1.0f)
                v = 1.0f;
            if (v < -1.0f)
                v = -1.0f;
            pcm16_out[j] = (int16_t)(v * 32767.0f);
        }
        drwav_write_pcm_frames(&out_wav, out_len, pcm16_out);

        // 更新尾部
        tail_len = new_tail_len;
        memcpy(tail_buffer, new_tail, tail_len * sizeof(float));
    }

    // 写入最后的尾部
    for (int j = 0; j < tail_len; j++)
    {
        float v = tail_buffer[j];
        if (v > 1.0f)
            v = 1.0f;
        if (v < -1.0f)
            v = -1.0f;
        pcm16_out[j] = (int16_t)(v * 32767.0f);
    }
    drwav_write_pcm_frames(&out_wav, tail_len, pcm16_out);

    free(pcm16_out);
    drwav_uninit(&out_wav);

    // printf("处理完成，输出 total_20250828-1535.wav, 样本数=%zu\n", out_len);

    free(audio);
    // free(pcm16);
    // free(final_audio);
    for (int i = 0; i < NUM_SEGMENTS; i++)
        free(frames[i]);
    free(frames);

    return 0;
}

#endif

// sin的方式来运行代码，主要为了不对等映射和处理两段相同index的音频的方法
// 需要优化，在实时运行中不能遍历每个index来判断是否重复的，重复的话+1，越界回到offset最小值，不用计算很方便，声音结果也差不多
// 真正的实时处理音频！！！不是拿到所有音频再处理！！还需要优化和简化代码，现在写的比较混乱，主要是调用那一块,差不多了
// 尝试使用读取bin来生成音频,并且为了节约ram实时读取bin文件
#if 0
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#define DR_WAV_IMPLEMENTATION
#include "dr_wav.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define DR_WAV_IMPLEMENTATION
#include "dr_wav.h"

#define SR_EXPECTED 16000
#define NUM_SEGMENTS 1000
#define FRAME_MS 50

// ---------------- 工具函数 ----------------

// 找零交点
size_t find_zero_crossing(float *samples, size_t n, int search_from_start, size_t tolerance)
{
    if (search_from_start)
    {
        size_t end = tolerance < n ? tolerance : n;
        for (size_t i = 1; i < end; i++)
        {
            if ((samples[i - 1] <= 0 && samples[i] > 0) || (samples[i - 1] >= 0 && samples[i] < 0))
            {
                return i;
            }
        }
        return 0;
    }
    else
    {
        size_t start = n > tolerance ? n - tolerance : 1;
        for (size_t i = n - 1; i > start; i--)
        {
            if ((samples[i - 1] <= 0 && samples[i] > 0) || (samples[i - 1] >= 0 && samples[i] < 0))
            {
                return i;
            }
        }
        return n;
    }
}

// 交叉淡入淡出拼接
float *stitch_frames(float **frames, int num_frames, int frame_len, int sr, float tolerance_ms, size_t *out_len)
{
    size_t tolerance = (size_t)(sr * tolerance_ms / 1000.0);
    // 最坏情况：直接拼接所有
    size_t cap = num_frames * frame_len * 2;
    float *output = (float *)malloc(cap * sizeof(float));
    size_t out_pos = 0;

    memcpy(output, frames[0], frame_len * sizeof(float));
    out_pos += frame_len;

    for (int i = 1; i < num_frames; i++)
    {
        float *prev = output;
        float *curr = frames[i];

        size_t idx_prev = find_zero_crossing(output, out_pos, 0, tolerance);
        size_t idx_curr = find_zero_crossing(curr, frame_len, 1, tolerance);

        if (idx_prev < out_pos && idx_curr < frame_len)
        {
            size_t overlap_len = (out_pos - idx_prev < frame_len - idx_curr) ? (out_pos - idx_prev) : (frame_len - idx_curr);
            if (overlap_len > 64)
                overlap_len = 64;
            if (overlap_len > 0)
            {
                for (size_t j = 0; j < overlap_len; j++)
                {
                    float fade_out = 1.0f - (float)j / (overlap_len - 1);
                    float fade_in = (float)j / (overlap_len - 1);
                    output[idx_prev + j] = output[idx_prev + j] * fade_out + curr[idx_curr + j] * fade_in;
                }
                memcpy(output + idx_prev + overlap_len, curr + idx_curr + overlap_len,
                       (frame_len - idx_curr - overlap_len) * sizeof(float));
                out_pos = idx_prev + overlap_len + (frame_len - idx_curr - overlap_len);
            }
            else
            {
                memcpy(output + out_pos, curr + idx_curr, (frame_len - idx_curr) * sizeof(float));
                out_pos += (frame_len - idx_curr);
            }
        }
        else
        {
            memcpy(output + out_pos, curr, frame_len * sizeof(float));
            out_pos += frame_len;
        }
    }

    *out_len = out_pos;
    return output;
}

// 实时拼接：返回 output_part 长度，同时输出新的尾部长度
int stitch_one_frame_realtime(float *tail_buffer, int tail_len,
                              float *curr_frame, int frame_len,
                              float *output_part, float *new_tail,
                              int sr, float tolerance_ms, int max_overlap,
                              int *new_tail_len)
{
    int tolerance = (int)(sr * tolerance_ms / 1000.0f);
    int idx_prev = find_zero_crossing(tail_buffer, tail_len, 0, tolerance);
    int idx_curr = find_zero_crossing(curr_frame, frame_len, 1, tolerance);

    // 计算 overlap_len（重叠长度）
    int overlap_len = 0;
    if (idx_prev > 0 && idx_prev < tail_len && idx_curr > 0 && idx_curr < frame_len)
    {
        // 先用尾部尾端长度
        overlap_len = tail_len - idx_prev;
        // 限制不能超过当前帧剩余长度
        int curr_remain = frame_len - idx_curr;
        if (overlap_len > curr_remain)
            overlap_len = curr_remain;
        if (overlap_len > max_overlap)
            overlap_len = max_overlap;
    }

    int out_len = 0;

    if (overlap_len > 0)
    {
        // 前半部分
        for (int i = 0; i < idx_prev; i++)
            output_part[out_len++] = tail_buffer[i]; // 前半段不重叠部分
        // 淡入淡出
        for (int i = 1; i < overlap_len; i++)
        {
            float fade_out = 1.0f - (float)i / (overlap_len - 1);
            float fade_in = (float)i / (overlap_len - 1);
            output_part[out_len++] = tail_buffer[idx_prev + i] * fade_out + curr_frame[idx_curr + i] * fade_in;
        }
        // 更新尾部缓冲区
        int tail_new_len = frame_len - (idx_curr + overlap_len);
        for (int i = 0; i < tail_new_len; i++)
            new_tail[i] = curr_frame[idx_curr + overlap_len + i];
        *new_tail_len = tail_new_len;
    }
    else
    {
        memcpy(output_part, tail_buffer, tail_len * sizeof(float));
        memcpy(new_tail, curr_frame, frame_len * sizeof(float));
        *new_tail_len = frame_len;
        out_len = tail_len;
    }

    return out_len;
}
FILE *fp_bin;
void get_float_wav(int index, float *ouput_dat)
{
    int fframe_len = (int)(FRAME_MS * SR_EXPECTED / 1000);
    if (index < 750)
    {
        fframe_len = (int)(40 * SR_EXPECTED / 1000);
        // 每帧的字节数
        size_t bytes_per_frame = fframe_len * sizeof(int16_t);

        // 定位到指定帧
        if (fseek(fp_bin, index * bytes_per_frame, SEEK_SET) != 0)
        {
            printf("fseek 定位失败 index=%d\n", index);
            return;
        }
    }
    else
    {
        fframe_len = (int)(FRAME_MS * SR_EXPECTED / 1000);
        // 每帧的字节数
        size_t bytes_per_frame = fframe_len * sizeof(int16_t);

        // 定位到指定帧
        if (fseek(fp_bin, 750 * (int)(40 * SR_EXPECTED / 1000) * sizeof(int16_t) + (index - 750) * bytes_per_frame, SEEK_SET) != 0)
        {
            printf("fseek 定位失败 index=%d\n", index);
            return;
        }
    }

    // 临时缓存 int16
    int16_t buf[fframe_len];
    size_t n = fread(buf, sizeof(int16_t), fframe_len, fp_bin);
    if (n != fframe_len)
    {
        printf("读取第 %d 段时数据不足，期望=%d 实际=%zu\n", index, fframe_len, n);
        return;
    }
    if (index == 0)
    {
        printf("fframe_len = %d\n", fframe_len);

        float tem_float;
        int16_t tem_16;
        for (int i = 0; i < fframe_len; i++)
        {
            // int16 -> float  (-1.0 ~ 1.0)
            tem_float = (float)buf[i] / 32768.0f;

            // float -> int16  （不用 round，直接算回去）
            int tmp = (int)(tem_float * 32768.0f);

            // 防止越界
            if (tmp > 32767)
                tmp = 32767;
            if (tmp < -32768)
                tmp = -32768;

            tem_16 = (int16_t)tmp;

            if (tem_16 != buf[i])
            {
                printf("i=%d, tem_16=%d, buf[i]=%d (0x%04x)\n",
                       i, tem_16, buf[i], (uint16_t)buf[i]);
            }

            // printf("%04x ", (uint16_t)buf[i]);
        }
        printf("\n");
    }

    // 转换为 float (-1.0 ~ 1.0)
    for (int i = 0; i < fframe_len; i++)
    {
        ouput_dat[i] = (float)buf[i] / 32768.0f;
    }

    return;
}

#if 1
// ---------------- 主流程 ----------------
int main_zhongban3()
{
    srand((unsigned int)time(NULL));

    fp_bin = fopen("zuo_16k4050_bin.bin", "rb"); // rb是二进制打开模式
    if (!fp_bin)
    {
        printf("无法写入 zuo_16k4050_bin.bin\n");
        return -1;
    }

    int frame_len = (int)(FRAME_MS * SR_EXPECTED / 1000); // 结果为320  20ms @ 16kHz

    // float **frames = (float **)malloc(NUM_SEGMENTS * sizeof(float *));
    // for (int i = 0; i < NUM_SEGMENTS; i++)
    // {
    //     frames[i] = (float *)calloc(frame_len, sizeof(float));

    //     // 先用 int16_t 读取
    //     int16_t *tmp = (int16_t *)malloc(frame_len * sizeof(int16_t));
    //     size_t n = fread(tmp, sizeof(int16_t), frame_len, fp_bin);
    //     if (n != frame_len)
    //     {
    //         printf("读取第 %d 段时数据不足，期望=%d 实际=%zu\n", i, frame_len, n);
    //         free(tmp);
    //         return -1;
    //     }

    //     // 转换成 float [-1,1]
    //     for (int j = 0; j < frame_len; j++)
    //     {
    //         frames[i][j] = tmp[j] / 32767.0f;
    //     }

    //     free(tmp);
    // }

    drwav out_wav;
    drwav_data_format format;
    format.container = drwav_container_riff;
    format.format = DR_WAVE_FORMAT_PCM;
    format.channels = 1;
    format.sampleRate = 16000;
    format.bitsPerSample = 16;

    if (!drwav_init_file_write(&out_wav, "20250910-1548-4050.wav", &format, NULL))
    {
        printf("无法写入 zuo_16k50_20250910-1327.wav\n");
        return -1;
    }

    int max_overlap = 64;
    // PCM 临时缓冲
    int16_t *pcm16_out = (int16_t *)malloc((frame_len + max_overlap) * sizeof(int16_t));

    float tail_buffer[frame_len + max_overlap];
    float new_tail[frame_len + max_overlap];
    float output_part[frame_len + max_overlap];
    int tail_len = frame_len;
    int out_len, new_tail_len;

    int all_count = 0;  // 重置计数
    int max_offset = 5; // ±5 帧浮动范围
    int min_idx = 0;
    int max_idx = 999;
    int range = max_idx - min_idx; // 489

    float tail_next_buffer[frame_len];

    // float **all_frames = (float **)malloc(NUM_SEGMENTS * 3 * sizeof(float *)); // 预分配足够空间

    // 初始化尾部缓冲区
    // memcpy(tail_buffer, all_frames[0], frame_len * sizeof(float));
    tail_len = (int)(40 * SR_EXPECTED / 1000);
     
    get_float_wav(0, tail_buffer);

    int last_idx = -1; // 记录上一次的索引
    for (int i = 1; i < NUM_SEGMENTS; i++)
    {
        // 当前 sin 映射索引
        float theta = 2.0f * M_PI * i / NUM_SEGMENTS * 4.0f; // 频率调快 2x
        int idx = (int)(min_idx + fabs(sin(theta)) * range + 0.5f);
        if (idx > max_idx)
            idx = max_idx;

        // 如果与上次相同，执行二次修改
        if (idx == last_idx)
        {
            int range_start = idx - max_offset;
            int range_end = idx + max_offset;
            if (range_start < min_idx)
                range_start = min_idx;
            if (range_end > max_idx)
                range_end = max_idx;

            idx++;
            if (idx > max_idx)
                idx = range_start; // 防止越界
        }
        idx = i;
        // printf("%d ", idx);

        // 保存帧
        // all_frames[all_count++] = frames[idx];

        get_float_wav(idx, tail_next_buffer);
        if (i < 750)
        {
            frame_len = (int)(40 * SR_EXPECTED / 1000);
        }
        else
        {
            frame_len = (int)(FRAME_MS * SR_EXPECTED / 1000);
        }

        out_len = stitch_one_frame_realtime(
            tail_buffer, tail_len,
            tail_next_buffer, frame_len,
            output_part, new_tail,
            SR_EXPECTED, 20.0f, max_overlap,
            &new_tail_len);

        // 写出 PCM
        for (int j = 0; j < out_len; j++)
        {
            float v = output_part[j];
            if (v > 1.0f)
                v = 1.0f;
            if (v < -1.0f)
                v = -1.0f;
            pcm16_out[j] = (int16_t)(v * 32767.0f);
        }
        drwav_write_pcm_frames(&out_wav, out_len, pcm16_out);

        // 更新尾部
        tail_len = new_tail_len;
        memcpy(tail_buffer, new_tail, tail_len * sizeof(float));

        // 更新 last_idx
        last_idx = idx;
    }

    // 写入最后的尾部
    for (int j = 0; j < tail_len; j++)
    {
        float v = tail_buffer[j];
        if (v > 1.0f)
            v = 1.0f;
        if (v < -1.0f)
            v = -1.0f;
        pcm16_out[j] = (int16_t)(v * 32767.0f);
    }
    drwav_write_pcm_frames(&out_wav, tail_len, pcm16_out);

    free(pcm16_out);
    drwav_uninit(&out_wav);

    // 重新分配精确大小
    // all_frames = (float **)realloc(all_frames, all_count * sizeof(float *));

    // for (int i = 1; i < NUM_SEGMENTS; i++)
    // {
    //     out_len = stitch_one_frame_realtime(
    //         tail_buffer, tail_len,
    //         all_frames[i], frame_len,
    //         output_part, new_tail,
    //         SR_EXPECTED, 10.0f, max_overlap,
    //         &new_tail_len);

    //     // 写出 PCM
    //     for (int j = 0; j < out_len; j++)
    //     {
    //         float v = output_part[j];
    //         if (v > 1.0f)
    //             v = 1.0f;
    //         if (v < -1.0f)
    //             v = -1.0f;
    //         pcm16_out[j] = (int16_t)(v * 32767.0f);
    //     }
    //     drwav_write_pcm_frames(&out_wav, out_len, pcm16_out);

    //     // 更新尾部
    //     tail_len = new_tail_len;
    //     memcpy(tail_buffer, new_tail, tail_len * sizeof(float));
    // }

    // // 写入最后的尾部
    // for (int j = 0; j < tail_len; j++)
    // {
    //     float v = tail_buffer[j];
    //     if (v > 1.0f)
    //         v = 1.0f;
    //     if (v < -1.0f)
    //         v = -1.0f;
    //     pcm16_out[j] = (int16_t)(v * 32767.0f);
    // }
    // drwav_write_pcm_frames(&out_wav, tail_len, pcm16_out);

    // free(pcm16_out);
    // drwav_uninit(&out_wav);

    // printf("处理完成，输出 total_20250828-1535.wav, 样本数=%zu\n", out_len);

    // free(audio);
    // free(pcm16);
    // free(final_audio);
    // for (int i = 0; i < NUM_SEGMENTS; i++)
    //     free(frames[i]);
    // free(frames);

    return 0;
}
#else

int main_zhongban3()
{
    srand((unsigned int)time(NULL));

    drwav wav;
    if (!drwav_init_file(&wav, "zuo.wav", NULL))
    {
        printf("无法打开 zuo.wav\n");
        return -1;
    }
    if (wav.channels != 1 || wav.sampleRate != SR_EXPECTED)
    {
        printf("需要单声道、%dHz，但实际是 %dHz, %d 通道\n", SR_EXPECTED, wav.sampleRate, wav.channels);
        drwav_uninit(&wav);
        return -1;
    }

    size_t total_samples = wav.totalPCMFrameCount;
    float *audio = (float *)malloc(total_samples * sizeof(float));
    drwav_read_pcm_frames_f32(&wav, wav.totalPCMFrameCount, audio);
    drwav_uninit(&wav);

    // === 写入 total_bin.bin (int16) ===
    FILE *fp = fopen("zuo_16k4050_bin.bin", "wb");
    if (!fp)
    {
        printf("无法写入 zuo_16k4050_bin.bin\n");
        return -1;
    }

    // 均匀切成 1000 段
    size_t seg_len = total_samples / NUM_SEGMENTS;
    int frame_len = (int)(FRAME_MS * SR_EXPECTED / 1000); // 结果为320  20ms @ 16kHz

    int16_t **frames = (int16_t **)malloc(NUM_SEGMENTS * sizeof(int16_t *));
    for (int i = 0; i < NUM_SEGMENTS; i++)
    {
        if (i < 750)
        {
            frame_len = (int)(40 * SR_EXPECTED / 1000);
        }
        else
        {
            frame_len = (int)(FRAME_MS * SR_EXPECTED / 1000);
        }
        frames[i] = (int16_t *)malloc(frame_len * sizeof(int16_t));
        // float -> int16_t
        for (int j = 0; j < frame_len; j++)
        {
            float sample = audio[i * seg_len + j];
            if (sample > 1.0f)
                sample = 1.0f;
            if (sample < -1.0f)
                sample = -1.0f;
            frames[i][j] = (int16_t)(sample * 32767.0f);
        }
        fwrite(frames[i], sizeof(int16_t), frame_len, fp);
    }
    fclose(fp);
    printf("已写入 zuo_16k4050_bin.bin，每段 %d 点，共 %d 段 (int16)\n", frame_len, NUM_SEGMENTS);

    // === 从 total_bin.bin 读回 (int16) ===
    fp = fopen("zuo_16k4050_bin.bin", "rb");
    if (!fp)
    {
        printf("无法读取 zuo_16k4050_bin.bin\n");
        return -1;
    }
    int16_t **frames_check = (int16_t **)malloc(NUM_SEGMENTS * sizeof(int16_t *));
    for (int i = 0; i < NUM_SEGMENTS; i++)
    {
        if (i < 750)
        {
            frame_len = (int)(40 * SR_EXPECTED / 1000);
        }
        else
        {
            frame_len = (int)(FRAME_MS * SR_EXPECTED / 1000);
        }
        frames_check[i] = (int16_t *)malloc(frame_len * sizeof(int16_t));
        size_t n = fread(frames_check[i], sizeof(int16_t), frame_len, fp);
        if (n != frame_len)
        {
            printf("读取第 %d 段时数据不足，期望=%d 实际=%zu\n", i, frame_len, n);
            fclose(fp);
            return -1;
        }
    }
    fclose(fp);

    // === 对比差异 ===
    size_t diff_count = 0;
    for (int i = 0; i < NUM_SEGMENTS; i++)
    {
        if (i < 750)
        {
            frame_len = (int)(40 * SR_EXPECTED / 1000);
        }
        else
        {
            frame_len = (int)(FRAME_MS * SR_EXPECTED / 1000);
        }
        for (int j = 0; j < frame_len; j++)
        {
            if (frames[i][j] != frames_check[i][j])
            {
                diff_count++;
                if (diff_count < 10)
                {
                    printf("差异[%d][%d]: 原=%d, 读回=%d\n",
                           i, j, frames[i][j], frames_check[i][j]);
                }
            }
        }
    }
    if (diff_count == 0)
        printf("验证成功，所有帧数据一致。\n");
    else
        printf("验证完成，发现差异 %zu 个采样点。\n", diff_count);

    // === 释放内存 ===
    for (int i = 0; i < NUM_SEGMENTS; i++)
    {
        free(frames[i]);
        free(frames_check[i]);
    }
    free(frames);
    free(frames_check);
    free(audio);

    return 0;
}
#endif
#endif

// sin的方式来运行代码，主要为了不对等映射和处理两段相同index的音频的方法
// 需要优化，在实时运行中不能遍历每个index来判断是否重复的，重复的话+1，越界回到offset最小值，不用计算很方便，声音结果也差不多
// 真正的实时处理音频！！！不是拿到所有音频再处理！！还需要优化和简化代码，现在写的比较混乱，主要是调用那一块,差不多了
// 尝试使用读取bin来生成音频,并且为了节约ram实时读取bin文件，继续优化代码
#if 0
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define DR_WAV_IMPLEMENTATION
#include "dr_wav.h"

#define SR_EXPECTED 16000
#define NUM_SEGMENTS 1000
#define FRAME_MS 20

// ---------------- 工具函数 ----------------

// 找零交点
uint16_t find_zero_crossing(float *samples, size_t n, uint16_t search_from_start, uint16_t tolerance)
{
    if (search_from_start)
    {
        uint16_t end = tolerance < n ? tolerance : n;
        for (uint16_t i = 1; i < end; i++)
        {
            if ((samples[i - 1] <= 0 && samples[i] > 0) || (samples[i - 1] >= 0 && samples[i] < 0))
            {
                return i;
            }
        }
        return 0;
    }
    else
    {
        uint16_t start = n > tolerance ? n - tolerance : 1;
        for (uint16_t i = n - 1; i > start; i--)
        {
            if ((samples[i - 1] <= 0 && samples[i] > 0) || (samples[i - 1] >= 0 && samples[i] < 0))
            {
                return i;
            }
        }
        return n;
    }
}

// 实时拼接：返回 output_part 长度，同时输出新的尾部长度
uint16_t stitch_one_frame_realtime(float *tail_buffer, uint16_t *tail_len,
                              float *curr_frame, uint16_t frame_len,
                              uint16_t sr, float tolerance_ms, uint16_t max_overlap,
                              float *output_part)
{
    uint16_t tolerance = (uint16_t)(sr * tolerance_ms / 1000.0f);
    uint16_t idx_prev = find_zero_crossing(tail_buffer, *tail_len, 0, tolerance);
    uint16_t idx_curr = find_zero_crossing(curr_frame, frame_len, 1, tolerance);

    // 计算 overlap_len（重叠长度）
    uint16_t overlap_len = 0;
    if (idx_prev > 0 && idx_prev < *tail_len && idx_curr > 0 && idx_curr < frame_len)
    {
        // 先用尾部尾端长度
        overlap_len = *tail_len - idx_prev;
        // 限制不能超过当前帧剩余长度
        uint16_t curr_remain = frame_len - idx_curr;
        if (overlap_len > curr_remain)
            overlap_len = curr_remain;
        if (overlap_len > max_overlap)
            overlap_len = max_overlap;
    }

    uint16_t out_len = 0;

    if (overlap_len > 0)
    {
        // 前半部分
        for (uint16_t i = 0; i < idx_prev; i++)
            output_part[out_len++] = tail_buffer[i]; // 前半段不重叠部分
        // 淡入淡出
        for (uint16_t i = 0; i < overlap_len; i++)
        {
            float fade_out = 1.0f - (float)i / (overlap_len - 1);
            float fade_in = (float)i / (overlap_len - 1);
            output_part[out_len++] = tail_buffer[idx_prev + i] * fade_out + curr_frame[idx_curr + i] * fade_in;
        }
        // 更新尾部缓冲区
        uint16_t tail_new_len = frame_len - (idx_curr + overlap_len);
        for (uint16_t i = 0; i < tail_new_len; i++)
            tail_buffer[i] = curr_frame[idx_curr + overlap_len + i];
        *tail_len = tail_new_len;
    }
    else
    {
        memcpy(output_part, tail_buffer, *tail_len * sizeof(float));
        memcpy(tail_buffer, curr_frame, frame_len * sizeof(float));
        out_len = *tail_len;
        *tail_len = frame_len;
    }

    return out_len;
}
FILE *fp_bin;

/**
 * @brief 将float类型转为int16
 *
 * @param x
 * @return int16_t
 */
int16_t float_2_int16(float x)
{
    if (x > 1.0f)
        x = 1.0f;
    if (x < -1.0f)
        x = -1.0f;
    return (int16_t)(x * 32767.0f);
}

/**
 * @brief 将int16类型转为float
 *
 * @param x
 * @return float
 */
float int16_2_float(int16_t x)
{
    return (float)x / 32768.0f;
}

void get_float_wav(uint16_t index, float *ouput_dat)
{
    uint16_t fframe_len = 320;
    // 每帧的字节数
    uint16_t bytes_per_frame = fframe_len * sizeof(int16_t);

    // 定位到指定帧
    if (fseek(fp_bin, index * bytes_per_frame, SEEK_SET) != 0)
    {
        printf("fseek 定位失败 index=%d\n", index);
        return;
    }

    // 临时缓存 int16
    int16_t buf[fframe_len];
    uint16_t n = fread(buf, sizeof(int16_t), fframe_len, fp_bin);
    if (n != fframe_len)
    {
        printf("读取第 %d 段时数据不足，期望=%d 实际=%zu\n", index, fframe_len, n);
        return;
    }

    // 转换为 float (-1.0 ~ 1.0)
    for (uint16_t i = 0; i < fframe_len; i++)
    {
        ouput_dat[i] = int16_2_float(buf[i]);
    }

    return;
}

/**
 * @brief 对音频index进行处理，保证和输入index不相同
 *
 * @param idx
 * @param max_idx
 * @param min_idx
 * @param max_offset
 */
void process_same_index(uint16_t *idx, uint16_t max_idx, uint16_t min_idx, uint8_t max_offset)
{
    uint16_t range_start = *idx - max_offset;
    uint16_t range_end = *idx + max_offset;
    if (range_start < min_idx)
        range_start = min_idx;
    if (range_end > max_idx)
        range_end = max_idx;

    (*idx)++;
    if (*idx > range_end)
        *idx = range_start; // 防止越界
}

typedef struct
{
    // index部分
    uint16_t min_idx;   // 音频index最小值
    uint16_t max_idx;   // 音频index最大值
    uint8_t max_offset; // 音频index上下偏移值范围
    uint16_t last_idx;  // 上一个音频index
    uint16_t now_idx;   // 当前音频index

    // 音频数据部分
    uint16_t frame_len; // 单帧音频长度 320
    float *input_wav;  // 输入一帧新音频

    float *tail_wav;  // 上一帧尾部音频
    uint16_t tail_len; // 上一帧尾部音频长度

    float *output_wav;         // 零点交叉淡化后的音频
    int16_t *output_int16_wav; // 零点交叉淡化后的int16_t音频
    uint16_t output_len;  // 零点交叉淡化后的音频长度
    
    uint16_t max_overlap; // 允许最大重叠长度

} stitch_t;

stitch_t s_wav;
// ---------------- 主流程 ----------------
int main_zhongban2()
{
    srand((unsigned int)time(NULL));

    fp_bin = fopen("total_bin.bin", "rb"); // rb是二进制打开模式
    if (!fp_bin)
    {
        printf("无法写入 total_bin.bin\n");
        return -1;
    }

    drwav out_wav;
    drwav_data_format format;
    format.container = drwav_container_riff;
    format.format = DR_WAVE_FORMAT_PCM;
    format.channels = 1;
    format.sampleRate = 16000;
    format.bitsPerSample = 16;

    if (!drwav_init_file_write(&out_wav, "total_20250902-1421.wav", &format, NULL))
    {
        printf("无法写入 total_20250901-0944.wav\n");
        return -1;
    }

    s_wav.min_idx = 0;
    s_wav.max_idx = 999;
    s_wav.max_offset = 5;
    s_wav.last_idx = 0;
    s_wav.now_idx = 1;
    s_wav.frame_len = (uint16_t)(16000 / 1000 * 20);
    s_wav.input_wav = (float *)malloc(s_wav.frame_len * sizeof(float));
    s_wav.tail_wav = (float *)malloc(s_wav.frame_len * sizeof(float));
    s_wav.output_wav = (float *)malloc(s_wav.frame_len * sizeof(float));
    s_wav.output_int16_wav = (int16_t *)malloc(s_wav.frame_len * sizeof(int16_t));
    s_wav.tail_len = s_wav.frame_len;
    s_wav.output_len = 0;
    s_wav.max_overlap = 64;

    uint16_t get_sin_index[NUM_SEGMENTS];
    for (uint16_t i = 0; i < NUM_SEGMENTS; i++)
    {
        // 当前 sin 映射索引
        float theta = 2.0f * M_PI * i / NUM_SEGMENTS * 4.0f; // 频率调快 2x
        s_wav.now_idx = (uint16_t)(s_wav.min_idx + fabs(sin(theta)) * (s_wav.max_idx - s_wav.min_idx) + 0.5f);
        if (s_wav.now_idx > s_wav.max_idx)
            s_wav.now_idx = s_wav.max_idx;

        // // 如果与上次相同，执行二次修改
        if (s_wav.now_idx == s_wav.last_idx)
        {
            process_same_index(&(s_wav.now_idx), s_wav.max_idx, s_wav.min_idx, s_wav.max_offset);
        }

        // 保存帧
        // get_sin_index[i] = s_wav.now_idx;
        get_sin_index[i] = i;

        // 更新 last_idx
        s_wav.last_idx = s_wav.now_idx;
    }

    // 初始化尾部缓冲区
    get_float_wav(0, s_wav.tail_wav);
    for (uint16_t i = 1; i < 2; i++)
    {
        get_float_wav(get_sin_index[i], s_wav.input_wav);

        s_wav.output_len = stitch_one_frame_realtime(
            s_wav.tail_wav, &(s_wav.tail_len),
            s_wav.input_wav, s_wav.frame_len,
            SR_EXPECTED, 10.0f, s_wav.max_overlap,
            s_wav.output_wav);

        // 写出 PCM
        for (uint16_t j = 0; j < s_wav.output_len; j++)
        {
            s_wav.output_int16_wav[j] = float_2_int16(s_wav.output_wav[j]);
        }
        drwav_write_pcm_frames(&out_wav, s_wav.output_len, s_wav.output_int16_wav);
    }
    // 写入最后的尾部
    for (uint16_t j = 0; j < s_wav.tail_len; j++)
    {
        s_wav.output_int16_wav[j] = float_2_int16(s_wav.output_wav[j]);
    }
    drwav_write_pcm_frames(&out_wav, s_wav.tail_len, s_wav.output_int16_wav);

    drwav_uninit(&out_wav);
    // printf("struct\n");

    return 0;
}

#endif

// sin的方式来运行代码，主要为了不对等映射和处理两段相同index的音频的方法
// 需要优化，在实时运行中不能遍历每个index来判断是否重复的，重复的话+1，越界回到offset最小值，不用计算很方便，声音结果也差不多
// 真正的实时处理音频！！！不是拿到所有音频再处理！！还需要优化和简化代码，现在写的比较混乱，主要是调用那一块,差不多了
// 尝试使用读取bin来生成音频,并且为了节约ram实时读取bin文件，继续优化代码
// 在单片机上运行声音存在差异，排查问题,是在交叉淡化时存在分母为0的情况，因此计算出来结果不一样
#if 0
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define DR_WAV_IMPLEMENTATION
#include "dr_wav.h"

#define SR_EXPECTED 16000
#define NUM_SEGMENTS 1000
#define FRAME_MS 20

// ---------------- 工具函数 ----------------

// 找零交点
uint16_t find_zero_crossing(float *samples, size_t n, uint16_t search_from_start, uint16_t tolerance)
{
    if (search_from_start)
    {
        uint16_t end = tolerance < n ? tolerance : n;
        for (uint16_t i = 1; i < end; i++)
        {
            if ((samples[i - 1] <= 0 && samples[i] > 0) || (samples[i - 1] >= 0 && samples[i] < 0))
            {
                return i;
            }
        }
        return 0;
    }
    else
    {
        uint16_t start = n > tolerance ? n - tolerance : 1;
        for (uint16_t i = n - 1; i > start; i--)
        {
            if ((samples[i - 1] <= 0 && samples[i] > 0) || (samples[i - 1] >= 0 && samples[i] < 0))
            {
                return i;
            }
        }
        return n;
    }
}
int stitch_count = 0;
// 实时拼接：返回 output_part 长度，同时输出新的尾部长度
uint16_t stitch_one_frame_realtime(float *tail_buffer, uint16_t *tail_len,
                                   float *curr_frame, uint16_t frame_len,
                                   uint16_t sr, float tolerance_ms, uint16_t max_overlap,
                                   float *output_part)
{
    uint16_t tolerance = (uint16_t)(sr * tolerance_ms / 1000.0f);
    uint16_t idx_prev = find_zero_crossing(tail_buffer, *tail_len, 0, tolerance);
    uint16_t idx_curr = find_zero_crossing(curr_frame, frame_len, 1, tolerance);

    // 计算 overlap_len（重叠长度）
    uint16_t overlap_len = 0;
    if (idx_prev > 0 && idx_prev < *tail_len && idx_curr > 0 && idx_curr < frame_len)
    {
        // 先用尾部尾端长度
        overlap_len = *tail_len - idx_prev;
        // 限制不能超过当前帧剩余长度
        uint16_t curr_remain = frame_len - idx_curr;
        if (overlap_len > curr_remain)
            overlap_len = curr_remain;
        if (overlap_len > max_overlap)
            overlap_len = max_overlap;
    }

    uint16_t out_len = 0;
    stitch_count++;

    if (overlap_len > 0)
    {

        // 前半部分
        for (uint16_t i = 0; i < idx_prev; i++)
            output_part[out_len++] = tail_buffer[i]; // 前半段不重叠部分
        // 淡入淡出
        for (uint16_t i = 1; i < overlap_len; i++)
        {
            float fade_out = 1.0f - (float)i / (overlap_len - 1);
            float fade_in = (float)i / (overlap_len - 1);
            output_part[out_len++] = tail_buffer[idx_prev + i] * fade_out + curr_frame[idx_curr + i] * fade_in;

            // if (stitch_count == 6)
            // {
            //     // printf("%+8.4f, %+8.4f \n", output_part[248], output_part[249]);
            //     printf("fade_out = %f,fade_in = %f\n", fade_out, fade_in);
            //     printf("tail_buffer[idx_prev + i]  = %f,curr_frame[idx_curr + i] = %f\n", tail_buffer[idx_prev + i], curr_frame[idx_curr + i]);
            // }
        }
        // 更新尾部缓冲区
        uint16_t tail_new_len = frame_len - (idx_curr + overlap_len);
        for (uint16_t i = 0; i < tail_new_len; i++)
            tail_buffer[i] = curr_frame[idx_curr + overlap_len + i];
        *tail_len = tail_new_len;
    }
    else
    {
        memcpy(output_part, tail_buffer, *tail_len * sizeof(float));
        memcpy(tail_buffer, curr_frame, frame_len * sizeof(float));
        out_len = *tail_len;
        *tail_len = frame_len;
    }

    // if (stitch_count == 6)
    // {
    //     printf("idx_prev = %d,overlap_len = %d,out_len = %d\n", idx_prev, overlap_len, out_len);
    //     printf("stitch_count = %d\n", stitch_count);
    //     for (int i = 0; i < out_len; i++)
    //     {
    //         printf("%+8.4f ", output_part[i]);
    //         // printf("%x ", s_wav.output_int16_wav[j]);
    //         if ((i + 1) % 16 == 0)
    //         {
    //             printf("\n");
    //         }
    //     }
    //     printf("\n**********************************************\n");
    // }

    return out_len;
}
FILE *fp_bin;

/**
 * @brief 将float类型转为int16
 *
 * @param x
 * @return int16_t
 */
int16_t float_2_int16(float x)
{
    if (x > 1.0f)
        x = 1.0f;
    if (x < -1.0f)
        x = -1.0f;
    return (int16_t)(x * 32767.0f);
}

/**
 * @brief 将int16类型转为float
 *
 * @param x
 * @return float
 */
float int16_2_float(int16_t x)
{
    return (float)x / 32768.0f;
}

void get_float_wav(uint16_t index, float *ouput_dat)
{
    uint16_t fframe_len = 320;
    // 每帧的字节数
    uint16_t bytes_per_frame = fframe_len * sizeof(int16_t);

    // 定位到指定帧
    if (fseek(fp_bin, index * bytes_per_frame, SEEK_SET) != 0)
    {
        printf("fseek 定位失败 index=%d\n", index);
        return;
    }

    // 临时缓存 int16
    int16_t buf[fframe_len];
    uint16_t n = fread(buf, sizeof(int16_t), fframe_len, fp_bin);
    if (n != fframe_len)
    {
        printf("读取第 %d 段时数据不足，期望=%d 实际=%zu\n", index, fframe_len, n);
        return;
    }

    // 转换为 float (-1.0 ~ 1.0)
    for (uint16_t i = 0; i < fframe_len; i++)
    {
        ouput_dat[i] = int16_2_float(buf[i]);
    }

    return;
}

/**
 * @brief 对音频index进行处理，保证和输入index不相同
 *
 * @param idx
 * @param max_idx
 * @param min_idx
 * @param max_offset
 */
void process_same_index(uint16_t *idx, uint16_t max_idx, uint16_t min_idx, uint8_t max_offset)
{
    uint16_t range_start = *idx - max_offset;
    uint16_t range_end = *idx + max_offset;
    if (range_start < min_idx)
        range_start = min_idx;
    if (range_end > max_idx)
        range_end = max_idx;

    (*idx)++;
    if (*idx > range_end)
        *idx = range_start; // 防止越界
}

typedef struct
{
    // index部分
    uint16_t min_idx;   // 音频index最小值
    uint16_t max_idx;   // 音频index最大值
    uint8_t max_offset; // 音频index上下偏移值范围
    uint16_t last_idx;  // 上一个音频index
    uint16_t now_idx;   // 当前音频index

    // 音频数据部分
    uint16_t frame_len; // 单帧音频长度 320
    float *input_wav;   // 输入一帧新音频

    float *tail_wav;   // 上一帧尾部音频
    uint16_t tail_len; // 上一帧尾部音频长度

    float *output_wav;         // 零点交叉淡化后的音频
    int16_t *output_int16_wav; // 零点交叉淡化后的int16_t音频
    uint16_t output_len;       // 零点交叉淡化后的音频长度

    uint16_t max_overlap; // 允许最大重叠长度

} stitch_t;

stitch_t s_wav;
// ---------------- 主流程 ----------------
int main_zhongban2()
{
    srand((unsigned int)time(NULL));

    fp_bin = fopen("total_bin.bin", "rb"); // rb是二进制打开模式
    if (!fp_bin)
    {
        printf("无法写入 total_bin.bin\n");
        return -1;
    }

    drwav out_wav;
    drwav_data_format format;
    format.container = drwav_container_riff;
    format.format = DR_WAVE_FORMAT_PCM;
    format.channels = 1;
    format.sampleRate = 16000;
    format.bitsPerSample = 16;

    if (!drwav_init_file_write(&out_wav, "total_20250902-1421.wav", &format, NULL))
    {
        printf("无法写入 total_20250901-0944.wav\n");
        return -1;
    }

    s_wav.min_idx = 0;
    s_wav.max_idx = 999;
    s_wav.max_offset = 5;
    s_wav.last_idx = 0;
    s_wav.now_idx = 1;
    s_wav.frame_len = (uint16_t)(16000 / 1000 * 20);
    s_wav.input_wav = (float *)malloc(s_wav.frame_len * sizeof(float));
    s_wav.tail_wav = (float *)malloc(s_wav.frame_len * sizeof(float));
    s_wav.output_wav = (float *)malloc(s_wav.frame_len * sizeof(float));
    s_wav.output_int16_wav = (int16_t *)malloc(s_wav.frame_len * sizeof(int16_t));
    s_wav.tail_len = s_wav.frame_len;
    s_wav.output_len = 0;
    s_wav.max_overlap = 64;

    get_float_wav(0, s_wav.tail_wav);

    // printf("get:\n");
    // for (uint16_t i = 0; i < 320; i++)
    // {
    //     printf("%+8.4f ", s_wav.tail_wav[i]);
    // }
    // printf("\n");

    uint16_t get_sin_index[NUM_SEGMENTS];
    for (uint16_t i = 0; i < NUM_SEGMENTS; i++)
    {
        // 当前 sin 映射索引
        float theta = 2.0f * M_PI * i / NUM_SEGMENTS * 4.0f; // 频率调快 2x
        s_wav.now_idx = (uint16_t)(s_wav.min_idx + fabs(sin(theta)) * (s_wav.max_idx - s_wav.min_idx) + 0.5f);
        if (s_wav.now_idx > s_wav.max_idx)
            s_wav.now_idx = s_wav.max_idx;

        // // 如果与上次相同，执行二次修改
        if (s_wav.now_idx == s_wav.last_idx)
        {
            process_same_index(&(s_wav.now_idx), s_wav.max_idx, s_wav.min_idx, s_wav.max_offset);
        }

        // 保存帧
        // get_sin_index[i] = s_wav.now_idx;
        get_sin_index[i] = i;

        // 更新 last_idx
        s_wav.last_idx = s_wav.now_idx;
    }
    int tttt = 0;
    // 初始化尾部缓冲区
    get_float_wav(0, s_wav.tail_wav);
    for (uint16_t i = 1; i < 1000; i++)
    {
        get_float_wav(get_sin_index[i], s_wav.input_wav);

        s_wav.output_len = stitch_one_frame_realtime(
            s_wav.tail_wav, &(s_wav.tail_len),
            s_wav.input_wav, s_wav.frame_len,
            SR_EXPECTED, 10.0f, s_wav.max_overlap,
            s_wav.output_wav);

        // printf("now = %d\n", i);

        // 写出 PCM
        // printf("get len:%d\n", s_wav.output_len);
        // if (i == 6)
        // {
            for (uint16_t j = 0; j < s_wav.output_len; j++)
            {
                s_wav.output_int16_wav[j] = float_2_int16(s_wav.output_wav[j]);
                // printf("%+8.4f ", s_wav.output_wav[j]);
                // // printf("%x ", s_wav.output_int16_wav[j]);
                // if ((j + 1) % 16 == 0)
                // {
                //     printf("\n");
                // }
                // // tttt++;
            }
        //     // printf("\n shi jin zhi\n");
        //     // for (uint16_t j = 0; j < s_wav.output_len; j++)
        //     // {
        //     //     printf("%d ", s_wav.output_int16_wav[j]);
        //     //     if ((j + 1) % 16 == 0)
        //     //     {
        //     //         printf("\n");
        //     //     }
        //     // }
        // }
        // printf("\n");
        drwav_write_pcm_frames(&out_wav, s_wav.output_len, s_wav.output_int16_wav);
    }
    // 写入最后的尾部
    for (uint16_t j = 0; j < s_wav.tail_len; j++)
    {
        s_wav.output_int16_wav[j] = float_2_int16(s_wav.output_wav[j]);
    }
    drwav_write_pcm_frames(&out_wav, s_wav.tail_len, s_wav.output_int16_wav);

    drwav_uninit(&out_wav);
    // printf("struct\n");

    return 0;
}

#endif

// 将wav音频转为bin
#if 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#define DR_WAV_IMPLEMENTATION
#include "dr_wav.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define DR_WAV_IMPLEMENTATION
#include "dr_wav.h"

#define SR_EXPECTED 16000
#define NUM_SEGMENTS 1000
#define FRAME_MS 50
int main_zhongban3()
{
    srand((unsigned int)time(NULL));

    drwav wav;
    if (!drwav_init_file(&wav, "zuo.wav", NULL))
    {
        printf("无法打开 zuo.wav\n");
        return -1;
    }
    if (wav.channels != 1 || wav.sampleRate != SR_EXPECTED)
    {
        printf("需要单声道、%dHz，但实际是 %dHz, %d 通道\n", SR_EXPECTED, wav.sampleRate, wav.channels);
        drwav_uninit(&wav);
        return -1;
    }

    size_t total_samples = wav.totalPCMFrameCount;
    int16_t *audio = (int16_t *)malloc(total_samples * sizeof(int16_t));
    drwav_read_pcm_frames_s16(&wav, wav.totalPCMFrameCount, audio);
    drwav_uninit(&wav);

    // === 写入 total_bin.bin (int16) ===
    FILE *fp = fopen("zuo_16k4050_bin.bin", "wb");
    if (!fp)
    {
        printf("无法写入 zuo_16k4050_bin.bin\n");
        return -1;
    }

    // 均匀切成 1000 段
    size_t seg_len = total_samples / NUM_SEGMENTS;
    int frame_len = (int)(FRAME_MS * SR_EXPECTED / 1000); // 结果为320  20ms @ 16kHz

    int16_t **frames = (int16_t **)malloc(NUM_SEGMENTS * sizeof(int16_t *));
    for (int i = 0; i < NUM_SEGMENTS; i++)
    {
        if (i < 750)
        {
            frame_len = (int)(40 * SR_EXPECTED / 1000);
        }
        else
        {
            frame_len = (int)(FRAME_MS * SR_EXPECTED / 1000);
        }
        frames[i] = (int16_t *)malloc(frame_len * sizeof(int16_t));
        // float -> int16_t
        for (int j = 0; j < frame_len; j++)
        {
            // float sample = audio[i * seg_len + j];
            // if (sample > 1.0f)
            //     sample = 1.0f;
            // if (sample < -1.0f)
            //     sample = -1.0f;
            // frames[i][j] = (int16_t)(sample * 32767.0f);
            frames[i][j] = audio[i * seg_len + j];
        }
        fwrite(frames[i], sizeof(int16_t), frame_len, fp);
    }
    fclose(fp);
    printf("已写入 zuo_16k4050_bin.bin，每段 %d 点，共 %d 段 (int16)\n", frame_len, NUM_SEGMENTS);

    // === 从 total_bin.bin 读回 (int16) ===
    fp = fopen("zuo_16k4050_bin.bin", "rb");
    if (!fp)
    {
        printf("无法读取 zuo_16k4050_bin.bin\n");
        return -1;
    }
    int16_t **frames_check = (int16_t **)malloc(NUM_SEGMENTS * sizeof(int16_t *));
    for (int i = 0; i < NUM_SEGMENTS; i++)
    {
        if (i < 750)
        {
            frame_len = (int)(40 * SR_EXPECTED / 1000);
        }
        else
        {
            frame_len = (int)(FRAME_MS * SR_EXPECTED / 1000);
        }
        frames_check[i] = (int16_t *)malloc(frame_len * sizeof(int16_t));
        size_t n = fread(frames_check[i], sizeof(int16_t), frame_len, fp);
        if (n != frame_len)
        {
            printf("读取第 %d 段时数据不足，期望=%d 实际=%zu\n", i, frame_len, n);
            fclose(fp);
            return -1;
        }
    }
    fclose(fp);

    // === 对比差异 ===
    size_t diff_count = 0;
    for (int i = 0; i < NUM_SEGMENTS; i++)
    {
        if (i < 750)
        {
            frame_len = (int)(40 * SR_EXPECTED / 1000);
        }
        else
        {
            frame_len = (int)(FRAME_MS * SR_EXPECTED / 1000);
        }
        for (int j = 0; j < frame_len; j++)
        {
            if (frames[i][j] != frames_check[i][j])
            {
                diff_count++;
                if (diff_count < 10)
                {
                    printf("差异[%d][%d]: 原=%d, 读回=%d\n",
                           i, j, frames[i][j], frames_check[i][j]);
                }
            }
        }
    }
    if (diff_count == 0)
        printf("验证成功，所有帧数据一致。\n");
    else
        printf("验证完成，发现差异 %zu 个采样点。\n", diff_count);

    // === 释放内存 ===
    for (int i = 0; i < NUM_SEGMENTS; i++)
    {
        free(frames[i]);
        free(frames_check[i]);
    }
    free(frames);
    free(frames_check);
    free(audio);

    return 0;
}
#endif
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <chrono>
#include "time.h"

#define SHM_FILE "shm_timer"

namespace shm_timer
{
    typedef struct lidar_time_mem
    {
        int64_t seq;
        int64_t base_time; // 触发点的时间
    } lidar_time_mem;

     int64_t last_stamp = 0;

    lidar_time_mem *lidar_time_mem_data = nullptr;
    void open()
    {
        int fd = shm_open(SHM_FILE, O_RDONLY, 0666);
        if (fd < 0)
        {
            printf("shm_timer open failed!!!\n");
            return;
        }

        lidar_time_mem_data = (lidar_time_mem *)mmap(NULL, sizeof(lidar_time_mem), PROT_READ, MAP_SHARED, fd, 0);
        if (!lidar_time_mem_data)
        {
            printf("shm_timer mmap failed!!!\n");
            return;
        }
    }

    void close()
    {
        if (lidar_time_mem_data != nullptr)
        {
            munmap(lidar_time_mem_data, sizeof(lidar_time_mem));
            shm_unlink(SHM_FILE);
        }
    }
    uint64_t cur_time()
    {
        auto t = std::chrono::high_resolution_clock::now();
        uint64_t sync_time = std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
        return sync_time;
    }   
    void recv_lidar_packet(int64_t raw_stamp)
    {
        if (!lidar_time_mem_data)
        {
            open();
            return ;
        }
        if (lidar_time_mem_data->seq == 0)
        {
            // 第一个点，不等检测触发，直接更新
            lidar_time_mem_data->seq += 1;
            lidar_time_mem_data->base_time = cur_time() - raw_stamp;
            last_stamp = raw_stamp;
        }
        else
        {
            if (last_stamp > raw_stamp
                &&  1000000000 > raw_stamp)
            {
                // 触发同步, 更新触发基准时间
                lidar_time_mem_data->base_time = cur_time() - raw_stamp;
                lidar_time_mem_data->seq += 1;
                printf("base time: %ld seq: %ld last_stamp: %ld\n", lidar_time_mem_data->base_time,
                lidar_time_mem_data->seq,last_stamp);
            }
            last_stamp = raw_stamp;
        }
    }
}
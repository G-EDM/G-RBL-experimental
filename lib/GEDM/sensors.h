#pragma once

/*//############################################################################################################
//
//  ██████        ███████ ██████  ███    ███  
// ██             ██      ██   ██ ████  ████  
// ██   ███ █████ █████   ██   ██ ██ ████ ██ 
// ██    ██       ██      ██   ██ ██  ██  ██ 
//  ██████        ███████ ██████  ██      ██ 
//
// This is a beta version for testing purposes.
// Only for personal use. Commercial use or redistribution without permission is prohibited. 
// Copyright (c) Roland Lautensack    
//
//############################################################################################################*/
#ifndef GSENSE_LOCK
#define GSENSE_LOCK

#include "definitions.h"
#include "api.h"
#include "widgets/language/en_us.h"
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/timer.h>
#include <esp32-hal-gpio.h>

static const int vsense_sampler_buffer_size = 128;

enum motion_plans {
    MOTION_PLAN_ZERO       = 0,
    MOTION_PLAN_FORWARD    = 1,
    MOTION_PLAN_HOLD_SOFT  = 2,
    MOTION_PLAN_HOLD_HARD  = 3,
    MOTION_PLAN_SOFT_SHORT = 4,
    MOTION_PLAN_HARD_SHORT = 5,
    MOTION_PLAN_TIMEOUT    = 6
};

enum i2s_states {
    I2S_CTRL_IDLE = 0,
    I2S_CTRL_BUSY,
    I2S_RESTARTING,
    I2S_LOCKED,
    I2S_CTRL_NOT_AVAILABLE
};

extern DRAM_ATTR std::atomic<bool> reset_vsense_queue;
extern DRAM_ATTR std::atomic<bool> restart_i2s_flag;
extern DRAM_ATTR std::atomic<bool> scope_use_high_res; // if true the adc monitor task will add each i2s sample from the batch to the scope
extern DRAM_ATTR std::atomic<bool> motion_switch_changed;  // motion on/off state changed

extern xQueueHandle remote_control_queue;
extern xQueueHandle adc_read_trigger_queue; 

extern bool IRAM_ATTR sensors_task_running( void );
extern IRAM_ATTR int get_calculated_motion_plan( void );



extern IRAM_ATTR void acquire_i2s_lock( i2s_states state = I2S_CTRL_IDLE );
extern IRAM_ATTR void release_i2s_lock( i2s_states state = I2S_CTRL_IDLE );





class G_SENSORS {

    private:

        bool has_init;
        int  mclk;
        int  sample_rate;
        int  buffer_count;

        void create_queues( void );
        void create_tasks( void );

    public:

        G_SENSORS();
        ~G_SENSORS();

        int               buffer_length;
        int               num_bytes;
        std::atomic<bool> i2s_is_ready;

        void stop( bool block = true );
        void begin( void );
        void reset( void );
        void restart( void );

        IRAM_ATTR bool       is_ready( void );
        IRAM_ATTR void       wait_for_idle( bool aquire_lock = false );
        IRAM_ATTR void       set_i2s_state( i2s_states state );
        IRAM_ATTR bool       is_state( i2s_states state );
        IRAM_ATTR i2s_states get_i2s_state( void );
        IRAM_ATTR void       process_batch( bool high_res = false );
        IRAM_ATTR void       set_avg_default( int avg );
        IRAM_ATTR void       change_sampling_rate( int rate );
        int  get_vfd_feedback( int counts = 20 );
        void reset_sensor_global( void );

        static bool IRAM_ATTR motion_switch_read( void );
        static bool IRAM_ATTR unlock_motion_switch( void );
        static bool IRAM_ATTR limit_switch_read( void );
        
        IRAM_ATTR int auto_adjust_sampler( int sample_rate = 0);

        void  generate_setpoint_min_max_adc( void );
        int   set_sample_rate( int rate );
        int   get_sample_rate( void );
        void  set_master_clock( int clock_speed );
        void  set_buffer_count( int buff_count );
        void  set_buffer_length( int buff_length );
        void  create_sensors( void );
        void  sensor_end( void );
        void  setup( void );
        bool  change_setting( setget_param_enum param_id, int int_value = 0, float float_value = 0.0 );
        int   get_setting_int( setget_param_enum param_id );
        bool  get_setting_bool( setget_param_enum param_id);
        float get_setting_float( setget_param_enum param_id );

};

extern G_SENSORS gsense;

#endif
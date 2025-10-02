#pragma once

//############################################################################################################
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
//############################################################################################################

#ifndef GSCOPE_LOCK
#define GSCOPE_LOCK

#include <stdint.h>
#include <freertos/queue.h>
#include <atomic>

extern DRAM_ATTR std::atomic<bool> scope_batch_ready;

class GPO_SCOPE {

    private:
    
        void pre_calc_math( void );

        float pwm_duty;      // duty cycle in percent
        float pwm_frequency; // frequency in khz
        bool  has_init;


    public:

        GPO_SCOPE( void );

        bool get_has_init( void );
        void flush_frame( void );

        IRAM_ATTR void set_sample_rate( uint32_t sample_rate );
        IRAM_ATTR void set_allow_motion( uint8_t motion_signal );
        IRAM_ATTR void set_digital_state( uint32_t vfd_sample );
        IRAM_ATTR void set_total_avg( uint32_t adc_value );
        IRAM_ATTR int  add_to_scope( uint32_t adc_value, int8_t plan );

        IRAM_ATTR void draw_scope( void );
        IRAM_ATTR void draw_wave( void );
        IRAM_ATTR void reset( void );
        IRAM_ATTR bool is_blocked( void );
        IRAM_ATTR bool scope_is_running( void );
        IRAM_ATTR void draw_setpoint( void );
        IRAM_ATTR bool add_meta( void );

        void draw_header( void );
        void start( void );
        void stop( void );
        void init( int16_t width, int16_t height, int16_t posx, int16_t posy );
        void set_vdrop_treshhold( int tresh );
        void set_full_range_size( int size );
        void set_pwm_duty( float duty_percent );
        void set_pwm_frequency( float frequency );
        void update_setpoint( uint16_t smin, uint16_t smax );

};


extern GPO_SCOPE gscope;

#endif
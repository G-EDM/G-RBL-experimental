#pragma once
#ifndef ARC_GENERATOR_H
#define ARC_GENERATOR_H
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

#include "definitions.h"
#include "driver/mcpwm.h"
#include <stdio.h>
#include <map>
#include <atomic>
#include "widgets/language/en_us.h"

extern std::atomic<bool> arc_generator_pwm_off_flag;
extern std::atomic<bool> spark_generator_is_running_flag;

typedef struct pulse_data {
  int frequency            = 20000;
  int pwm_period_us        = 0;
  int pwm_period_times_ten = 0;
  int pwm_2500us_in_pulses = 0;
} pulse_data;

extern DRAM_ATTR volatile pulse_data pconf;

enum arc_gen {
    ARC_OK = 0,
    ARC_ON,
    ARC_OFF,
    ARC_CREATE
};

extern std::map<arc_gen, const char*> arc_messages;


class ARC_GENERATOR {

    private:

        bool  pwm_is_off;
        int   pwm_pin;  // PWM+ output pin
        int   pwm_frequency_intern;
        float pwm_duty_cycle_percent;
        float pwm_period;
        //bool  spark_generator_is_running; // this only marks if the spark engine is running. Even with a duty of 0 it can be running
        float duty_percent_backup;
        int   frequency_backup;

    public:

        ARC_GENERATOR(void);
        ~ARC_GENERATOR(void);

        IRAM_ATTR void set_pwm_state( bool low = true );
        IRAM_ATTR void reset_pwm_state( void );
        bool IRAM_ATTR get_pwm_is_off( void );
        void IRAM_ATTR pwm_off( bool soft = false );
        void IRAM_ATTR pwm_on( bool soft = false );

        void  create( void );
        bool  attach_events( void );
        void  setup_pwm_channel(void);
        bool  is_running(void);
        int   get_freq(void);
        float get_duty_percent(void);
        void  set_pwm_pin( int pin );
        void  change_pwm_frequency(int freq);
        void  change_pwm_duty(float duty);
        void  update_values(void);
        void  disable_spark_generator(void);
        void  enable_spark_generator(void);
        void  update_duty(float duty_percent);
        void  probe_mode_on( int _frequency, float _duty );
        void  probe_mode_off( void );


        bool  change_setting(    setget_param_enum param_id, int int_value = 0, float float_value = 0.0 );
        int   get_setting_int(   setget_param_enum param_id );
        float get_setting_float( setget_param_enum param_id );
        bool  get_setting_bool(  setget_param_enum param_id );

};


extern ARC_GENERATOR arcgen;

#endif
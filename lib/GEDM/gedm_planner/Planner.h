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

#include "GCode.h"

static const uint16_t POSITION_HISTORY_LENGTH = 256;




typedef struct flush_config {
  int  steps_wanted       = 0;
  int  steps_done_back    = 0;
  int  steps_done_forward = 0;
  bool is_retracting      = false;
  bool pwm_was_disabled   = false;
  bool full_retraction    = false;
} flush_config;


typedef struct planner_config {
  int  short_circuit_max_duration_us = DEFAULT_SHORTCIRCUIT_MAX_DURATION_US;
  int  line_to_line_confirm_counts   = DEFAULT_LINE_END_CONFIRMATIONS;
  int  early_exit_on_plan            = DEFAULT_EARLY_EXIT_ON_PLAN;
  int  max_reverse_depth             = DEFAULT_MAX_REVERSE_LINES;
  uint8_t sinker_axis_mask           = 0; //..
} planner_config;


typedef struct protection_config {
  int broken_wire_steps = 0;
} protection_config;

typedef struct retraction_config {
  int soft_retract_start = 0;
  int hard_retract_start = 0;
  int steps_done         = 0;
  int steps_total        = 0;
  int steps_case_3       = 0;
  int steps_case_4       = 0;
  int steps_case_5       = 0;
  int steps_case_0       = 0;
  int early_exit_confirmations = 0;
} retraction_config;


typedef struct {
  float delta   = 0;
  long  steps   = 0;
  long  counter = 0;
} Axis_Direct;


typedef struct {
  uint8_t direction_bits = 0;
  uint8_t step_bits      = 0;
  long    counter        = 0;
} micro_history;

typedef struct {
    int     tmp                     = 0; // used for random stuff
    bool    ignore_limit_switch     = 1;  // grbl has a background task running for limits, but the new planner is not async and while homing it needs a custom limits check
    bool    ignore_feed_limit       = 0;
    bool    enable_position_history = 0;
    bool    enable_flushing         = 0;
    bool    skip_feed               = 0; 
    bool    motion_plan_enabled     = 0;
    int     step_delay              = 0;
    int     last_motion_plan        = 0;
    long    step_event_count        = 0;
    uint8_t direction_bits          = 0;
    uint8_t step_bits               = 0;
    int     step_count              = 0;
    int     accel_rounds            = 0;
    bool    is_arc                  = 0;
    Axis_Direct line_math[N_AXIS];
} Line_Config;



typedef struct {
    int    arc_counter;
    bool   wire_has_first_contact;
    bool   is_flushing;
    int8_t motion_plan;
    int    total_retraction_steps;
    int    real_short;
} planner_state;

extern DRAM_ATTR volatile planner_config plconfig;
extern DRAM_ATTR volatile flush_config flconf;



class G_EDM_PLANNER{


    public:
      G_EDM_PLANNER( void );
      bool     IRAM_ATTR protection_logic( void );
      bool     IRAM_ATTR reset_short_circuit_protection( void );
      void     IRAM_ATTR convert_target_to_steps( float* target, int32_t* __target );
      uint16_t IRAM_ATTR push_to_position_history( int32_t* target, bool override, int override_index );
      uint8_t  IRAM_ATTR plan_history_line( float* target, plan_line_data_t* pl_data );
      uint16_t IRAM_ATTR get_current_work_index( void );
      void     IRAM_ATTR position_history_force_sync( void );
      int      IRAM_ATTR get_used_motion_plan( void );
      void     override_target_with_current( float* target );
      void     position_history_reset( void );
      void     push_break_to_position_history( void );
      void     push_current_mpos_to_position_history( void );
      int      position_history_undo( int count );
      bool     position_history_sync( Line_Config &line );
      void     set_ignore_breakpoints( bool ignore_break );
      void     IRAM_ATTR set_retraction_steps( void );
      void set_sinker_axis( int axis );
      void reset_planner_state( void );
      void configure( void );
      bool get_is_paused( void );
      void next_gcode_round( void );
      bool flush_end( void );
      void flush_reenable_pwm( void );
      bool flush_begin( void );
      int  get_current_round( void );
      void IRAM_ATTR reset_flush_retract_timer( void );


  private:

      bool IRAM_ATTR check_if_time_to_flush( void );

      bool IRAM_ATTR is_end_of_line( Line_Config &line );
      bool IRAM_ATTR process_wire( Line_Config &line );
      int32_t IRAM_ATTR get_retraction_steps( float travel_mm );

      bool IRAM_ATTR pre_process_history_line( Line_Config &line );
      void IRAM_ATTR pre_process_history_line_forward( Line_Config &line );


      void IRAM_ATTR reset_rconf( void );
      int sinker_axis;
      bool IRAM_ATTR short_circuit_estop( void );

      bool IRAM_ATTR do_flush_if_needed( void );
      int  IRAM_ATTR get_motion_plan( void );
      bool IRAM_ATTR probe_check( Line_Config &line );
      void IRAM_ATTR wire_line_end( Line_Config &line );
      bool IRAM_ATTR move_line( int32_t* target, Line_Config &line );
      bool IRAM_ATTR process_stage( int32_t* target, Line_Config &line );
      uint16_t position_history_get_previous( bool peek = false ); // used by the ringbuffer internally
      uint16_t position_history_get_next( void );     // used by the ringbuffer internally
      uint16_t position_history_work_get_previous( bool peek = false ); // used by the user
      uint16_t position_history_work_get_next( bool peek = false );     // used by the user
      bool     position_history_is_break( int32_t* target );
      bool     future_position_history_is_at_final_index( bool reverse = false );
      bool     position_history_is_at_final_index( void );
      bool     position_history_move_back( void );
      bool     position_history_move_forward( bool no_motion, Line_Config &line );
      void     history_backup_recover_target( void );
      int      get_success_direction( int last_direction );
      int      get_failure_direction( int last_direction );
      bool     process_direction( int direction, Line_Config &line );
      void     pause( bool redraw = false );
      void     resume( void );
      void     prepare( float* target );
      int32_t  position_history[ POSITION_HISTORY_LENGTH ][N_AXIS];
      uint16_t position_history_recover_index;
      uint16_t position_history_index;
      uint16_t position_history_index_current;
      uint16_t position_history_final_index; 
      bool     position_history_is_between;
      bool     planner_is_synced;
      bool     position_history_ignore_breakpoints;
      bool was_paused;
      int pause_recover_count;
      int64_t short_circuit_start_time; // after a given amount of time within a short circuit force the process to pause
      int has_reverse;
      plan_line_data_t  default_plan_data;
      plan_line_data_t* default_pl_data;
      bool is_paused;
      int    gcode_round;

};

extern G_EDM_PLANNER planner;
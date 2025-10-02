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

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "gpo_scope.h"
#include "ili9341_config.h"
#include "definitions.h"

static DRAM_ATTR TFT_eSprite canvas_b = TFT_eSprite(&tft);

GPO_SCOPE gscope;

struct gpo_scope_data {
    bool     header_drawn          = false;
    bool     positive_direction    = true;
    uint8_t  motion_signal         = 0;
    uint8_t  dstate                = 1;
    uint8_t  right_bar_width       = 5;
    uint8_t  vfd_state_old         = 10;
    uint8_t  cfd_state_old         = 10;
    uint32_t vrop_tresh            = 1500;
    uint32_t favg_size             = 0;
    uint32_t setpoint_min          = 0;
    uint32_t setpoint_max          = 0;
    uint32_t voltage_feedback      = 0;
    uint32_t scope_current_cursor  = 0;
    uint32_t scope_width           = 0;
    uint32_t scope_height          = 0;
    uint32_t scope_height_total    = 0;
    uint32_t scope_width_total     = 0;
    uint32_t scope_width_minus_one = 0;
    uint32_t scope_pos_x           = 0;
    uint32_t scope_pos_y           = 0;
    uint32_t sample_rate           = 0;
    uint32_t total_avg             = 0;
    uint32_t vfd_indicator_x       = 0;
    uint32_t vfd_indicator_y       = 0;
    uint32_t cfd_indicator_x       = 0;
    uint32_t cfd_indicator_y       = 0;
    uint32_t h_res_boost           = 0;
    uint32_t hv_res_boost          = 0;
};

typedef struct draw_cache_data {
    uint32_t color          = 0;
    int32_t  start_x        = 0;
    int32_t  start_y        = 0;
    int32_t  adc_value      = 0;
    int32_t  pixel_vertical = 0;
    int32_t  current_x      = 0;
    int8_t   plan           = 0;
} draw_cache_data;

static const int      scope_buffer_size = 320; // width of the display
static const uint16_t boost_scaler      = 1024;

DRAM_ATTR std::atomic<bool> scope_is_running_flag(false);
DRAM_ATTR std::atomic<bool> scope_batch_ready( false );

//static const uint16_t plan_colors[] = { COLORLIGHTGREEN, COLORLIGHTGREEN, COLORLIGHTGREEN, COLORLIGHTGREEN, COLORLIGHTGREEN, COLORLIGHTGREEN };
static DRAM_ATTR const uint16_t  plan_colors[]                        = { TFT_WHITE, TFT_WHITE, TFT_GREEN, TFT_PURPLE, TFT_ORANGE, COLORORANGERED };
static DRAM_ATTR int8_t          scope_values_plan[scope_buffer_size] = {0,}; // 0.3125 kilobytes.
static DRAM_ATTR uint32_t        scope_values[scope_buffer_size]      = {0,};
static DRAM_ATTR gpo_scope_data  gpo_data;
static DRAM_ATTR draw_cache_data draw_cache;



GPO_SCOPE::GPO_SCOPE() : has_init( false ) {


}

//#######################################################################
// Set ADC value and motion_plan used
//#######################################################################
IRAM_ATTR int GPO_SCOPE::add_to_scope( uint32_t adc_value, int8_t plan ){
    if( scope_batch_ready.load() ){ return 1; }
    scope_values[gpo_data.scope_current_cursor]      = adc_value;
    scope_values_plan[gpo_data.scope_current_cursor] = plan;
    if( ++gpo_data.scope_current_cursor >= gpo_data.scope_width ){
        gpo_data.scope_current_cursor = 0;
        scope_batch_ready.store( true );
        return 1;
    }
    return 0;
}

IRAM_ATTR bool GPO_SCOPE::is_blocked(){
    return scope_batch_ready.load();
}

IRAM_ATTR bool GPO_SCOPE::add_meta(){
    return ( gpo_data.scope_current_cursor >= gpo_data.scope_width_minus_one );
}

//#######################################################################
// Those are only really needed once a batch is full and ready
// for drawing
//#######################################################################
IRAM_ATTR void GPO_SCOPE::set_total_avg( uint32_t adc_value ){
    gpo_data.total_avg = adc_value;
}
IRAM_ATTR void GPO_SCOPE::set_sample_rate( uint32_t sample_rate ){
    gpo_data.sample_rate = sample_rate; // value in khz
}
IRAM_ATTR void GPO_SCOPE::set_allow_motion( uint8_t value ){
    gpo_data.motion_signal = value;
}
IRAM_ATTR void GPO_SCOPE::set_digital_state( uint32_t vfd_sample ){
    gpo_data.voltage_feedback = vfd_sample;
    if( vfd_sample < gpo_data.vrop_tresh ){
        gpo_data.dstate = 0;
    } else {
        gpo_data.dstate = 1;
    }
}

void GPO_SCOPE::pre_calc_math( void ){
    gpo_data.scope_width_minus_one = gpo_data.scope_width-1;
    gpo_data.h_res_boost  = gpo_data.scope_height * boost_scaler / VSENSE_RESOLUTION;
    gpo_data.hv_res_boost = gpo_data.scope_height * boost_scaler / VSENSE_RESOLUTION; // this was a little different in the past. To not mess all the code up i keep this for now
}

IRAM_ATTR void GPO_SCOPE::draw_wave(){

    //reset();
    draw_cache.start_x        = 0;
    draw_cache.start_y        = gpo_data.scope_height;
    draw_cache.pixel_vertical = 0;
    draw_cache.current_x      = 1;
    draw_cache.color          = plan_colors[2];

    for( int i = 0; i < gpo_data.scope_width_minus_one; ++i ){

        //###################################################
        // Calculate the pixel relative the the scope canvas
        //###################################################
        draw_cache.pixel_vertical = 1;

        if( scope_values[i] > 0 ){
            draw_cache.pixel_vertical = ( gpo_data.h_res_boost * scope_values[i] ) >> 10;
            if( draw_cache.pixel_vertical > gpo_data.scope_height ){
                draw_cache.pixel_vertical = gpo_data.scope_height;
            }
        } 

        if( !draw_cache.pixel_vertical ){
            draw_cache.pixel_vertical = 1;
        }

        //###################################################
        // Add coloration
        //###################################################
        if( scope_values_plan[i] < 0 ){ 
            scope_values_plan[i] = (scope_values_plan[i] ^ (scope_values_plan[i] >> 7)) - (scope_values_plan[i] >> 7); //plan*=-1; 
        }

        //###################################################
        // Draw the line and update the last y position
        //###################################################
        draw_cache.current_x = draw_cache.start_x+1;

        canvas_b.drawLine( 
            draw_cache.start_x, 
            draw_cache.start_y, 
            draw_cache.current_x, 
            gpo_data.scope_height-draw_cache.pixel_vertical, 
            gconf.gedm_retraction_motion ? COLORRAL6003 : plan_colors[ scope_values_plan[i] ] 
        );

        draw_cache.start_x = draw_cache.current_x;
        draw_cache.start_y = gpo_data.scope_height-draw_cache.pixel_vertical;

    }
    
    //###################################################
    // Calculate the pixel relative the the scope canvas
    //###################################################
    // cFd
    draw_cache.color = gpo_data.total_avg > gpo_data.setpoint_max ? TFT_RED : TFT_WHITE;
    draw_cache.start_x = gpo_data.scope_width-gpo_data.favg_size;
    draw_cache.start_y = gpo_data.scope_height-( ( gpo_data.hv_res_boost * gpo_data.total_avg ) >> 10 );
    canvas_b.drawFastHLine( draw_cache.start_x, draw_cache.start_y,   gpo_data.favg_size, draw_cache.color  );
    canvas_b.drawFastHLine( draw_cache.start_x, draw_cache.start_y+1, gpo_data.favg_size, draw_cache.color  );

    // vFd
    canvas_b.drawFastHLine( 0, 
        gpo_data.scope_height-( ( gpo_data.hv_res_boost * gpo_data.voltage_feedback ) >> 10 ), 15, 
        ( gpo_data.voltage_feedback < gpo_data.vrop_tresh ? TFT_RED : TFT_DARKGREY )  );

    canvas_b.drawNumber( gpo_data.voltage_feedback, 5,  2, 2 );
    canvas_b.drawNumber( gpo_data.total_avg,        48, 2, 2 );
    canvas_b.drawNumber( gpo_data.sample_rate,      91, 2, 2 );

    if( gpo_data.dstate != gpo_data.vfd_state_old ){
        gpo_data.vfd_state_old = gpo_data.dstate;
        tft.fillCircle( gpo_data.vfd_indicator_x, gpo_data.vfd_indicator_y, 2, ( gpo_data.vfd_state_old==1 ? TFT_GREEN : TFT_RED ) ); 
    }   

    if( gpo_data.motion_signal != gpo_data.cfd_state_old ){
        gpo_data.cfd_state_old = gpo_data.motion_signal;
        int color = TFT_RED;
        if( gpo_data.cfd_state_old == 1 ){
            color = TFT_GREEN;
        } else if( gpo_data.cfd_state_old == 2 ){
            color = TFT_ORANGE;
        }
        tft.fillCircle( gpo_data.cfd_indicator_x, gpo_data.cfd_indicator_y, 2, color ); 
    }

    canvas_b.pushSprite( gpo_data.scope_pos_x, gpo_data.scope_pos_y+20 );vTaskDelay(1);
    canvas_b.deleteSprite();

}


IRAM_ATTR void GPO_SCOPE::reset(){
    canvas_b.createSprite( gpo_data.scope_width, gpo_data.scope_height );
    vTaskDelay(1);
    gpo_data.scope_current_cursor = 0;
    scope_batch_ready.store( false );
}

void GPO_SCOPE::flush_frame(){
    canvas_b.deleteSprite();
    reset();
    canvas_b.fillSprite( TFT_BLACK );
    canvas_b.pushSprite( gpo_data.scope_pos_x, gpo_data.scope_pos_y+20 );vTaskDelay(1);
    canvas_b.deleteSprite();
    reset();
}



#include "widgets/char_helpers.h"


void GPO_SCOPE::draw_header(){
    int pos_start = 5;
    canvas_b.deleteSprite();
    canvas_b.createSprite( gpo_data.scope_width_total, gpo_data.scope_height_total );vTaskDelay(1);
    canvas_b.fillSprite( TFT_BLACK );
    canvas_b.pushSprite( gpo_data.scope_pos_x, gpo_data.scope_pos_y );vTaskDelay(1);
    canvas_b.deleteSprite();
    canvas_b.createSprite( gpo_data.scope_width_total, 19 );vTaskDelay(1);
    canvas_b.fillSprite( BUTTON_LIST_BG );vTaskDelay(1);
    //canvas_b.setTextColor( TFT_WHITE );
    canvas_b.setTextColor( TFT_LIGHTGREY );
    canvas_b.drawString( "vFd", pos_start, 2,  2 );vTaskDelay(1);
    pos_start += 43;
    canvas_b.drawString( "cFd", pos_start, 2, 2 );vTaskDelay(1);
    pos_start += 43;
    canvas_b.drawString( "kSps", pos_start, 2, 2 );vTaskDelay(1);

    //pos_start += 43;
    pos_start += 63;
    std::string pwm_data = "";
    pwm_data += float_to_string( pwm_frequency, 2 );
    pwm_data += "kHz @ ";
    pwm_data += float_to_string( pwm_duty, 2 );
    pwm_data += "%";
    canvas_b.setTextColor( TFT_DARKGREY );
    canvas_b.setTextFont( DEFAULT_FONT );
    int pos_x = ( gpo_data.scope_width_total - canvas_b.textWidth( pwm_data.c_str() ) - gpo_data.right_bar_width - 5 );
    canvas_b.drawString( pwm_data.c_str(), pos_x, 2, 2 );vTaskDelay(1);

    canvas_b.pushSprite( gpo_data.scope_pos_x, gpo_data.scope_pos_y );vTaskDelay(1);
    canvas_b.deleteSprite();
    canvas_b.setTextColor( TFT_WHITE );
    gpo_data.header_drawn = true;
    draw_setpoint();
    reset();
}


IRAM_ATTR void GPO_SCOPE::draw_setpoint(){
    canvas_b.deleteSprite();
    canvas_b.createSprite( gpo_data.right_bar_width, gpo_data.scope_height );vTaskDelay(1);
    canvas_b.fillSprite( COLORORANGERED );vTaskDelay(1);
    int32_t min = ( gpo_data.scope_height - ( gpo_data.scope_height * boost_scaler / VSENSE_RESOLUTION * gpo_data.setpoint_min ) / boost_scaler );
    int32_t max = ( gpo_data.scope_height - ( gpo_data.scope_height * boost_scaler / VSENSE_RESOLUTION * gpo_data.setpoint_max ) / boost_scaler );
    canvas_b.fillRect( 0,  max, gpo_data.right_bar_width, min-max, TFT_GREEN );vTaskDelay(1);
    canvas_b.pushSprite( gpo_data.scope_pos_x+gpo_data.scope_width, gpo_data.scope_pos_y+20 );vTaskDelay(1);
    canvas_b.deleteSprite();
}


IRAM_ATTR void GPO_SCOPE::draw_scope(){
    if( !scope_is_running_flag.load() || !scope_batch_ready.load() ){ return; }

    #ifdef ENABLE_SCOPE_LOCK
    acquire_lock_for( ATOMIC_LOCK_GSCOPE );
    #endif

    draw_wave();
    reset(); 

    #ifdef ENABLE_SCOPE_LOCK
    release_lock_for( ATOMIC_LOCK_GSCOPE );
    #endif
}
void GPO_SCOPE::update_setpoint( uint16_t smin, uint16_t smax ){

    #ifdef ENABLE_SCOPE_LOCK
    acquire_lock_for( ATOMIC_LOCK_GSCOPE );
    #endif

    gpo_data.setpoint_min    = smin;
    gpo_data.setpoint_max    = smax;
    //if( scope_is_running() ){ draw_setpoint(); }

    #ifdef ENABLE_SCOPE_LOCK
    release_lock_for( ATOMIC_LOCK_GSCOPE );
    #endif

}
void GPO_SCOPE::set_vdrop_treshhold( int tresh ){

    #ifdef ENABLE_SCOPE_LOCK
    acquire_lock_for( ATOMIC_LOCK_GSCOPE );
    #endif

    gpo_data.vrop_tresh = tresh;

    #ifdef ENABLE_SCOPE_LOCK
    release_lock_for( ATOMIC_LOCK_GSCOPE );
    #endif

}
void GPO_SCOPE::set_full_range_size( int size ){

    #ifdef ENABLE_SCOPE_LOCK
    acquire_lock_for( ATOMIC_LOCK_GSCOPE );
    #endif

    gpo_data.favg_size = size;

    #ifdef ENABLE_SCOPE_LOCK
    release_lock_for( ATOMIC_LOCK_GSCOPE );
    #endif

}
IRAM_ATTR bool GPO_SCOPE::scope_is_running(){
    return scope_is_running_flag.load();
}
void GPO_SCOPE::start(){

    #ifdef ENABLE_SCOPE_LOCK
    acquire_lock_for( ATOMIC_LOCK_GSCOPE );
    #endif

    gpo_data.vfd_state_old = 10;
    gpo_data.cfd_state_old = 10;
    gpo_data.vfd_indicator_x = gpo_data.scope_pos_x+36;
    gpo_data.vfd_indicator_y = gpo_data.scope_pos_y+10;
    gpo_data.cfd_indicator_x = gpo_data.scope_pos_x+79;
    gpo_data.cfd_indicator_y = gpo_data.scope_pos_y+10;
    if( !gpo_data.header_drawn ){
         gscope.draw_header();
    }
    scope_is_running_flag.store( true );
    reset();

    #ifdef ENABLE_SCOPE_LOCK
    release_lock_for( ATOMIC_LOCK_GSCOPE );
    #endif

}

bool GPO_SCOPE::get_has_init(){
    return has_init;
}

void GPO_SCOPE::stop(){

    #ifdef ENABLE_SCOPE_LOCK
    acquire_lock_for( ATOMIC_LOCK_GSCOPE );
    #endif

    has_init = false;

    scope_is_running_flag.store( false );
    scope_batch_ready.store( true ); // prevent the sensor class from writing
    gpo_data.header_drawn = false;
    canvas_b.deleteSprite();
    
    #ifdef ENABLE_SCOPE_LOCK
    release_lock_for( ATOMIC_LOCK_GSCOPE );
    #endif
    
}
void GPO_SCOPE::init( int16_t width, int16_t height, int16_t posx, int16_t posy ){
    stop();

    #ifdef ENABLE_SCOPE_LOCK
    acquire_lock_for( ATOMIC_LOCK_GSCOPE );
    #endif

    gpo_data.header_drawn       = false;
    gpo_data.scope_width        = width-gpo_data.right_bar_width;
    gpo_data.scope_width_total  = width;
    gpo_data.scope_height_total = height;
    gpo_data.scope_height       = height-20;
    gpo_data.scope_pos_x        = posx;
    gpo_data.scope_pos_y        = posy;
    pre_calc_math();
    canvas_b.deleteSprite();
    reset();
    canvas_b.setTextColor( TFT_WHITE );
    gscope.draw_header();
    tft.drawFastHLine( gpo_data.scope_pos_x, gpo_data.scope_pos_y+height, gpo_data.scope_width, BUTTON_LIST_BG );

    has_init = true;

    #ifdef ENABLE_SCOPE_LOCK
    release_lock_for( ATOMIC_LOCK_GSCOPE );
    #endif

}

void GPO_SCOPE::set_pwm_duty( float duty_percent ){

    #ifdef ENABLE_SCOPE_LOCK
    acquire_lock_for( ATOMIC_LOCK_GSCOPE );
    #endif

    pwm_duty = duty_percent;

    #ifdef ENABLE_SCOPE_LOCK
    release_lock_for( ATOMIC_LOCK_GSCOPE );
    #endif

}
void GPO_SCOPE::set_pwm_frequency( float frequency ){

    #ifdef ENABLE_SCOPE_LOCK
    acquire_lock_for( ATOMIC_LOCK_GSCOPE );
    #endif

    pwm_frequency = frequency;

    #ifdef ENABLE_SCOPE_LOCK
    release_lock_for( ATOMIC_LOCK_GSCOPE );
    #endif

}
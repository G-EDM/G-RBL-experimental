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
// int is_high = (GPIO_REG_READ(GPIO_IN_REG) >> (gpio_num_t)(GEDM_PWM_PIN & 0x1F)) & 1U;
/*



    GPIO_REG_READ(GPIO_IN_REG)
    Reads the current value of the GPIO input register. This register contains the logic levels of all GPIO pins.

    GEDM_PWM_PIN & 0x1F
    Masks the GEDM_PWM_PIN with 0x1F (which is 31 in decimal). This effectively extracts the lower 5 bits of GEDM_PWM_PIN, assuming GPIO pin numbers are within 0-31.

    (gpio_num_t)(GEDM_PWM_PIN & 0x1F)
    Casts the masked value to the gpio_num_t type, which is typically an enum or typedef representing GPIO pin numbers.

    GPIO_REG_READ(GPIO_IN_REG) >> (gpio_num_t)(GEDM_PWM_PIN & 0x1F)
    Shifts the register value right by the pin number, moving the target GPIO pin's bit to the least significant bit position.

    & 1U
    Masks out all bits except the least significant bit, effectively reading the state (0 or 1) of that specific GPIO pin.


*/
//############################################################################################################

#include "pwm_controller.h"
#include "sensors.h"
#include "gedm_dpm_driver.h"
#include <freertos/portmacro.h>
#include "ili9341_tft.h"
#include <esp32-hal-gpio.h>

DRAM_ATTR volatile bool lock_reference_voltage = false;

std::atomic<bool> arc_generator_pwm_off_flag(false);
std::atomic<bool> spark_generator_is_running_flag(false);

ARC_GENERATOR arcgen;

std::map<arc_gen, const char*> arc_messages = {
    { ARC_OK,     "" },
    { ARC_ON,     "Arcgenerator: ON" },
    { ARC_OFF,    "Arcgenerator: OFF" },
    { ARC_CREATE, "Creating arc generator" }
};

DRAM_ATTR volatile pulse_data pconf;
DRAM_ATTR volatile bool pconf_is_running = false;

#ifdef PWM_IS_INVERTED
    DRAM_ATTR volatile mcpwm_duty_type_t duty_mode = MCPWM_DUTY_MODE_1;
#else
    DRAM_ATTR volatile mcpwm_duty_type_t duty_mode = MCPWM_DUTY_MODE_0;
#endif

ARC_GENERATOR::ARC_GENERATOR(){
    pwm_frequency_intern   = DEFAULT_PWM_FREQUENCY;
    pwm_duty_cycle_percent = DEFAULT_PWM_DUTY;
    pwm_period             = 0.0;
    spark_generator_is_running_flag.store( false );
};

void IRAM_ATTR pwm_isr( void ){ // 
    if( 
        pconf_is_running || xQueueIsQueueEmptyFromISR( adc_read_trigger_queue ) == pdFALSE
        //|| stop_sampling.load()
    ){ return; }
    pconf_is_running = true;
    int data = 0;
    xQueueOverwriteFromISR(adc_read_trigger_queue, &data, NULL);
    pconf_is_running = false;
}

ARC_GENERATOR::~ARC_GENERATOR(){}

void ARC_GENERATOR::create(){
    debuglog( arc_messages[ARC_CREATE] );
    set_pwm_pin( GEDM_PWM_PIN );
    setup_pwm_channel();
    change_pwm_frequency( pwm_frequency_intern );
    update_duty( pwm_duty_cycle_percent );
    disable_spark_generator();
}

//####################################################################
// 
//####################################################################
bool ARC_GENERATOR::attach_events(){
    if( adc_read_trigger_queue == NULL ){ debuglog("*adc_read_trigger_queue not found!"); }
    attachInterrupt(pwm_pin, pwm_isr, RISING);
    return true;
}

//####################################################################
// Check if spark generator is running
//####################################################################
bool ARC_GENERATOR::is_running(){
    return spark_generator_is_running_flag.load();
}

//####################################################################
// Check if PWM is on or off. Doesn't check if the arc generator is 
// running. Just if the PWM is currently running.
//####################################################################
bool IRAM_ATTR ARC_GENERATOR::get_pwm_is_off(){
    return arc_generator_pwm_off_flag.load();
}

//####################################################################
// Change the PWM frequency (Hz)
//####################################################################
void ARC_GENERATOR::change_pwm_frequency( int freq ){
    if( freq < PWM_FREQUENCY_MIN ){
        freq = PWM_FREQUENCY_MIN;  
    } else if( freq > PWM_FREQUENCY_MAX ){
        freq = PWM_FREQUENCY_MAX;
    }
    pconf.frequency      = freq;
    pwm_frequency_intern = freq;
    update_values();
    if( is_system_mode_edm() && get_pwm_is_off() ){
        // gets here if value is changed in the process screen
        // not sure if this is still needed.. Needs a review.
        return;
    }
    mcpwm_set_frequency(MCPWM_UNIT_1, MCPWM_TIMER_0, freq);
}

//####################################################################
// Change the MCPWM duty etc. This is only called from within this 
// class to finally apply the new duty. Don't call it directly.
//####################################################################
void ARC_GENERATOR::change_pwm_duty( float duty ){
    if( is_system_mode_edm() && get_pwm_is_off() ){
        // gets here if value is changed in the process screen
        // not sure if this is still needed.. Needs a review.
        return;
    }
    if( !spark_generator_is_running_flag.load() && !lock_reference_voltage ){
        duty = 0;
    }
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, float( duty ) );
    reset_pwm_state();
}

//####################################################################
// 
//####################################################################
void ARC_GENERATOR::update_values(){      
    pwm_period                 = 1.0 / float( pwm_frequency_intern );
    pconf.pwm_period_us        = int( round( pwm_period * 1000.0 * 1000.0 ) );
    pconf.pwm_period_times_ten = pconf.pwm_period_us * 10;
    pconf.pwm_2500us_in_pulses = round( 2500 / pconf.pwm_period_us );
}

//####################################################################
// Fastly turn PWM off without disabling the arc generator
//####################################################################
void IRAM_ATTR ARC_GENERATOR::pwm_off( bool soft ){
    change_pwm_duty( 0.0 );
    arc_generator_pwm_off_flag.store( true );
}

//####################################################################
// Fastly turn PWM back on without affecting the arc generator state
//####################################################################
void IRAM_ATTR ARC_GENERATOR::pwm_on( bool soft ){
    arc_generator_pwm_off_flag.store( false );
    set_pwm_state( true );
    change_pwm_frequency( pwm_frequency_intern );
    change_pwm_duty( pwm_duty_cycle_percent );
}

//####################################################################
// Manually force PWM to be high or low via MCPWM
//####################################################################
IRAM_ATTR void ARC_GENERATOR::set_pwm_state( bool low ){
    mcpwm_set_signal_low( MCPWM_UNIT_1,MCPWM_TIMER_0, MCPWM_GEN_A );
}

//####################################################################
// It sure as fuck does something that resets something related to
// MCPWM I guess? It resets the duty type but I have no idea why
// that function was implemented. It is used so there must have been 
// a valid reason and I think it had to do with reenabling PWM
// after shorts and prevent an initial heavy discharge when turning
// it back on
//####################################################################
IRAM_ATTR void ARC_GENERATOR::reset_pwm_state(){
    #ifdef PWM_IS_INVERTED
        mcpwm_set_duty_type( MCPWM_UNIT_1,MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM_DUTY_MODE_1 );
    #else
        mcpwm_set_duty_type( MCPWM_UNIT_1,MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM_DUTY_MODE_0 );
    #endif
}

//####################################################################
// Create the MCPWM channel for the given GPIO
//####################################################################
void ARC_GENERATOR::setup_pwm_channel(){
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, pwm_pin);
    mcpwm_config_t pwm_config = {};
    pwm_config.frequency      = pwm_frequency_intern;
    pwm_config.cmpr_a         = 0;
    pwm_config.cmpr_b         = 0;
    pwm_config.counter_mode   = MCPWM_UP_COUNTER;
    pwm_config.duty_mode      = duty_mode;
    set_pwm_state( true );
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);
    change_pwm_duty(0.0);
}

//####################################################################
// Set the PWM output GPIO
//####################################################################
void ARC_GENERATOR::set_pwm_pin( int pin ){
    pwm_pin = pin;
}

//####################################################################
// Get the PWM frequency in Hz
//####################################################################
int ARC_GENERATOR::get_freq(){
    return pwm_frequency_intern;
}

//####################################################################
// Get the PWM duty cycle in percent (0-100%)
//####################################################################
float ARC_GENERATOR::get_duty_percent(){
    return pwm_duty_cycle_percent;
}

//####################################################################
// Update the PWM duty cycle and change the duty via MCPWM
//####################################################################
void ARC_GENERATOR::update_duty( float duty_percent ){
    if( duty_percent > PWM_DUTY_MAX ){
        duty_percent = PWM_DUTY_MAX;
    }
    change_pwm_duty( duty_percent );
    pwm_duty_cycle_percent = duty_percent;
    update_values();
}

//####################################################################
// Turn the arc generator OFF
//####################################################################
void ARC_GENERATOR::disable_spark_generator(){
    bool was_running = spark_generator_is_running_flag.load();
    pwm_off();
    spark_generator_is_running_flag.store(false);
    //spark_generator_is_running = false;
    if( ! was_running ) return;
    debuglog( arc_messages[ARC_OFF] );
}

//####################################################################
// Turn the arc generator ON
//####################################################################
void ARC_GENERATOR::enable_spark_generator(){
    bool was_running = spark_generator_is_running_flag.load();
    spark_generator_is_running_flag.store(true);
    pwm_on();
    spark_generator_is_running_flag.store(true);
    if( was_running ) return;
    debuglog( arc_messages[ARC_ON] );
}

//####################################################################
// Enable probe mode. It will adjust PWM frequency and duty
// and also change the DPM voltage and current and turn it on if
// DPM support is enabled. The current settings are saved
// and restored after probing
//####################################################################
void ARC_GENERATOR::probe_mode_on( int _frequency, float _duty ){
    acquire_i2s_lock();
    lock_reference_voltage = true;
    duty_percent_backup    = get_duty_percent();
    frequency_backup       = get_freq();
    pwm_on();
    change_pwm_frequency( _frequency );
    update_duty( _duty );
    release_i2s_lock();
    vTaskDelay(300);

}

//#####################################################################
// Restore the backup settings after probing
// restores PWM frequency, duty and set lock_reference_voltage to false
// then turns pwm_off without disabling the arc gen
//#####################################################################
void ARC_GENERATOR::probe_mode_off(){
    acquire_i2s_lock();
    lock_reference_voltage = false;
    change_pwm_frequency( frequency_backup );
    update_duty( duty_percent_backup );
    release_i2s_lock();
    pwm_off();

}

bool ARC_GENERATOR::change_setting( setget_param_enum param_id, int int_value, float float_value ){
    bool restart_i2s_req = false;
    acquire_i2s_lock();
    vTaskDelay(1);
    switch( param_id ){
        case PARAM_ID_FREQ: change_pwm_frequency( int_value ); break;
        case PARAM_ID_DUTY: update_duty( float_value );        break;
        case PARAM_ID_PWM_STATE: ( int_value == 1 ? enable_spark_generator() : disable_spark_generator() ); enforce_redraw.store( true ); break;
        //case PARAM_ID_PWM_STATE: ( int_value == 1 ? enable_spark_generator() : disable_spark_generator() ); restart_i2s_req = true; break;
        default: break; 
    }
    if( restart_i2s_req ){ 
        restart_i2s_flag.store( true ); 
    }
    release_i2s_lock();
    return true;
}

int ARC_GENERATOR::get_setting_int( setget_param_enum param_id ){
    int value = 0;
    acquire_i2s_lock();
    switch( param_id ){
        case PARAM_ID_FREQ: value = pwm_frequency_intern; break;
        default: break;
    }
    release_i2s_lock();
    return value;
}

float ARC_GENERATOR::get_setting_float( setget_param_enum param_id ){
    float value = 0;
    acquire_i2s_lock();
    switch( param_id ){
        case PARAM_ID_DUTY: value = pwm_duty_cycle_percent; break;
        default: break;
    }
    release_i2s_lock();
    return value;
}

bool ARC_GENERATOR::get_setting_bool( setget_param_enum param_id ){
    bool value = false;
    acquire_i2s_lock();
    switch( param_id ){
        case PARAM_ID_PWM_STATE:  value = spark_generator_is_running_flag.load(); break;
        default: break;
    }
    release_i2s_lock();
    return value;
}
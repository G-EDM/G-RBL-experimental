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

#include "sensors.h"
#include "ili9341_tft.h"
#include "gpo_scope.h"

//#include <soc/rtc_wdt.h>
#include <soc/sens_struct.h>
#include <esp_attr.h>
#include <driver/i2s.h>

//#include "MotionControl.h"
#include "Protocol.h"

#include <condition_variable>
#include <mutex>
#include <thread>
#include "soc/syscon_reg.h"
#include "soc/syscon_struct.h"
#include <driver/adc.h>
#include <esp_adc_cal.h>

#define MAX_BUFFER_SIZE        1024
#define I2S_MASTER_CLOCK_SPEED 2000000

static esp_adc_cal_characteristics_t adc1_chars;
std::mutex mtx;
std::condition_variable cv;

//############################################################
// The main DMA buffer used to collect the I2S samples
// If buffer length changes this buffer will get adjusted
//############################################################
static uint16_t* dma_buffer              = nullptr;
static size_t    dma_buffer_current_size = 0;
//############################################################
// Taskhandle for the adc monitor task
// This task does nothing else then reading the data collected
// with i2s and create the motion plan etc.
// it also restarts I2S if needed
// nothing else touches I2S except this task
//############################################################
TaskHandle_t adc_monitor_task_handle;
//############################################################
// This task just waits for command submitted to a queue
// it can turn pwm on and off and do some other things
// Was made for thread safety
//############################################################
TaskHandle_t remote_control_task_handle;
//############################################################
// The adc_monitor_task uses this queue to wait until 
// something is pushed to the queue to start a readout
//############################################################
xQueueHandle adc_read_trigger_queue = NULL; 
//############################################################
// The remote_control_task uses this queue to wait for 
// commands pushed to the queue
//############################################################
xQueueHandle remote_control_queue = NULL;
//############################################################
// If timer based adc readings are enabled this timer will
// trigger a readout of the i2s samples at 20khz or something
// Does nothing if not enabled
//############################################################
hw_timer_t * vsense_adc_sampler_timer = NULL; 
//############################################################
// Hardware timer used to generate the kSps benchmarks
//############################################################
hw_timer_t * benchmark_timer = NULL; 



typedef struct adc_sampling_stats {
    uint32_t sample_rate = 0; // real archived rate
} adc_sampling_stats;

typedef struct SENSOR_STATES {
  bool on_off_switch_event_detected = false;
  bool limit_switch_event_detected  = false;
  bool block_on_off_switch          = false;
} SENSOR_STATES;



typedef struct adc_readings {
  int32_t  plan                       = 0;
  int32_t  edge_type                  = 0;
  int32_t  previous_plan              = 0;
  int32_t  previous_edge              = 0;
  uint32_t zeros_in_a_row             = 0;
  uint32_t highs_in_a_row             = 0;
  uint32_t recent                     = 0;
  uint32_t sampled                    = 0;
  uint32_t avg_full_range_last        = 0;
  uint32_t avg_full_range             = 0;
  uint32_t voltage_channel_adc_recent = 0;
  uint32_t voltage_channel_shorts     = 0;
  uint32_t current_channel_high_load  = 0;
} adc_readings;



static DRAM_ATTR volatile bool reset_sensor_state = 0; // dirty flag used to signal the sensor loop to reset the sensor objects

//##################################################
// cFd = Current sense feedback
//##################################################
static DRAM_ATTR uint32_t cfd_setpoint_lower;
static DRAM_ATTR uint32_t cfd_setpoint_upper;
static DRAM_ATTR uint32_t cfd_setpoint_probing;
static DRAM_ATTR uint32_t cfd_average_rising_boost; // add a rising edge sample x times to the multibuffer to increase the speed of the rising. Faster rising results in higher sensitivity
static DRAM_ATTR uint32_t cfd_average_slow;
static DRAM_ATTR uint32_t cfd_average_fast;   
static DRAM_ATTR uint32_t cfd_edge_treshhold;   // value is in adc resolution. 12bit = 0-4095. Changes above this relative to previous are considered an edge
static DRAM_ATTR uint32_t cfd_zero_treshhold;   // used for the stop/go signals to filter jitter
static DRAM_ATTR uint32_t cfd_sample_pool_size; // number of full i2s batches need to confirm a forward motion; This will collect x batches if a forward motion is indicated and select the highest value. The other samples a ignored and not added to the multisampler neither      
static DRAM_ATTR uint32_t cfd_edge_to_edge_block_us;

//##################################################
// vFd = Voltage sense feedback
//##################################################
static DRAM_ATTR uint64_t vfd_last_short_condition;
static DRAM_ATTR bool     vfd_pin_is_low;
static DRAM_ATTR int      vfd_low_count;
static DRAM_ATTR int      vfd_probe_trigger;                // adc threshold in adc value
static DRAM_ATTR int      vfd_channel_shorts_for_pulse_off; // max vfd samples inside the I2S below the short circuit threshold allowed. If more shorts are in the batch it will toogle pwm off for a short time
static DRAM_ATTR int      vfd_channel_shorts_for_block;     // number of shorts within the i2s voltage batch that will block the next forward motion
static DRAM_ATTR int      vfd_short_recover_time_us;        // microseconds without forward motion after vfd indicated a short. Forward motion is blocked given ms after a vfd short
static DRAM_ATTR int      vfd_short_circuit_threshhold    ; // vFd below this is considered a short circuit
static DRAM_ATTR int      vfd_low_reset_count;
static DRAM_ATTR int      vfd_had_short_condition;

//##################################################
// Stop/Go signals
//##################################################
static DRAM_ATTR bool use_stop_and_go_flags;
static DRAM_ATTR int  zeros_jitter;       // the adc is not really synched and sometimes it captures the offtime. That would create a false reset of the high in a row counter.
static DRAM_ATTR int  highs_in_a_row_max; // high load signal; if iven number of high load situations occur in a row it will block forward motion and change the plane to 3 and flag it negative if it was a forward motion overwrite (1 to -3)
static DRAM_ATTR int  zeros_in_a_row_max; // low load signal; forward motion blocked until given number of low loads in a row
static DRAM_ATTR int  high_plan_at;

//##################################################
// Other stuff not yet part of the remake
//##################################################
static DRAM_ATTR int  pwm_off_count;
static DRAM_ATTR int  pulse_off_duration;
static DRAM_ATTR int  short_circuit_max_duration_us;
static DRAM_ATTR int  retract_confirmations;
static DRAM_ATTR bool block_short_low_override = false; // after manually turning pwm off for prtection i2s may deliver samples with the off time 
                                                        // that look like low load conditions. There is some logic going on that skips a few samples 
                                                        // after a manual PWM off.

//##################################################
// Stuff for kSps benchmarking
//##################################################
DRAM_ATTR bool     benchmark_sample_rate   = false;
DRAM_ATTR uint32_t benchmark_ksps          = 0;
DRAM_ATTR uint32_t benchmark_buff_len      = I2S_NUM_SAMPLES; // this is just a copy used for the benchmark task. I2S is super picky abotu other stuff accesing the same stuff it used for setup even if threadsafe access. Don't ever touch what I2S has touched from outside the task that controls I2S.
DRAM_ATTR uint32_t benchmark_adc_counter   = 0;
DRAM_ATTR uint32_t bench_timer_interval    = 2048; // needs to be a power of two; don't change without changing the bits to right shift; Power of two: (2,4,8,16,32,64,128,256,512,1024,2048...)
DRAM_ATTR size_t   benchmark_bits_to_shift = 11;   // Corresponds to the numbers of two from the comment above: (1,2,3,4,5,6,7,8,9,10,11...)

//############################################################
// Going to switch to more atomics (work in progress)
// And make everything more threadsave without reducing
// the perfromance too much. Also getting rid of all the
// volatile stuff where possible.
//############################################################
DRAM_ATTR std::atomic<bool> reset_vsense_queue( DEFAULT_RESET_SENSE_QUEUE );
DRAM_ATTR std::atomic<int>  motion_plan_atomic( 0 );                          // atomic int used to store/receive the motion plan
DRAM_ATTR std::atomic<bool> new_motion_plan( false );                         // set to true if a new plan is available
DRAM_ATTR std::atomic<bool> restart_i2s_flag( false );                        // if this flag is set to true the adc monitor task will restart i2s
DRAM_ATTR std::atomic<bool> scope_use_high_res( DEFAULT_SCOPE_USE_HIGH_RES ); // if true the adc monitor task will add each i2s sample from the batch to the scope
DRAM_ATTR std::atomic<bool> motion_switch_changed( false );                   // motion on/off state changed
DRAM_ATTR std::atomic<i2s_states> i2s_state( I2S_CTRL_IDLE ); 

std::atomic<bool> sennsors_running( false );       // set to true after the task and queues are started (doesn't care if the tasks are already running)
std::atomic<bool> adc_monitor_task_running(false); // set to true once the adc monitor task enters the inner loop


DRAM_ATTR SENSOR_STATES sensors; // used for the limits switch and motionswitch to hold the current state and flag onchange events


typedef struct i2s_shift_out_data {
    bool     success             = false;
    uint64_t read_sum_a          = 0;
    uint64_t read_sum_b          = 0;
    uint32_t count_a             = 0;
    uint32_t count_b             = 0;
    int32_t  sample              = 0;
    uint32_t selected_channel    = 0;
} i2s_shift_out_data;



static DRAM_ATTR adc_readings adc_data;



static DRAM_ATTR int32_t  highest_adc                   = 0;
static DRAM_ATTR int32_t  adc_pre_rising_edge           = -1;
static DRAM_ATTR uint32_t num_samples_in_pool           = 0;
static DRAM_ATTR uint32_t multiple_rising_edges_count   = 0;
static DRAM_ATTR uint32_t edge_to_edge_block_count      = 0;
static DRAM_ATTR uint32_t last_rising_edge_adc          = 0;
static DRAM_ATTR uint64_t falling_edge_block_start_time = 0;
static DRAM_ATTR uint64_t micros_current                = 0;

static DRAM_ATTR bool falling_edge_accepted  = false;
static DRAM_ATTR bool multiple_rising_edges  = false;
static DRAM_ATTR bool is_rising_edge         = false;
static DRAM_ATTR bool is_falling_edge        = false;
static DRAM_ATTR bool rising_edge_normalized = false;
static DRAM_ATTR bool edge_recovery_failed   = false;
static DRAM_ATTR bool rising_edge_block      = false;
static DRAM_ATTR bool falling_edge_block     = false;

// Bitpositions: (FLAGPOS1-FLAGPOS8)
// 0: falling_edge_accepted
// 1: multiple_rising_edges
// 2: is_rising_edge
// 3: is_falling_edge
// 4: rising_edge_normalized
// 5: edge_recovery_failed
// 6: rising_edge_block
// 7: falling_edge_block
// wave_flag_bitmask |=  (1 << bit_position) ( set to 1 ) // wave_flag_bitmask |= 1 << bit_position is ok too here
// wave_flag_bitmask &= ~(1 << bit_position) ( clear to 0 )
// wave_flag_bitmask &   (1 << bit_position) ( check if set )
//static DRAM_ATTR volatile uint8_t wave_flag_bitmask;



//############################################################
// Basic lock acquire; default expected state = I2S_CTRL_IDLE
//############################################################
IRAM_ATTR void acquire_i2s_lock( i2s_states state ){ 
    i2s_states expected = state;
    while( !i2s_state.compare_exchange_strong(expected, I2S_LOCKED, std::memory_order_acquire,std::memory_order_relaxed) ) {
        expected = state;
        vTaskDelay(1);
    }
}

//############################################################
// Lock release
//############################################################
IRAM_ATTR void release_i2s_lock( i2s_states state ){
    i2s_state.store( state, std::memory_order_release );
}


//###################################################
// 
//###################################################
int percentage_to_adc( float percentage ){    
    if( percentage <= 0.0 ) return 0;
    if( percentage >= 100.0 ) return VSENSE_RESOLUTION;
    return round( ( float( VSENSE_RESOLUTION ) / 100.0 ) * percentage );
}

float adc_to_percentage( int adc_value ){   
    if( adc_value <= 0 ) return 0.0;
    if( adc_value >= VSENSE_RESOLUTION ) return 100.0;
    return ( float( adc_value ) / float( VSENSE_RESOLUTION ) ) * 100.0;
}


G_SENSORS gsense;
G_SENSORS::~G_SENSORS(){}

//############################################################
// I2S controller
// Constructor; 
// Setup the pincs etc.
//############################################################
G_SENSORS::G_SENSORS() : mclk( I2S_MASTER_CLOCK_SPEED ), sample_rate( I2S_SAMPLE_RATE ), 
                         buffer_count( I2S_BUFF_COUNT ), buffer_length( I2S_NUM_SAMPLES ),
                         num_bytes( sizeof(uint16_t) * I2S_NUM_SAMPLES ), i2s_is_ready( false )
{
    cfd_setpoint_lower               = percentage_to_adc( DEFAULT_CFD_SETPOINT_MIN );
    cfd_setpoint_upper               = percentage_to_adc( DEFAULT_CFD_SETPOINT_MAX );
    cfd_setpoint_probing             = percentage_to_adc( DEFAULT_CFD_SETPOINT_PROBE );
    use_stop_and_go_flags            = DEFAULT_USE_STOP_GO_SIGNALS;
    vfd_pin_is_low                   = false;
    vfd_last_short_condition         = 0;
    vfd_low_count                    = 0;
    vfd_low_reset_count              = 0;
    vfd_had_short_condition          = 0;
    cfd_average_fast                 = DEFAULT_CFD_AVERAGE_FAST;
    cfd_average_slow                 = DEFAULT_CFD_AVERAGE_SLOW;
    cfd_average_rising_boost         = DEFAULT_CFD_AVERAGE_RISE_BOOST;
    cfd_sample_pool_size             = DEFAULT_SAMPLE_BEST_OF;
    vfd_probe_trigger                = 90;
    //multisample_counts               = 0;
    vfd_short_circuit_threshhold     = DEFAULT_VFD_SHORT_THRESH;
    vfd_channel_shorts_for_pulse_off = DEFAULT_VFD_SHORTS_TO_POFF;
    vfd_channel_shorts_for_block     = DEFAULT_VFD_SHORTS_TO_BLOCK; // number of shorts within the i2s voltage batch that will block the next forward motion
    vfd_short_recover_time_us        = 100;
    pwm_off_count                    = DEFAULT_PWM_OFF_AFTER;
    pulse_off_duration               = DEFAULT_POFF_DURATION;
    short_circuit_max_duration_us    = DEFAULT_SHORTCIRCUIT_MAX_DURATION_US;
    zeros_jitter                     = DEFAULT_CFD_SIGNAL_RESET_JITTER; // the adc is not really synched and sometimes it captures the offtime. That would create a false reset of the high in a row counter.
    highs_in_a_row_max               = DEFAULT_CFD_HIGHS_FOR_PROTECT;
    zeros_in_a_row_max               = DEFAULT_CFD_ZEROS_FOR_MOVE;
    cfd_zero_treshhold               = DEFAULT_CFD_SIGNAL_ZERO_THRESH;
    high_plan_at                     = 3;
    cfd_edge_treshhold               = DEFAULT_EDGE_THRESHOLD;  // value is in adc resolution. 12bit = 0-4095. Changes above this relative to previous are considered an edge
    retract_confirmations            = RETRACT_CONFIRMATIONS;

    pinMode( ON_OFF_SWITCH_PIN,       INPUT_PULLDOWN ); // motion switch
    pinMode( STEPPERS_LIMIT_ALL_PIN,  INPUT_PULLDOWN ); // limit switches (one pin for all)

    adc1_config_width(VSENSE_BIT_WIDTH);

    adc1_config_channel_atten( CURRENT_SENSE_CHANNEL, ADC_ATTEN_11db );
    adc1_config_channel_atten( VOLTAGE_SENSE_CHANNEL, ADC_ATTEN_11db ) ;

    adc1_get_raw( VOLTAGE_SENSE_CHANNEL ); 
    adc1_get_raw( CURRENT_SENSE_CHANNEL ); 

    esp_adc_cal_characterize( ADC_UNIT_1, ADC_ATTEN_11db, VSENSE_BIT_WIDTH, 1100, &adc1_chars );

    pinMode( CURRENT_SENSE_PIN,     INPUT );
    pinMode( VOLTAGE_SENSE_PIN, INPUT );

    //pinMode( CURRENT_SENSE_PIN,     INPUT );
    //pinMode( VOLTAGE_SENSE_PIN, INPUT );

}

//############################################################
// Create and return a DMA buffer of variable size
//############################################################
uint16_t* get_adc_batch( size_t variable_length ){
    if( dma_buffer != nullptr && dma_buffer_current_size != variable_length ){
        heap_caps_free(dma_buffer);
        dma_buffer = nullptr;
    }
    if( dma_buffer == nullptr ) {
        size_t alloc_size = variable_length;
        if( alloc_size > MAX_BUFFER_SIZE ) {
            return nullptr;
        }
        dma_buffer = (uint16_t*)heap_caps_malloc(alloc_size * sizeof(uint16_t), MALLOC_CAP_DMA);
        if( dma_buffer == nullptr ) {
            return nullptr;
        }
        dma_buffer_current_size = variable_length;
        memset(dma_buffer, 0, alloc_size * sizeof(uint16_t));
    }
    return dma_buffer;
}

//############################################################
// Some sample rates are just not working and i2s fails to
// start. Not all the time the error is catchable
// this function does also not 100% ensure success
// but it reduces possible errors by adjusting the rate
// to be within a given tolerance
//############################################################
bool sample_rate_valid( int rate, int master_clock = I2S_MASTER_CLOCK_SPEED, int bits_per_sample = 16, int channels = 2 ) {
    double bclkFreq = double( rate ) * bits_per_sample * channels;
    double ratio    = double( master_clock ) / bclkFreq;
    double ratio_rounded = std::round(ratio);
    double diff = std::abs(ratio - ratio_rounded);
    double tolerance = 0.2;
    if (diff <= tolerance) {
        return true;
    } else {
        return false;
    }
}

//############################################################
// Set the sample rate in Hz
//############################################################
int G_SENSORS::set_sample_rate( int rate ){
    while( !sample_rate_valid( rate, mclk ) ){ ++rate; }
    if( rate > 1000000 ){ while( !sample_rate_valid( rate, mclk ) ){ --rate; } }
    //sample_rate = rate;
    return rate;
}

//############################################################
// Set the buffer count; This is the number of parallel 
// buffers that are filled by I2S
//############################################################
void G_SENSORS::set_buffer_count( int buff_count ){
    buffer_count = buff_count;
}

//############################################################
// Set the buffer length; This is the number of samples
// filled into each buffer
//############################################################
void G_SENSORS::set_buffer_length( int buff_length ){
    buffer_length = buff_length;
}

//############################################################
// Set the clock speed. 2mHz default
//############################################################
void G_SENSORS::set_master_clock( int clock_speed ){
    mclk = clock_speed;
}

//############################################################
// Check if i2s is ready; atomic flag
//############################################################
IRAM_ATTR bool G_SENSORS::is_ready(){
    return i2s_is_ready.load();
}

//############################################################
// Get the current state (idle, budy etc)
//############################################################
IRAM_ATTR i2s_states G_SENSORS::get_i2s_state(){
    return i2s_state.load();
}

//############################################################
// Compare the current state against a wanted state
//############################################################
IRAM_ATTR bool G_SENSORS::is_state( i2s_states state ){
    return ( i2s_state.load() == state );
}

//############################################################
// Set the state (idle, budy etc)
//############################################################
IRAM_ATTR void G_SENSORS::set_i2s_state( i2s_states state ){
    i2s_state.store( state );
}

//############################################################
// Reset things after i2s is started
//############################################################
void G_SENSORS::reset(){
    reset_sensor_global();
    set_i2s_state( I2S_CTRL_IDLE );
    release_i2s_lock(); // same as above; redundant but let it be..
}

//############################################################
// Wait for I2S to finish what it does
//############################################################
IRAM_ATTR void G_SENSORS::wait_for_idle( bool aquire_lock ){
    while( !is_state( I2S_CTRL_IDLE ) ){
        vTaskDelay(1);
    }
}

//############################################################
// Stop I2S and uninstall driver. Use with care!
//############################################################
void G_SENSORS::stop( bool block ){
    if( is_state( I2S_CTRL_NOT_AVAILABLE ) ){ // wasn't ready
        return;
    }
    if( block ){
        acquire_i2s_lock();
    }
    i2s_stop( I2S_NUM_0 );
    i2s_adc_disable(I2S_NUM_0);
    i2s_driver_uninstall( I2S_NUM_0 );
    set_i2s_state( I2S_CTRL_NOT_AVAILABLE );
}

//############################################################
// Restarts the sampling service
//############################################################
void G_SENSORS::restart(){

    //#######################################
    // Restart only if i2s is running
    //#######################################
    if( is_state( I2S_CTRL_NOT_AVAILABLE ) ){
        return;
    }
    
    while( restart_i2s_flag.load() ){ // prevent multiple reloads and wait a little to ensure there is not another restart coming in
        restart_i2s_flag.store( false );
        vTaskDelay(200);
    }
    
    restart_i2s_flag.store( false );

    acquire_i2s_lock(); // lock is released by the reset function

    //#######################################
    // Set state to restart
    //#######################################
    set_i2s_state( I2S_RESTARTING );

    //#######################################
    // Stop I2S and uninstall driver
    //#######################################
    stop( false );

    release_i2s_lock();

    //#######################################
    // Install i2s driver and start it
    //#######################################
    begin(); // state is now set to I2S_CTRL_IDLE by the reset function called inside begin()

}


//############################################################
// Starts the sampling service
//############################################################
void G_SENSORS::begin(){

    acquire_i2s_lock();

    //############################################################
    // This little delay is needed for stable operation
    //############################################################
    static const int required_delay = 1000;

    //############################################################
    // Create a new DMA buffer if needed
    //############################################################
    if( dma_buffer != nullptr ){
        heap_caps_free( dma_buffer );
        dma_buffer = nullptr;
    }
    while( get_adc_batch( buffer_length ) == nullptr ){
        vTaskDelay(DEBUG_LOG_DELAY);
    }

    periph_module_reset(PERIPH_I2S0_MODULE);

    //############################################################
    // Some math can be done in advance
    //############################################################
    num_bytes = sizeof(uint16_t) * buffer_length;

    int _rate          = set_sample_rate( sample_rate );
    int _buffer_count  = buffer_count;
    int _buffer_length = buffer_length;

    //############################################################
    // I2S core configuration
    //############################################################
    i2s_config_t i2s_config; 
    i2s_config.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN);
    i2s_config.sample_rate          = _rate;  
    i2s_config.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT; 
    i2s_config.channel_format       = I2S_CHANNEL_FMT_ALL_LEFT; //I2S_CHANNEL_FMT_ONLY_LEFT,
    i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S | I2S_COMM_FORMAT_STAND_MSB);
    i2s_config.intr_alloc_flags     = (ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_INTRDISABLED); //ESP_INTR_FLAG_LEVEL1,
    i2s_config.dma_buf_count        = _buffer_count;
    i2s_config.dma_buf_len          = _buffer_length;
    i2s_config.use_apll             = true;
    i2s_config.tx_desc_auto_clear   = true;
    i2s_config.fixed_mclk           = mclk;

    //############################################################
    // install i2s driver
    //############################################################
    while( ESP_OK != i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL) ){ 
        vTaskDelay(5); 
    }

    //############################################################
    // Setup some things and enable second channel
    //############################################################
    i2s_set_adc_mode(ADC_UNIT_1, CURRENT_SENSE_CHANNEL);
    i2s_adc_enable(I2S_NUM_0);

    SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR1_PATT_LEN, 1, SYSCON_SARADC_SAR1_PATT_LEN_S);
    WRITE_PERI_REG(SYSCON_SARADC_SAR1_PATT_TAB1_REG, 0x7F6F0000);
    //SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR1_PATT_LEN, 7, SYSCON_SARADC_SAR1_PATT_LEN_S);

    delay(required_delay);

    i2s_start(I2S_NUM_0);

    reset();

}

//###########################################################################
// Returns true if the sensors task started
//###########################################################################
IRAM_ATTR bool sensors_task_running(){
    return adc_monitor_task_running.load();
}

//###########################################################################
// Calculates the sampling rate in kSps every x time
//###########################################################################
IRAM_ATTR void bench_on_timer() {
    static uint16_t max_rate;
    if( benchmark_sample_rate || !benchmark_adc_counter ) return;
    benchmark_sample_rate = true;
    benchmark_ksps        = ( uint32_t )( ( benchmark_adc_counter * benchmark_buff_len ) >> benchmark_bits_to_shift );
    benchmark_adc_counter = 0;
    benchmark_sample_rate = false;
}






void G_SENSORS::create_sensors(){
    debuglog("Creating sensors", DEBUG_LOG_DELAY );
    unlock_motion_switch();
    debuglog("Starting sensor queues", DEBUG_LOG_DELAY );
    create_queues();
    debuglog("Starting sensor tasks", DEBUG_LOG_DELAY );
    create_tasks();
    sennsors_running.store( true );
}

//###################################################
// Create the queues
//###################################################
void G_SENSORS::create_queues(){
    int max_rounds = 10;
    while( adc_read_trigger_queue == NULL && --max_rounds > 0 ){ adc_read_trigger_queue = xQueueCreate( 1,  sizeof(int) ); vTaskDelay(5); }
    max_rounds = 10;
    while( remote_control_queue == NULL && --max_rounds > 0 ){ remote_control_queue  = xQueueCreate( 20,  sizeof(int) ); vTaskDelay(5); }
}



//###########################################################################
// Flag limit event
//###########################################################################
void IRAM_ATTR limit_switch_on_interrupt() {
    if( 
        !gconf.gedm_disable_limits && !sensors.limit_switch_event_detected &&
        (  get_machine_state() < STATE_ALARM && !is_machine_state( STATE_HOMING ) )
    ){
        sensors.limit_switch_event_detected = true;
        int data = 2;
        if( remote_control_queue != NULL ){ xQueueSendFromISR( remote_control_queue, &data, NULL ); }
    }
}

//###########################################################################
// Check the limit switch
//###########################################################################
bool IRAM_ATTR G_SENSORS::limit_switch_read(){
    vTaskDelay( 32 / portTICK_PERIOD_MS ); 
    bool state  = false;
    if ( !is_machine_state( STATE_HOMING ) && GRBL_LIMITS::limits_get_state() ){
        if( !gconf.gedm_disable_limits ){
            if( !is_machine_state( STATE_ESTOP ) ){ 
                set_alarm( ERROR_HARDLIMIT );
                set_machine_state( STATE_ESTOP );
                vTaskDelay(DEBUG_LOG_DELAY);
            }
        }
        state = true;
    } else {
        state = false;
    }
    new_motion_plan.store( true ); // early exit on waits
    sensors.limit_switch_event_detected = false;
    return state;
}

//###########################################################################
// Flag estop event
//###########################################################################
void IRAM_ATTR motion_switch_on_interrupt(){
    if( sensors.block_on_off_switch ){ return; }
    sensors.on_off_switch_event_detected = true;
    int data = 1;
    if( remote_control_queue != NULL ){ 
        xQueueSendFromISR( remote_control_queue, &data, NULL );
    }
}

bool IRAM_ATTR G_SENSORS::unlock_motion_switch(){
    sensors.block_on_off_switch = false;
    motion_switch_read();
    motion_switch_changed.store( true );
    return true;
}

//###########################################################################
// Check the estop switch
// Need to change this
// currently toggling the switch faster can create errors
// todo
//###########################################################################
IRAM_ATTR bool G_SENSORS::motion_switch_read(){
    vTaskDelay( 32 / portTICK_PERIOD_MS ); 
    bool state  = false;
    //int is_high = ( GPIO_REG_READ( GPIO_IN1_REG ) >> ( ON_OFF_SWITCH_PIN - 32 ) ) & 0x1;;
    //if( is_high ){ //
    if( digitalRead(ON_OFF_SWITCH_PIN) ){
        if( system_block_motion ){
            enforce_redraw.store( true );
            motion_switch_changed.store( true );
        }
        system_block_motion = false;
        state = true;
    } else {
        if( is_system_mode_edm() ){
            sensors.block_on_off_switch = true; // get some control over the shutdown and block reenabling motion until all is done
        }
        if( !system_block_motion ){
            enforce_redraw.store( true );
            motion_switch_changed.store( true );
        }
        system_block_motion = true;
        state = false;
    }
    new_motion_plan.store( true );
    sensors.on_off_switch_event_detected = false;
    return state;
}

//##############################################
// Default event queue loop
//##############################################
IRAM_ATTR void remote_control_task(void *parameter){
    while( remote_control_queue == NULL ) vTaskDelay(10);
    //##############################################
    // Timer interrupt for benchmarking the kSps
    //##############################################
    benchmark_timer = timerBegin(0, 80, true);
    timerAttachInterrupt(benchmark_timer, &bench_on_timer, true);
    timerAlarmWrite(benchmark_timer, bench_timer_interval*1000, true);
    timerAlarmEnable(benchmark_timer);
    int data = 0;
    for (;;){
        //###################
        // Enter wait queue 
        //###################
        xQueueReceive( remote_control_queue, &data, portMAX_DELAY ); 
        switch ( data ){
            case 1:  G_SENSORS::motion_switch_read(); break;
            case 2:  G_SENSORS::limit_switch_read();  break;
            case 4:  arcgen.pwm_off();    break;
            case 5:  arcgen.pwm_on();     break;
            case 6:  wire_feeder.stop();  break;
            case 7:  wire_feeder.start(); break;
            case 10: break; // called from pwm class on arc generator turn on
            case 11: break; // called from pwm class on arc generator turn off
            default: break;
        }
        data = 0;
        vTaskDelay(1);
    }
    vTaskDelete(NULL);
}











//###################################################
// Set the flag for a sensor object reset
// Before processing the next i2s batch it will reset
// the sensor objects used to make decision
//###################################################
void G_SENSORS::reset_sensor_global(){ // called from planner on core1
    reset_sensor_state = true;
}


void G_SENSORS::generate_setpoint_min_max_adc(){
    acquire_i2s_lock();
    gscope.update_setpoint( cfd_setpoint_lower, cfd_setpoint_upper );
    reset_sensor_global();
    release_i2s_lock();
}

// only core 0
bool G_SENSORS::change_setting( setget_param_enum param_id, int int_value, float float_value ){
    bool restart_i2s_req = false;
    bool reset_req       = false;
    acquire_i2s_lock();
    vTaskDelay(1);
    switch( param_id ){
        case PARAM_ID_SETMIN:          cfd_setpoint_lower   = percentage_to_adc( float_value ); reset_req = true; break;
        case PARAM_ID_SETMAX:          cfd_setpoint_upper   = percentage_to_adc( float_value ); reset_req = true; break;
        case PARAM_ID_PROBE_TR_C:      cfd_setpoint_probing = percentage_to_adc( float_value ); reset_req = true; break;
        case PARAM_ID_USE_STOPGO:      use_stop_and_go_flags = int_value == 1 ? true : false; break;
        case PARAM_ID_I2S_RATE:        sample_rate   = int_value; restart_i2s_req = true; break;
        case PARAM_ID_I2S_BUFFER_L:    buffer_length = int_value; benchmark_buff_len = int_value; restart_i2s_req = true; break;
        case PARAM_ID_I2S_BUFFER_C:    buffer_count  = int_value; restart_i2s_req = true; break;
        case PARAM_ID_MAIN_AVG:        cfd_average_fast = MIN( int_value, cfd_average_slow ); break;
        case PARAM_ID_FAVG_SIZE:       cfd_average_slow = MAX( int_value, cfd_average_fast ); break;
        case PARAM_ID_RISE_BOOST:      cfd_average_rising_boost         = int_value;          break;
        case PARAM_ID_RETRACT_CONF:    retract_confirmations            = int_value;          break;
        case PARAM_ID_BEST_OF:         cfd_sample_pool_size             = int_value;          break;
        case PARAM_ID_VDROP_THRESH:    vfd_short_circuit_threshhold     = int_value;          break;
        case PARAM_ID_VFD_DROPS_BLOCK: vfd_channel_shorts_for_block     = int_value;          break;
        case PARAM_ID_VFD_SHORTS_POFF: vfd_channel_shorts_for_pulse_off = int_value;          break;
        case PARAM_ID_POFF_DURATION:   pulse_off_duration               = int_value;          break;
        case PARAM_ID_POFF_AFTER:      pwm_off_count                    = int_value;          break;
        case PARAM_ID_ZERO_JITTER:     zeros_jitter                     = int_value;          break;
        case PARAM_ID_ZERO_THRESH:     cfd_zero_treshhold               = int_value;          break;
        case PARAM_ID_READINGS_L:      zeros_in_a_row_max               = int_value;          break;
        case PARAM_ID_READINGS_H:      highs_in_a_row_max               = int_value;          break;
        case PARAM_ID_EDGE_THRESH:     cfd_edge_treshhold               = int_value;          break;
        case PARAM_ID_SHORT_DURATION:  short_circuit_max_duration_us    = int_value;          break;
        case PARAM_ID_PROBE_TR_V:      vfd_probe_trigger                = int_value;          break;
        default: break;
    }
    release_i2s_lock();
    if( reset_req ){
        vTaskDelay(10);
        generate_setpoint_min_max_adc(); // needed to update the setpoints on the scope etc
    }
    if( restart_i2s_req ){
        restart_i2s_flag.store( true );
    }
    return true;
}

int G_SENSORS::get_setting_int( setget_param_enum param_id ){
    int value = 0;
    acquire_i2s_lock();
    switch( param_id ){
        case PARAM_ID_I2S_RATE:        value = sample_rate;                      break;
        case PARAM_ID_I2S_BUFFER_L:    value = buffer_length;                    break;
        case PARAM_ID_I2S_BUFFER_C:    value = buffer_count;                     break;
        case PARAM_ID_BEST_OF:         value = cfd_sample_pool_size;             break;
        case PARAM_ID_FAVG_SIZE:       value = cfd_average_slow;                 break;
        case PARAM_ID_MAIN_AVG:        value = cfd_average_fast;                 break;
        case PARAM_ID_RISE_BOOST:      value = cfd_average_rising_boost;         break;
        case PARAM_ID_RETRACT_CONF:    value = retract_confirmations;            break;
        case PARAM_ID_VDROP_THRESH:    value = vfd_short_circuit_threshhold;     break;
        case PARAM_ID_VFD_DROPS_BLOCK: value = vfd_channel_shorts_for_block;     break;
        case PARAM_ID_VFD_SHORTS_POFF: value = vfd_channel_shorts_for_pulse_off; break;
        case PARAM_ID_POFF_DURATION:   value = pulse_off_duration;               break;
        case PARAM_ID_POFF_AFTER:      value = pwm_off_count;                    break;
        case PARAM_ID_ZERO_JITTER:     value = zeros_jitter;                     break;
        case PARAM_ID_ZERO_THRESH:     value = cfd_zero_treshhold;               break;
        case PARAM_ID_READINGS_L:      value = zeros_in_a_row_max;               break;
        case PARAM_ID_READINGS_H:      value = highs_in_a_row_max;               break;
        case PARAM_ID_EDGE_THRESH:     value = cfd_edge_treshhold;               break;
        case PARAM_ID_SHORT_DURATION:  value = short_circuit_max_duration_us;    break;
        case PARAM_ID_PROBE_TR_V:      value = vfd_probe_trigger;                break;
        default: break;
    }
    release_i2s_lock();
    return value;
}

float G_SENSORS::get_setting_float( setget_param_enum param_id ){
    float value = 0.0;
    acquire_i2s_lock();
    switch( param_id ){
        case PARAM_ID_SETMIN:     value = adc_to_percentage( cfd_setpoint_lower );   break;
        case PARAM_ID_SETMAX:     value = adc_to_percentage( cfd_setpoint_upper );   break;
        case PARAM_ID_PROBE_TR_C: value = adc_to_percentage( cfd_setpoint_probing ); break;
        default: break;
    }
    release_i2s_lock();
    return value;
}

bool G_SENSORS::get_setting_bool( setget_param_enum param_id ){
    bool value = false;
    acquire_i2s_lock();
    switch( param_id ){
        case PARAM_ID_USE_STOPGO:  value = use_stop_and_go_flags; break;
        default: break;
    }
    release_i2s_lock();
    return value;
}





//###########################################################################
// Called from planner on core 1 to request the motionplan
//###########################################################################
IRAM_ATTR int get_calculated_motion_plan(){
    if( reset_vsense_queue.load() ){
        std::unique_lock<std::mutex> lock(mtx);
        bool signaled = cv.wait_for(lock, std::chrono::microseconds(1000000), [] { 
            return new_motion_plan.load(); 
        });
        if (!signaled) {
            return MOTION_PLAN_TIMEOUT;
        }
    }
    new_motion_plan.store( false );
    return motion_plan_atomic.load();
}




// This is dirty
// It read the voltage to create a notification
// if voltage levels are zero on process start 
// needs a remake
int G_SENSORS::get_vfd_feedback( int counts ){
    debuglog("Validating vFd feedback", DEBUG_LOG_DELAY );
    vTaskDelay(200);
    acquire_i2s_lock();
    uint32_t avg = 0;
    for( int i = 0; i < counts; ++i ){
        avg += adc_data.voltage_channel_adc_recent;
        vTaskDelay(5);
    }

    int vfd_reading = avg/counts;
    release_i2s_lock();
    return vfd_reading;
}

//###########################################################################
// Toogle PWM off if needed for protection
//###########################################################################
static IRAM_ATTR void pwm_off_protection(){ 
    if( is_system_mode_edm() && (( vfd_pin_is_low // voltage feedback ADC i2s batch was below threshold 
                    //|| had_short_condition > 0
                    || adc_data.voltage_channel_shorts > vfd_channel_shorts_for_pulse_off // given number of samples in the i2s batch where below threshold
                 ) && pulse_off_duration>0 && vfd_low_count > pwm_off_count )){ 
        block_short_low_override = true;
        arcgen.set_pwm_state( true );
        delayMicroseconds( pulse_off_duration * pconf.pwm_period_us );
        arcgen.reset_pwm_state();
        vfd_last_short_condition = esp_timer_get_time();
    }
    if( !vfd_pin_is_low ) { 
        vfd_low_count = 0;
    }
}

//###########################################################################
// Push samples to the scope ( the lock does nothing currently )
//###########################################################################
static IRAM_ATTR bool adc_to_scope_high_res( uint16_t sample, int8_t plan ){
    if( scope_batch_ready.load() ){ return false; } // full batch waiting to get processed

    #ifdef ENABLE_SCOPE_LOCK
    acquire_lock_for( ATOMIC_LOCK_GSCOPE );
    #endif

    if( gscope.add_meta() ){
        uint8_t mstate = 2;
        gscope.set_sample_rate( benchmark_ksps );
        gscope.set_total_avg( adc_data.avg_full_range );
        gscope.set_digital_state( adc_data.voltage_channel_adc_recent );
        if( adc_data.zeros_in_a_row >= zeros_in_a_row_max ){
            mstate = 1;
        } else if( adc_data.highs_in_a_row >= highs_in_a_row_max ){
            mstate = 0;
        }
        gscope.set_allow_motion( mstate );
    }
    gscope.add_to_scope( sample, plan ); // sets the flag so it needs to be last

    #ifdef ENABLE_SCOPE_LOCK
    release_lock_for( ATOMIC_LOCK_GSCOPE );
    #endif

    return true;
}




int G_SENSORS::get_sample_rate(){
    return sample_rate;
}




IRAM_ATTR void adc_monitor_task(void *parameter){ 
    G_SENSORS *__this = (G_SENSORS *)parameter;

    static DRAM_ATTR i2s_shift_out_data i2s_data;
    static DRAM_ATTR uint32_t work_index = 0;
    static DRAM_ATTR uint32_t multisample_counts = 0;
    static DRAM_ATTR uint32_t multisample_loop_index = 0;
    static DRAM_ATTR uint32_t multisample_work_index = 0;
    static DRAM_ATTR uint32_t multisample_buffer[vsense_sampler_buffer_size] = {0,};

    //######################################################
    // Some counters to count some things. Don't ask....
    //######################################################
    static DRAM_ATTR uint32_t cfd_above_setmax_retract_confirm_counter = 0; // if plan is >=4 (abiver setpoint and something else-..) it counts for confirmation before providing a retraction instruction
    static DRAM_ATTR uint32_t cfd_probe_confirm_counter                = 0; // on probe positive it starts to count and confirm the touch and resets if it fails; Maybe deprecated as I2S sampling is very stable.
    static DRAM_ATTR uint32_t vfd_short_recover_confirm_counter        = 0; // after a short condition it confirms a low load situation x times before switching back to forward motion


    static DRAM_ATTR int8_t cfd_plan_realtime     = 0; 
    static DRAM_ATTR int8_t cfd_plan_fast_average = 0; 
    static DRAM_ATTR int8_t cfd_plan_slow_average = 0;
    static DRAM_ATTR bool   create_negative_plan  = false;


    static DRAM_ATTR int data                = 1;
    static DRAM_ATTR bool high_res           = scope_use_high_res.load();    
    static DRAM_ATTR uint32_t add_sample_num = 0;

    __this->begin();                        // start with the default settings
    __this->wait_for_idle();                // idle state is set after it is ready

    adc_monitor_task_running.store( true ); // set flag that task is running
    restart_i2s_flag.store( true );         // flag for i2s restart on initial run

    static DRAM_ATTR bool had_lock = false;




    static const DRAM_ATTR uint32_t adc_jitter      = 20;
    static DRAM_ATTR uint32_t samples_per_pulse_max = 0;
    static DRAM_ATTR uint32_t samples_per_pulse     = 0;
    static DRAM_ATTR uint32_t cfd_high_samples      = 0;
    static DRAM_ATTR uint32_t captured_pulses       = 0;
    static DRAM_ATTR uint32_t captured_pulses_max   = 1;
    static DRAM_ATTR bool     capture_begin         = false;
    static DRAM_ATTR uint32_t capture_rounds        = 0;





    for(;;){

        if( had_lock ){ release_i2s_lock(); }

        xQueueReceive(adc_read_trigger_queue, &data, portMAX_DELAY);

        if( restart_i2s_flag.load() ){
            __this->restart();
            vTaskDelay(DEBUG_LOG_DELAY); 
            continue;
        }
        

        acquire_i2s_lock(); 
        had_lock = true;

        //#########################################################
        // Reset sensor object if needed. Set by the planner on
        // core 1 after pause / pause recovery end / flush end
        //#########################################################
        if( reset_sensor_state ){
            reset_sensor_state = false;
            arcgen.reset_pwm_state();
            high_res                                 = scope_use_high_res.load();
            cfd_above_setmax_retract_confirm_counter = 0;
            cfd_probe_confirm_counter                = 0;
            vfd_short_recover_confirm_counter        = 0;
            rising_edge_normalized                   = true;
            edge_recovery_failed                     = false;
            rising_edge_block                        = false;
            multiple_rising_edges_count              = 0;
            adc_pre_rising_edge                      = -1;
            last_rising_edge_adc                     = cfd_setpoint_upper; // dirty solution from the past to create a falling edge more easy on the next read and free the lock faster
            adc_data.zeros_in_a_row                  = 0;
            adc_data.highs_in_a_row                  = 0;
            adc_data.previous_edge                   = 0;
            adc_data.edge_type                       = 0;
            vfd_low_reset_count                      = 0;
            vfd_had_short_condition                  = 0;
            block_short_low_override                 = false;
            benchmark_adc_counter                    = 0;
            samples_per_pulse_max                    = 0;
            captured_pulses_max                      = 1;
            capture_rounds                           = 0;
            adc_data.current_channel_high_load       = 0;
        }
        //#########################################################
        // Get the I2S buffer and parse it to a single ADC value
        //#########################################################
        if( arcgen.get_pwm_is_off() ){
            if( !arcgen.is_running() ){ vTaskDelay(10); }
            benchmark_adc_counter = 0;
            continue;
        }
        //###########################################################################
        // Get the i2s buffer with the precious samples
        //###########################################################################
        i2s_data.success = true;
        int    samples_read;
        size_t bytes_read;
        if( ESP_OK == i2s_read(I2S_NUM_0, dma_buffer, gsense.num_bytes, &bytes_read, I2S_TIMEOUT_TICKS) ){
            samples_read = bytes_read >> 1;
            if( samples_read == gsense.buffer_length ){
                i2s_data.success = true;
            }
        } 

        if( i2s_data.success ){

            //###########################################################################
            // Increment the batch counter for the ksps benchmark
            //###########################################################################
            ++benchmark_adc_counter;

            //###########################################################################
            // Reset some things needed to parse the i2s batch
            //###########################################################################
            i2s_data.read_sum_a                = 0;
            i2s_data.read_sum_b                = 0;
            i2s_data.count_a                   = 0;
            i2s_data.count_b                   = 0;
            adc_data.voltage_channel_shorts    = 0;
            adc_data.current_channel_high_load = 0;


            samples_per_pulse = 0;
            cfd_high_samples  = 0;
            captured_pulses   = 0;
            capture_begin     = false;


            //#################################################################################################
            // Process the I2S buffer. Any tiny change here like a simple addition or anything can decrease the
            // max sample rate easily by several kSps and more
            // this section loops over the I2S buffer that contains a batch of adc readings
            //#################################################################################################
            for( work_index = 0; work_index < samples_read; work_index++ ){

                i2s_data.selected_channel = (dma_buffer[ work_index ] >> 12) & 0x07;
                i2s_data.sample           = dma_buffer[ work_index ] & 0xfff;

                if( i2s_data.selected_channel == 6 ){ // current sense


                        if( !capture_begin && i2s_data.sample >= cfd_edge_treshhold ){ // 
                            capture_begin = true;
                            if( ++captured_pulses > captured_pulses_max ){
                                captured_pulses_max = captured_pulses;
                            }
                        }

                        if( capture_begin ){
                            //############################################################
                            // Either within a pulse capture or the start of a fresh pulse
                            //############################################################
                            if( i2s_data.sample > cfd_setpoint_upper ){
                                ++cfd_high_samples;
                            }

                            if( i2s_data.sample <= adc_jitter ){

                                if( samples_per_pulse_max > 2 && cfd_high_samples >= ( samples_per_pulse_max>>1 ) ){
                                    ++adc_data.current_channel_high_load;
                                }

                                capture_begin     = false;
                                samples_per_pulse = 0;
                                cfd_high_samples  = 0;

                            } else {
                                if( ++samples_per_pulse > samples_per_pulse_max ){
                                    samples_per_pulse_max = samples_per_pulse;
                                }
                            }
                            


                        }




                    i2s_data.read_sum_a += i2s_data.sample;
                    ++i2s_data.count_a;
                    if( high_res ){ // Note: this only adds the current motionplan and not the one for this recent sample
                        adc_to_scope_high_res( i2s_data.sample, adc_data.plan ); 
                    }


                } else if( i2s_data.selected_channel == 7 ){ // voltage sense
                    i2s_data.read_sum_b += i2s_data.sample;
                    ++i2s_data.count_b;
                    if( i2s_data.sample < vfd_short_circuit_threshhold ){
                        ++adc_data.voltage_channel_shorts;
                    }
                }



            }
            // create the vFd and cFd average
            adc_data.voltage_channel_adc_recent = i2s_data.read_sum_b > 0 ? int( i2s_data.read_sum_b / i2s_data.count_b ) : 0; 
            adc_data.recent                     = i2s_data.read_sum_a > 0 ? int( i2s_data.read_sum_a / i2s_data.count_a ) : 0; 


            //#########################################################
            // Single discharge analytics postprocessing
            //#########################################################
            if( 
                   adc_data.recent < 30 // tiny average cfd 
                || capture_rounds  < 20 // not enough data yet
            ){
                captured_pulses                    = 0;
                adc_data.current_channel_high_load = 0;
            }
            if( capture_rounds < 20 ){ ++capture_rounds; }




            //#########################################################
            // Create the voltage feedback (vfd) state
            //#########################################################
            if( 
                adc_data.voltage_channel_adc_recent < vfd_short_circuit_threshhold //||adc_data.voltage_channel_shorts > 10
            ){ // voltage dropped below threshold; shorted;
                vfd_pin_is_low      = true;
                vfd_low_reset_count = 3;
                vfd_low_count = (++vfd_low_count) & 131071;
            } else {
                if( vfd_low_reset_count == 0 ){
                    vfd_pin_is_low = false;
                } else if( vfd_low_reset_count > 0 ){
                    --vfd_low_reset_count;
                }
            }

            //#########################################################
            // Add protection logic if not probing
            //#########################################################
            if( !is_machine_state( STATE_PROBING ) ){
                if( block_short_low_override || vfd_had_short_condition>0 ){ 
                    if( --vfd_had_short_condition < 0 || adc_data.recent > cfd_setpoint_lower ){
                        vfd_had_short_condition = 0;
                    }
                    block_short_low_override = false;
                    if( adc_data.recent < cfd_setpoint_lower && !vfd_pin_is_low ){
                        pwm_off_protection();
                        continue; // ignore the reading after a manual pwm off
                    }
                }
                pwm_off_protection();
            }

            //#########################################################################################################
            // Wait until the adc pool is filled and select the highest value. If load condition is detected skip the 
            // pool filling and go on to react fast
            //#########################################################################################################
            if( ++num_samples_in_pool < ( gconf.gedm_retraction_motion ? cfd_sample_pool_size+1 : cfd_sample_pool_size ) ){
                highest_adc = MAX( adc_data.recent, highest_adc );
                if( 
                    //!pin_is_low &&
                    highest_adc <= cfd_setpoint_lower
                ){ // only collect more if forward motion is indicated
                    pwm_off_protection();
                    continue;
                }
                adc_data.recent = highest_adc;
            } 

            //#########################################################
            // Reset to default
            //#########################################################
            highest_adc                  = 0;
            num_samples_in_pool          = 0;
            falling_edge_accepted        = false;
            multiple_rising_edges        = false;
            is_rising_edge               = false;
            is_falling_edge              = false;
            adc_data.edge_type           = 0; // default

            //##############################################################################
            // Determine edge type
            //##############################################################################
            if( adc_data.recent > adc_data.sampled + cfd_edge_treshhold ){ // new sample is higher then the previous multisample plus edge threshold
                is_rising_edge = true;

                ++multiple_rising_edges_count;

                //###########################################################################
                // Rising edge...
                //###########################################################################
                rising_edge_block      = true;
                falling_edge_block     = false;
                rising_edge_normalized = false;
                edge_to_edge_block_count = pconf.pwm_2500us_in_pulses; // 2500us until forced recovery (this is not exact. i2s timing differs etc )

                if( adc_pre_rising_edge == -1 ){
                    //###########################################################################
                    // Lock the previous adc value. After a spark the feedback should normalize 
                    // back to the pre rising edge state. "Should"... "Somehow"... "Ideally"...
                    //###########################################################################
                    adc_pre_rising_edge    = adc_data.sampled; 
                    last_rising_edge_adc   = adc_data.recent;
                    edge_recovery_failed = false;
                } else {
                    //###############################
                    // existing rising edges
                    //###############################
                    multiple_rising_edges = true;
                }

            } else if( 
                rising_edge_block && // rising_edge_block &&
                adc_data.recent < adc_data.sampled - cfd_edge_treshhold 
            ){
                is_falling_edge = true;

                falling_edge_block_start_time = esp_timer_get_time();
                rising_edge_block           = false;
                falling_edge_block          = true;
                adc_data.edge_type                     = -1;

            } else {
                // no load condition after a rising+falling edge. This would end in a motion block loop and can be a load and a no load situation
                // edge not rising but still at an equal level to another rising edge or edge not falling but still at a low level
                if( falling_edge_block || rising_edge_block ){// rising_edge_block ){
                    if( esp_timer_get_time() - falling_edge_block_start_time > pconf.pwm_period_times_ten ){
                        rising_edge_block  = false;
                        falling_edge_block = false;
                    }
                }
            }


  
            if( adc_pre_rising_edge != -1 ){
                //###########################################################################
                // Unsolved rising edge waiting for normalisation
                //###########################################################################
                if( --edge_to_edge_block_count <= 0 || adc_data.recent <= adc_pre_rising_edge ){
                    // adc back to the pre rising level
                    falling_edge_accepted = true;
                    multiple_rising_edges_count=0;
                    if( edge_to_edge_block_count <= 0 ){
                        edge_recovery_failed = true;
                    } else {
                        edge_recovery_failed = false;
                    }
                    adc_pre_rising_edge       = -1;
                    rising_edge_normalized = true;
                    edge_to_edge_block_count  = 0;
                }
            }



            if( is_rising_edge || !rising_edge_normalized ){
                adc_data.edge_type = 1;
            } 
            if( multiple_rising_edges ){
                adc_data.edge_type = multiple_rising_edges_count > 4 ? 3 : 2;
            }




            //###########################################################################
            // Add sample to multisampler
            //###########################################################################
            add_sample_num = ( is_rising_edge ? MIN( cfd_average_rising_boost, cfd_average_fast ) : 1 );
            if (add_sample_num <= 1) {
                multisample_counts                     = (multisample_counts + 1) & (vsense_sampler_buffer_size - 1);
                multisample_buffer[multisample_counts] = adc_data.recent;
            } else {
                for (int adds = 0; adds < add_sample_num; ++adds) {
                    multisample_counts                     = (multisample_counts + 1) & (vsense_sampler_buffer_size - 1);
                    multisample_buffer[multisample_counts] = adc_data.recent;
                }
            }

            //###########################################################################
            // Determine slow and fast average
            //###########################################################################
            if( cfd_average_slow <= 1 ){

                adc_data.sampled        = adc_data.recent; // fast cfd average
                adc_data.avg_full_range = adc_data.recent; // slow cfd average

            } else {

                if( cfd_average_fast > cfd_average_slow ){
                    cfd_average_fast = cfd_average_slow;
                }

                adc_data.sampled        = 0; // fast cfd average
                adc_data.avg_full_range = 0; // slow cfd average
                multisample_work_index  = multisample_counts;

                bool     fast_added         = false;
                uint16_t fast_avg_minus_one = cfd_average_fast - 1;
                for( multisample_loop_index = 0; multisample_loop_index < cfd_average_slow; ++multisample_loop_index ){
                    if( multisample_loop_index < cfd_average_fast ){ // fast cfd avergae
                        adc_data.sampled += multisample_buffer[multisample_work_index];
                    } 
                    if( multisample_loop_index >= fast_avg_minus_one ){
                        if( !fast_added ){
                            adc_data.avg_full_range = adc_data.sampled;
                            fast_added = true;
                        }
                        adc_data.avg_full_range += multisample_buffer[multisample_work_index]; // slow cfd average
                    }
                    multisample_work_index = (multisample_work_index - 1) & (vsense_sampler_buffer_size - 1); // move reverse
                }

                adc_data.sampled        /= cfd_average_fast;
                adc_data.avg_full_range /= cfd_average_slow;

            }




            //###########################################################################
            // Create the motion plan
            //###########################################################################
            if( is_machine_state( STATE_PROBING ) ){



                //#############################################################
                // Probing motion plan is a little different
                //#############################################################
                cfd_plan_fast_average = MOTION_PLAN_HOLD_SOFT; 

                if( probe_touched ){ 
                
                    reset_sensor_state = true;
                    cfd_plan_fast_average = MOTION_PLAN_SOFT_SHORT; 
                
                } else {

                    if( 
                        adc_data.voltage_channel_adc_recent < vfd_probe_trigger || // voltage feedback vfd below trigger 
                        adc_data.recent > cfd_setpoint_probing                     // current feedback cfd above trigger
                    ){
                        cfd_plan_fast_average = MOTION_PLAN_HOLD_SOFT; // pause motion
                        if( ++cfd_probe_confirm_counter >= MOTION_PLAN_PROBE_CONFIRMATIONS ){
                            cfd_plan_fast_average = MOTION_PLAN_SOFT_SHORT; // probe confirmed
                            probe_touched      = true;
                            reset_sensor_state = true;
                        }
                    } else {
                        probe_touched = false;
                        if( cfd_probe_confirm_counter > 0 || adc_data.edge_type > 0 ){
                            cfd_plan_fast_average = MOTION_PLAN_HOLD_SOFT; // skip one round after a positive
                        } else {
                            cfd_plan_fast_average = MOTION_PLAN_FORWARD;
                        }
                        cfd_probe_confirm_counter = 0; // fully reset the counter
                    }
                }

            } else {

                create_negative_plan  = false; // negative plan is used to keep track of no load steps in the planner if we override a forward plan

                //###########################################################################
                // Calculate the initial raw pure plan based on the feedback provided
                //###########################################################################
                cfd_plan_fast_average = adc_data.sampled        < cfd_setpoint_lower ? MOTION_PLAN_FORWARD : MOTION_PLAN_HOLD_SOFT; // fast average; can be set to single reading too...
                cfd_plan_slow_average = adc_data.avg_full_range < cfd_setpoint_lower ? MOTION_PLAN_FORWARD : MOTION_PLAN_HOLD_SOFT; // slow average
                cfd_plan_realtime     = adc_data.recent         < cfd_setpoint_lower ? MOTION_PLAN_FORWARD : MOTION_PLAN_HOLD_SOFT; // most recent sample

                //###########################################################################
                // At this point every plan is either a 1 or a 2
                // Now check for 4 and 5
                //###########################################################################
                if( adc_data.avg_full_range > cfd_setpoint_upper || vfd_low_reset_count>0 ){ 
                    cfd_plan_fast_average = MAX( cfd_plan_fast_average, MOTION_PLAN_SOFT_SHORT ); // slow average cfd above upper setpoint or vfd short reset counter not done; kind of heursitic
                }

                //##########################################################################
                // Hard short circuit conditions
                //##########################################################################
                if( // low vfd pin is always a hard short circuit and bad
                    vfd_pin_is_low                            // voltage dropped below threshold
                    //|| (adc_data.previous_plan==5&&(plan==4))
                    //|| adc_data.current_channel_high_load > 0                            // high load cfd pulse in channel
                    //|| captured_pulses_max > 0 && captured_pulses >= captured_pulses_max // high number of pulses in channel
                ){ 
                    cfd_plan_fast_average = MOTION_PLAN_HARD_SHORT; // retract at high speed for real shorts
                    //if( vfd_pin_is_low ){
                    vfd_had_short_condition           = 3;
                    vfd_short_recover_confirm_counter = 0;
                    //}

                } else if(
                    adc_data.current_channel_high_load > 0                                   // high load cfd pulse in channel
                    || ( captured_pulses_max > 0 && captured_pulses >= captured_pulses_max ) // high number of pulses in channel 
                ){ 
                    if( cfd_plan_fast_average <= MOTION_PLAN_HOLD_SOFT ){
                        create_negative_plan = true;
                    }


                    //cfd_plan_fast_average = MOTION_PLAN_SOFT_SHORT; 
                    cfd_plan_fast_average = MOTION_PLAN_HARD_SHORT; // retract at high speed for real shorts
                    //if( vfd_pin_is_low ){
                    vfd_had_short_condition           = 3;
                    vfd_short_recover_confirm_counter = 0;

                }



                //###########################################################################
                // Check if anything is against a forwward motion
                // If any of the blocking conditions returns true it will increase the plan
                // to the next higher one (MOTION_PLAN_HOLD_SOFT) and flag it negative
                // negative plan indicates that the forward plan was overwritten or something
                //###########################################################################
                if( cfd_plan_fast_average == MOTION_PLAN_FORWARD ){ 
                    if( 
                        !( cfd_plan_slow_average == MOTION_PLAN_FORWARD 
                        && cfd_plan_fast_average == MOTION_PLAN_FORWARD 
                        && cfd_plan_realtime     == MOTION_PLAN_FORWARD
                        && adc_data.avg_full_range <= adc_data.avg_full_range_last // only on a falling full range average
                        && vfd_low_count <= 0 // only if there was no short going on
                        && ( esp_timer_get_time() - vfd_last_short_condition >= vfd_short_recover_time_us ) // last short given microseconds in the past
                        && adc_data.voltage_channel_shorts <= vfd_channel_shorts_for_block )
                    ){
                        // failed to confirm a forward motion; increase the plan to hold and flag it negative
                        cfd_plan_fast_average = MOTION_PLAN_HOLD_SOFT; // increase to MOTION_PLAN_HOLD_SOFT and flag negative 
                        create_negative_plan = true;
                    }
                }

                //###########################################################################
                // Set the stop/go signals
                //###########################################################################
                if( adc_data.recent <= cfd_zero_treshhold ){
                    if( ++adc_data.zeros_in_a_row > zeros_jitter ){
                        adc_data.highs_in_a_row = 0;
                    }
                } else {
                    if( adc_data.recent >= cfd_setpoint_upper ){ 
                        ++adc_data.highs_in_a_row;
                    } else {
                       adc_data.highs_in_a_row = 0;
                    }
                    adc_data.zeros_in_a_row = 0;
                }

                //###########################################################################
                // Alter plan based on wave edge
                //###########################################################################
                if( 
                    adc_data.edge_type >= 2 || falling_edge_block //|| rising_edge_block
                ){ // multiple rising edges
                    if( cfd_plan_fast_average == MOTION_PLAN_FORWARD ){ create_negative_plan = true; }
                    cfd_plan_fast_average = MAX( MOTION_PLAN_HOLD_SOFT, cfd_plan_fast_average );
                }

                adc_data.previous_edge = adc_data.edge_type;

                //###########################################################################
                // Alter plan based on stop/go signals
                //###########################################################################
                if( use_stop_and_go_flags ){
                    if( cfd_plan_fast_average == MOTION_PLAN_FORWARD && adc_data.zeros_in_a_row <= zeros_in_a_row_max ){
                        cfd_plan_fast_average = MOTION_PLAN_HOLD_SOFT;
                        create_negative_plan = true;
                    } else if( adc_data.highs_in_a_row >= highs_in_a_row_max ){
                        if( cfd_plan_fast_average == MOTION_PLAN_FORWARD ){ create_negative_plan = true; }
                        cfd_plan_fast_average = MAX( cfd_plan_fast_average, MOTION_PLAN_HOLD_HARD ); // too many high load readings in a row; heurisitics
                    }
                }

                //###########################################################################
                // Final checks
                //###########################################################################
                adc_data.avg_full_range_last = adc_data.avg_full_range;

                if( vfd_had_short_condition > 0 ){
                    if( cfd_plan_fast_average < MOTION_PLAN_HARD_SHORT ){
                        ++vfd_short_recover_confirm_counter;
                        if( vfd_short_recover_confirm_counter <= 3 ){
                            cfd_plan_fast_average = MOTION_PLAN_HARD_SHORT;
                        } 
                    }
                    if( cfd_plan_fast_average < MOTION_PLAN_SOFT_SHORT && vfd_short_recover_confirm_counter >= 3 ){
                        vfd_had_short_condition = 0;
                    }
                }

                if( cfd_plan_fast_average >= MOTION_PLAN_SOFT_SHORT && ++cfd_above_setmax_retract_confirm_counter < retract_confirmations ){
                    --cfd_plan_fast_average;
                } else if( cfd_plan_fast_average < MOTION_PLAN_SOFT_SHORT ){
                    cfd_above_setmax_retract_confirm_counter = 0;
                    vfd_low_reset_count = 0;
                }

                if( gconf.gedm_retraction_motion ){
                    cfd_plan_fast_average = MAX( cfd_plan_fast_average, cfd_plan_slow_average );
                } else if( create_negative_plan ){ // no negative plan for retractions
                    cfd_plan_fast_average = (~cfd_plan_fast_average) + 1; //plan *= -1;
                }
       
            }

    




            adc_data.plan = cfd_plan_fast_average;   

            //###########################################################################
            // Distribute the motion plan
            //###########################################################################
            motion_plan_atomic.store( adc_data.plan );

            if( reset_vsense_queue.load() ){
                new_motion_plan.store(true);
                cv.notify_all();
            }

            if( !high_res ){
                adc_to_scope_high_res( adc_data.sampled, adc_data.plan ); 
            }

        } else {
            restart_i2s_flag.store( true );
        } 


    }
    vTaskDelete(NULL);
}







//###################################################
// Start tasks
//###################################################
void G_SENSORS::create_tasks(){
    xTaskCreatePinnedToCore( remote_control_task, "remote_control_task_handle", STACK_SIZE_A, this, 
        1,//TASK_SENSORS_DEFAULT_PRIORITY, 
        &remote_control_task_handle, I2S_TASK_A_CORE);
    xTaskCreatePinnedToCore( adc_monitor_task, "adc_monitor_task", STACK_SIZE_B, this, 
        TASK_VSENSE_RECEIVER_PRIORITY, 
        //5,
        &adc_monitor_task_handle,  I2S_TASK_B_CORE); 
    while( !adc_monitor_task_running.load() ){
        vTaskDelay(10);
    }
}



void G_SENSORS::sensor_end(){
    debuglog("Stopping sensors");
    if( ! sennsors_running.load() ){
        debuglog("Sensors not running",DEBUG_LOG_DELAY);
        return;
    }
    vTaskDelete( remote_control_task_handle );
    vTaskDelete( adc_monitor_task_handle );
    if( adc_read_trigger_queue != NULL ){ vQueueDelete( adc_read_trigger_queue ); adc_read_trigger_queue = NULL; }
    if( remote_control_queue != NULL ){ vQueueDelete( remote_control_queue ); remote_control_queue = NULL; }    
    vTaskDelay(DEBUG_LOG_DELAY);
    gsense.stop( false );
}

//###################################################
// 
//###################################################
void G_SENSORS::setup(){
    vTaskDelay(DEBUG_LOG_DELAY);
    detachInterrupt( digitalPinToInterrupt( ON_OFF_SWITCH_PIN ) );
    detachInterrupt( digitalPinToInterrupt( STEPPERS_LIMIT_ALL_PIN ) );
    attachInterrupt( ON_OFF_SWITCH_PIN,      motion_switch_on_interrupt, CHANGE );
    attachInterrupt( STEPPERS_LIMIT_ALL_PIN, limit_switch_on_interrupt,  CHANGE );
}





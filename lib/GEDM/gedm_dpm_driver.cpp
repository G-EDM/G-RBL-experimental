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


#include "gedm_dpm_driver.h"

DRAM_ATTR int dpm_locked          = false;
DRAM_ATTR int settings_milliamps  = 0;
DRAM_ATTR int settings_millivolts = 0;
DRAM_ATTR int measured_ma         = 0;
DRAM_ATTR int measured_mv         = 0;



std::map<dpm_msg, const char*> dpm_messages = {
    { DPM_OK,         "" },
    { DPM_COM_FAIL,   "*DPM communication failed" },
    { DPM_ENA_FAIL,   "  Failed to enable DPM" },
    { DPM_ENA_FAIL_B, "  Please turn it on manually" },
    { DPM_ENA_OK,     "@DPM enabled" },
    { DPM_DIS_OK,     "@DPM disabled" },
    { DPM_NO_SUPPORT, " DPM/DPH support not enabled" },
    { DPM_WAIT,       "@Waiting for DPM/DPH connection"},
    { DPM_PREPARE,    "Preparing DPM/DPH"},
    { DPM_TRY_ON,     "Enabling DPM" },
    { DPM_TRY_OFF,    "Disabling DPM" }
};


G_EDM_DPM_DRIVER dpm_driver;


G_EDM_DPM_DRIVER::G_EDM_DPM_DRIVER(){
    is_ready = false;
}

void G_EDM_DPM_DRIVER::end(){
    if( dpm_serial ){
        dpm_serial->end();
    }
}


bool G_EDM_DPM_DRIVER::setup( HardwareSerial& serial ){
    debuglog( dpm_messages[DPM_PREPARE], DEBUG_LOG_DELAY );
    dpm_serial = &serial;
    //pinMode( DPM_INTERFACE_TX, OUTPUT );
    //pinMode( DPM_INTERFACE_RX, INPUT );
    return true;
}
bool G_EDM_DPM_DRIVER::init(){
    dpm_serial->updateBaudRate( DPM_RXTX_SERIAL_COMMUNICATION_BAUD_RATE );
    dpm_serial->flush();
    return true;
}

bool G_EDM_DPM_DRIVER::power_on_off( int turn_on, int max_retry ){
    debuglog( ( turn_on ? dpm_messages[DPM_TRY_ON] : dpm_messages[DPM_TRY_OFF] ), DEBUG_LOG_DELAY );
    char response[BUFFER_SIZE];
    char command_buf[10];
    bool success = false;
    sprintf(command_buf,":01w12=%d,", turn_on); 
    do{
        dpm_serial->flush();
        success = send( command_buf, response );
        if( success ){ //:01ok should be returned
            success = extract_ok( response );
        } else {
            vTaskDelay(100);
        }
        dpm_serial->flush();
        vTaskDelay(10);
    } while ( !success && --max_retry>=0 );
    if( !success ){
        debuglog( dpm_messages[DPM_COM_FAIL] );
        if( turn_on ){
            debuglog( dpm_messages[DPM_ENA_FAIL] );
            debuglog( dpm_messages[DPM_ENA_FAIL_B] );
        }
        vTaskDelay(1000);
    } else {
        if( turn_on ){
            debuglog( dpm_messages[DPM_ENA_OK] );
        } else {
            debuglog( dpm_messages[DPM_DIS_OK] );
        }
    }
    return success;
}

//##############################################################
// float value = current in amps
//##############################################################
bool G_EDM_DPM_DRIVER::set_setting_current(float value){
    char response[BUFFER_SIZE];
    int i_value = floor(value * 1000);
    char command_buf[15];
    sprintf(command_buf,":01w11=%d,", i_value);  
    int  max_retry = 3;
    bool success   = false;
    do{
        dpm_serial->flush();
        success = send( command_buf, response );
        if( success ){ //:01ok should be returned
            success = extract_ok( response );
        } else {
            vTaskDelay(100);
        }
        dpm_serial->flush();
        vTaskDelay(10);
    } while ( !success && --max_retry>=0 );
    return success ? true : false;
}


//##############################################################
// float value = voltage in volts
//##############################################################
bool G_EDM_DPM_DRIVER::set_setting_voltage(float value){
    char response[BUFFER_SIZE];
    int i_value = floor(value * 100);
    char command_buf[15];
    sprintf(command_buf,":01w10=%d,", i_value);  
    int  max_retry = 3;
    bool success   = false;
    do{
        dpm_serial->flush();
        success = send( command_buf, response );
        if( success ){ //:01ok should be returned
            success = extract_ok( response );
        } else {
            vTaskDelay(100);
        }
        dpm_serial->flush();
        vTaskDelay(10);
    } while ( !success && --max_retry>=0 );
    return success ? true : false;
}

bool G_EDM_DPM_DRIVER::get_power_is_on( void ){
    // send cmd: :01r12=0
    // receive:  :01r12=0, or  :01r12=1,
    // 0 = off, 1 = on
    char response[BUFFER_SIZE];
    char command_buf[10];
    int status;
    sprintf(command_buf,":01r12=%d,", 0);  
    int  max_retry = 3;
    bool success   = false;
    do{
        dpm_serial->flush();
        success = send( command_buf, response );
        if( success ){ 
            success = extract( status, 1, response );
        } else {
            vTaskDelay(100);
        }
        vTaskDelay(10);
        dpm_serial->flush();
    } while ( !success && --max_retry>=0 );
    if( ! success ){ return false; }
    return status == 1 ? true : false;
}//:01r12=0,

bool G_EDM_DPM_DRIVER::get_setting_voltage( int &voltage ){
    char response[BUFFER_SIZE];
    dpm_serial->flush();
    char command_buf[10];
    sprintf(command_buf,":01r10=%d,", 0);   
    int  max_retry = 3;
    bool success   = false;

    do{
        dpm_serial->flush();
        success = send( command_buf, response );
        if( success ){ 
            extract( voltage, 10, response );
            success = voltage < 0 ? false : true;
        } else {
            vTaskDelay(100);
        }
        vTaskDelay(10);
        dpm_serial->flush();
    } while ( !success && --max_retry>=0 );

    if( ! success ){
        voltage = -1;
        return success;
    }
    memcpy(&settings_millivolts, &voltage, sizeof(int));
    return true;
}
bool G_EDM_DPM_DRIVER::get_setting_current( int &current ){
    char response[BUFFER_SIZE];
    dpm_serial->flush();
    char command_buf[10];
    sprintf(command_buf,":01r11=%d,", 0);  
    int  max_retry = 3;
    bool success   = false;
    do{
        dpm_serial->flush();
        success = send( command_buf, response );
        if( success ){ 
            extract( current, 1, response );
            success = current < 0 ? false : true;
        } else {
            vTaskDelay(100);
        }
        vTaskDelay(10);
        dpm_serial->flush();
    } while ( !success && --max_retry>=0 );

    if( ! success ){
        current = -1;
        return success;
    }
    memcpy(&settings_milliamps, &current, sizeof(int));
    return true;
}


bool IRAM_ATTR G_EDM_DPM_DRIVER::send( char* cmd, char* response, bool blocking ){
    while( dpm_locked ){}
    dpm_locked = true;
    int8_t retry = 0;
    bool completed = false;
    
    
    //vTaskDelay(1000);
    //dpm_serial->flush();


    do{
        if( retry > 10 ){vTaskDelay(1);}
        memset( response, 0, sizeof(response)-1 );


        while( dpm_serial->available() ){ // Clear RX buffer
             dpm_serial->read();
        }

        dpm_serial->println( cmd );
        completed = fetch( response, blocking );
    } while ((!completed) && (++retry < 10) );
    dpm_locked = false;
    return completed ? true : false;
}
bool IRAM_ATTR G_EDM_DPM_DRIVER::fetch( char* response, bool blocking ){
    bool success = false;
    int length   = 0;
    if( blocking ){
        unsigned long start = millis();
        while( !dpm_serial->available() ){ 
            if( millis() - start > 200 ) break;
        }
    }

    while( dpm_serial->available() ){
        char c = dpm_serial->read();
        if (c == '\n'){
            success= true;
            break;
        } else { 
            response[length] = c;
            if(++length>=BUFFER_SIZE){
                dpm_serial->flush();
                break;
            }
        }
    }

    while(  dpm_serial->available() ){ // Clear RX buffer
         dpm_serial->read();
    }


    //debuglog( response, DEBUG_LOG_DELAY );
    return success;
}


//##############################################################
// this only returns the voltage and current that is configured 
// in the setting and not the realtime measurement 
// values are in mV and mA (integer)
//##############################################################
bool G_EDM_DPM_DRIVER::get_setting_voltage_and_current( int &voltage, int &current ){
    //bool success = send( ":01r10=0," );
    bool success = get_setting_voltage( voltage );
    if( success ){
        success = get_setting_current( current );
    }
    if( ! success ){
        voltage = -1;
        current = -1;
    }
    return success;
}









bool IRAM_ATTR G_EDM_DPM_DRIVER::extract_ok( char* response ){
    bool ok = false;
    for( int i = 0; i < BUFFER_SIZE; ++i ){
        if( i >= (BUFFER_SIZE-2) ){
            break;
        }
        if( response[i] == 'o' && response[i+1] == 'k' ){
            ok = true;
            break;
        }
    }
    return ok;
}
int IRAM_ATTR G_EDM_DPM_DRIVER::extract( int &value, int multiplier, char* response ){
    bool start = false;
    value      = 0;
    bool has_point = false;
    for( int i = 5; i < BUFFER_SIZE; ++i ){
        char c = response[i];
        //“,” or “.” or “,,”
        if( c == '.' || c == ',' ){ //  || c == ',,' not handled yet
            has_point = true;
            break;
        } else if( !start && c == '=' ){
            start = true;
        } else if( start && isdigit(c) ){
            if( ! value ){
                value = c - '0';
            } else {
                value *= 10;
                value += c - '0';
            }
        } else if( start && !has_point && !isdigit(c) ){
            // garbage?
            break;
        }
    }
    if( !start || !has_point ){
        //error
        value = -1;
    } else {
        if( multiplier > 0 ){
            value *= multiplier;// mV and mA
        } else if( multiplier < 0 ){
            value /= multiplier;
        }
    }
    return 1; 
}



int IRAM_ATTR G_EDM_DPM_DRIVER::backup_current_settings( void ){
    // read and store the current dpm settings for voltage and current
    bool success = dpm_driver.get_setting_voltage_and_current( voltage_backup_mv, current_backup_ma ); // return values are in mV/mA
    return success?1:0;
}

int IRAM_ATTR G_EDM_DPM_DRIVER::restore_backup_settings( void ){

    debuglog( "Restore DPM settings", DEBUG_LOG_DELAY );

    voltage_backup = float(voltage_backup_mv);
    current_backup = float(current_backup_ma);
    if( voltage_backup_mv > -1 ){
      voltage_backup /= 1000.0;
    }
    if( current_backup_ma > -1 ){
      current_backup /= 1000.0;
    }

    bool success = false;
    success = dpm_driver.set_setting_voltage( voltage_backup ); // set voltage in float Volts
    success = dpm_driver.set_setting_current( current_backup ); // set current in float Amp

    debuglog( "   Done", DEBUG_LOG_DELAY );

    return success?1:0;

}
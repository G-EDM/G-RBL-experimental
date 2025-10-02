#pragma once
//############################################################################################################
//
//  ██████        ███████ ██████  ███    ███  
// ██             ██      ██   ██ ████  ████  
// ██   ███ █████ █████   ██   ██ ██ ████ ██ 
// ██    ██       ██      ██   ██ ██  ██  ██ 
//  ██████        ███████ ██████  ██      ██ 
//  ____  _                 _                  _   _ ___ 
// |  _ \| |__   __ _ _ __ | |_ ___  _ __ ___ | | | |_ _|
// | |_) | '_ \ / _` | '_ \| __/ _ \| '_ ` _ \| | | || | 
// |  __/| | | | (_| | | | | || (_) | | | | | | |_| || | 
// |_|   |_| |_|\__,_|_| |_|\__\___/|_| |_| |_|\___/|___|
//   
//
// This is a beta version for testing purposes.
// Only for personal use. Commercial use or redistribution without permission is prohibited. 
// Copyright (c) Roland Lautensack    
//
//############################################################################################################

#ifndef PHANTOM_UI_INTERFACE
#define PHANTOM_UI_INTERFACE


#include <sstream>
#include <iostream>
#include <vector>
#include <functional>
#include <unordered_map>
#include <map>
#include <HardwareSerial.h>
#include <memory>
#include <list>
#include <algorithm>
#include <unordered_set>
#include "sd_card.h"
#include "widgets/phantom_page.h"
#include "widgets/widget_vertical_button_list.h"
#include "widgets/widget_button.h"
#include "widgets/widget_list.h"
#include "widgets/widget_split_list.h"
#include "widgets/widget_logger.h"
#include "widgets/widget_files.h"

#define MAIN_PANEL_WIDTH 70
#define CALDATA_SIZE 5

static const uint8_t page_tree_depth = 20;

enum sd_write_action_type {
    SD_LOAD_SETTINGS = 0, // create a custom name
    SD_SAVE_AUTONAME = 1, // auto generate name
    SD_AUTOSAVE      = 2  // save as last_settings
};

extern DRAM_ATTR std::atomic<bool> reload_waiting;
extern DRAM_ATTR std::atomic<bool> logger_busy_writing;
extern DRAM_ATTR std::atomic<bool> logger_show_on_line;
extern DRAM_ATTR std::atomic<bool> logs_available;

extern std::shared_ptr<PhantomWidget> factory_button(      int16_t w = 0, int16_t h = 0, int16_t x = 0, int16_t y = 0, int8_t t = 0, bool s = false );
extern std::shared_ptr<PhantomWidget> factory_split_list(  int16_t w = 0, int16_t h = 0, int16_t x = 0, int16_t y = 0, int8_t t = 0, bool s = false );
extern std::shared_ptr<PhantomWidget> factory_list(        int16_t w = 0, int16_t h = 0, int16_t x = 0, int16_t y = 0, int8_t t = 0, bool s = false );
extern std::shared_ptr<PhantomWidget> factory_vbuttonlist( int16_t w = 0, int16_t h = 0, int16_t x = 0, int16_t y = 0, int8_t t = 0, bool s = false );
extern std::shared_ptr<PhantomWidget> factory_nkeyboard(   int16_t w = 0, int16_t h = 0, int16_t x = 0, int16_t y = 0, int8_t t = 0, bool s = false );
extern std::shared_ptr<PhantomLogger> factory_logger(      int16_t w = 0, int16_t h = 0, int16_t x = 0, int16_t y = 0, int8_t t = 0, bool s = false );
extern std::shared_ptr<PhantomWidget> factory_akeyboard(   int16_t w, int16_t h, int16_t x, int16_t y, int8_t t, bool s );

extern void charge_logger( const char *line );
extern void show_logger( void );
extern void fill_screen( int color );



class PhantomUI : public EventManager {

    public:

        PhantomUI( void );

        info_codes msg_id;

        std::string settings_file;
        std::string gcode_file;
        bool load_settings_file_next;
        bool deep_check;
        int  sd_card_state;
        bool loading_page;
        bool refresh_sd_disabled;

        std::shared_ptr<PhantomLogger> logger;
        std::shared_ptr<PhantomWidgetItem> get_item( ItemIdentPackage &package );
        std::shared_ptr<PhantomWidget>     get_widget_by_item( PhantomWidgetItem * item );
        std::shared_ptr<PhantomWidget>     get_widget( uint8_t page_id, uint8_t page_tab_id, uint8_t widget_id, uint8_t item_id );
        std::shared_ptr<PhantomPage>       addPage( uint8_t page_id, uint8_t num_tabs );
        std::shared_ptr<PhantomPage>       get_page( uint8_t page_id );
        std::map<std::uint8_t, std::shared_ptr<PhantomPage>> pages;

        bool save_settings_writeout( std::string file_name );

        uint8_t next_page_to_load;
        bool    load_next_page;
        bool    save_param_next;
        int16_t save_param_id;
        void show_page_next( uint8_t page_id );

        IRAM_ATTR void monitor_touch( void );
        IRAM_ATTR bool touch_within_bounds(int x, int y, int xpos, int ypos, int w, int h );
        IRAM_ATTR void track_touch( void );
        IRAM_ATTR void process_update( void );
        IRAM_ATTR void process_update_sd( void );
        IRAM_ATTR void wpos_update( void );
        IRAM_ATTR void reload_page_on_change( void );

        bool save_settings( sd_write_action_type action = SD_AUTOSAVE ); 
        void set_gcode_file( const char* file_path );
        void set_settings_file( const char* file_path );
        bool delete_nvs_storage( void );
        bool firmware_is_recent( void );
        bool firmware_is_recent_set_flag( void );
        std::string get_firmware_key( void );

    
        void    emit_param_set(int param_id, const char* value);
        void    lock_touch( void );
        void    unlock_touch( void );
        void    show_page( uint8_t page );
        bool    get_touch( uint16_t *x, uint16_t *y );
        uint8_t get_previous_active_page( void );
        void    build_phantom( void );
        void    begin( void );
        void    close_active( void );
        void    reload_page( void );
        void    get_has_sd_card( void );
        void    get_has_motion( void );
        void    init( void );
        bool    prepare( void );
        void    restart_display( void );
        void    loader_screen( bool fill_screen = true );
        bool    get_motion_is_active( void );
        void    set_motion_is_active( bool state );
        void    sd_refresh( void );
        void    update_active_tabs( uint8_t page_id, uint8_t tab );
        uint8_t get_active_page( void );
        uint8_t get_active_tab_for_page( uint8_t page_id );
        bool    restore_last_session( void );
        bool    save_current_to_nvs( int param_id = -1 );
        bool    start( void );
        bool    touch_calibrate( bool force );
        void    load_settings( std::string path_full );
        void    set_scope_and_logger( uint8_t page_id );
        void    exit( void );
        bool    get_is_ready( void );
        void    set_is_ready( void );

    private:

        void write_calibration_data( void );

        uint16_t calibration_data[5];
        uint8_t  page_tree[page_tree_depth];
        uint8_t  active_tabs[TOTAL_PAGES];
        uint8_t  page_tree_index;
        bool     motion_is_active;
        bool     sd_card_is_available;
        uint8_t  active_page;
        uint8_t  previous_active_page;
        uint8_t  active_tab;

};

extern PhantomUI ui;


#endif
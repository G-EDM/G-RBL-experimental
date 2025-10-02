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

#ifndef PHANTOM_UI_WIDGET_BTN
#define PHANTOM_UI_WIDGET_BTN

#include "widget_base.h"

class PhantomButton : public PhantomWidget {
    public:
        PhantomButton( 
            int16_t w = 0, 
            int16_t h = 0, 
            int16_t x = 0, 
            int16_t y = 0, 
            int8_t type = 0, 
            bool _use_sprite = false 
        ) : PhantomWidget(w,h,x,y,type,_use_sprite ){};
        virtual ~PhantomButton(){};
        void create( void )                   override;
        void show( void )                     override;
        IRAM_ATTR void redraw_item( int16_t item_index, bool reload = true ) override;
        IRAM_ATTR void draw_button_item( TFT_eSPI* display, int16_t item_index, bool reload = false, int16_t screen_x_offset = 0, int16_t screen_y_offset = 0 );

};

#endif
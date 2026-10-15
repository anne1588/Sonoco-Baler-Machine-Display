void my_touchpad_read( lv_indev_drv_t * indev_drv, lv_indev_data_t * data )
{
    if (ft6336u.read_touch1_event() > 1)  // CAP TOUCH
    {                                     // CAP TOUCH
        data->state = LV_INDEV_STATE_PR;
        x = ft6336u.read_touch1_x();          // CAP TOUCH
        y = ft6336u.read_touch1_y();          // CAP TOUCH
                       

        /*Set the coordinates*/
        data->point.x = x;
        data->point.y = y;

        Serial.print( "Data x " );
        Serial.println( x);

        Serial.print( "Data y " );
        Serial.println( y );

        /* NOTE : UN-DIM SCREEN */
        analogWrite(BL, 255);

    }
    
}

# NodeMCU Arduino based RTC syncronizer

This sketch is to update a DS1307, etc. RTC to current NTP time.

Optional accessory is an i2c LCD display to notify the user.

## Wiring

```
i2c RTC - Nodemcu
-----------------
    SCL - D1
    SDL - D2
    VCC - D3
    GND - D4
    
i2c LCD - Nodemcu
-----------------
    VCC - 3V3
    GND - GND
    SDL - D2
    SCL - D1
    
 To set clock short D8 to 3V3
```

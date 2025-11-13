// config.h
#ifndef CONFIG_H
#define CONFIG_H

#define RPM_DISPLAY_MIN 0      // Minimum RPM value that can be shown on the slider.

#define RPM_DISPLAY_MAX 20000   // Maximum RPM value that can be shown on the slider.

#define RPM_MIN (int)(RPM_DISPLAY_MAX * 0.15)              // Starts flashing when RPM drops *below* this value (shift down).

#define RPM_MAX (int)(RPM_DISPLAY_MAX * 0.85)            // Starts flashing when RPM exceeds *above* this value (shift up).

#define TEMP_MAX 100           // Startsts flashing when oil temperature rises *above* this value. Celcius.

#define PRESSURE_MIN 60        // Starts flashing when oil pressure falls *below* this value. kPa.

#define VOLTAGE_MIN 12         // Starts flashing when voltage drops *below* this value.

/*
If you have any issues, please contact Ethan Maya at ethanmayaemail@gmail.com or if it's urgent, preferably whatsapp me or give me a ring on 07598 413640.
*/

#endif // CONFIG_H

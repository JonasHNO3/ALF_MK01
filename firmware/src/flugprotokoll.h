/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _FLUGPROTOKOLL_H    /* Guard against multiple inclusion */
#define _FLUGPROTOKOLL_H

#include <stdint.h>

//this function creates any milliseconds delay
//useful to make sure that a function has been completed 
//or to introduce a delay in the program flow.
void delay_ms(uint16_t ms);

//this function creates any microseconds delay
//useful to make sure that a function has been completed 
//or to introduce a delay in the program flow.
void delay_us(uint16_t us);

//this function resets all necessary variables for a restart flight process
//and also to start a new process
void change_flugprozess_variable(void);

//To change the format from degrees minutes to degrees
//The gps sends the positions in the degrees minutes format
//so this has to be changed to degrees system
double change_degree_minutes_to_degree(double tochange);

//this function change a value in radians to a value in degrees
//this is useful for the courseTO and distance function
double radians_to_degrees(double rad);

//this function change a value in degrees to a value in radians
//this is useful for the courseTO and distance function
double degrees_to_radians(double degrees);

//this function calculates the distance between the start position and the end
//position in meters.
double distance(double start_lat, double start_lon, double end_lat, double end_lon);

//this function calculates the angle for the cardinal direction in which
//cardinal direction the drone has to fly from the start position to the
//end position. The unit is in degrees
double courseTO(double lat1, double lon1, double lat2, double lon2);

//this is a datatype of 4 variables with the name
//single_satelite_data
//this is to safe for each connected satellite their data
typedef struct {
    int amount;
    int elevation;
    int azimuth;
    int signal_strength;
}single_satelite_data;

//this function calculates the strenght of the signal of the gps modul
//to the satellites in dB. This is necessary because to calculate the
//cardinal direction, it would be better to use the data from the satellites
//with the strongest connection
int split_satelites_data(char* data, single_satelite_data* satelites);

//this function calculates the cardinal direction of the drone
//it will return a double in the unit degrees
double compass_direction(single_satelite_data* satelites, int amountSatelites);

//this function is for the calculation of the current high of the drone and has
//to be a double be more precise. The unit is in meters
double read_current_altitude(void);

//this function calculates the current distance of the drone to the end position
//unit of this is in meters and has to be a double because to be more precise
double read_current_distance(void);

//this function will return the current latitude in degrees
double read_current_latitude(void);

//this function will return the current longitude in degrees
double read_current_longitude(void);

//function to create the message which will be send over uart at SERCOM0
//to the flight controller with the values of the roll, pitch, yaw and throttle
void write_flight_controller(double roll, double pitch, double yaw, 
        double throttle);

//this function is for the end process and will be called when the flight
//process is ready for the reset
void end_of_flight_process(void);

//this function is necessary to be able to get the state of the
//flight_process boolean
bool get_fly_process();

//this function is necessary to be able to change the fly_process
//boolean
void set_fly_process(bool set);

//this function controlls the full fly protocol and the setup
void fly_process(void);


#endif /* _FLUGPROTOKOLL_H */

/* *****************************************************************************
 End of File
 */

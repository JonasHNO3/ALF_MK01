/* ************************************************************************** */
/** flugprotokoll

  @Company
    Schindelar

  @File Name
    flugprotokoll.c

  @Summary
    This class controlls the whole fly, safety and setup process
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "definitions.h"
#include "flugprotokoll.h"

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Definitions Area                                                  */
/* ************************************************************************** */
/* ************************************************************************** */

#define TWO_PI 2*M_PI   //defines two pi for the calculation
#define latitude_distance 111.19494     //defines the exact distance between two latitudes
#define one_degree_in_radians 0.01745   //defines one degree in radians 
#define delta_limited_high 3    //defines the maximum distancen for the altitude as deviation from the the start altitude high
#define max_weight 3.0    //this is the max weight which the load cell allows
#define magnetic_declination 5.05   //define the magnetic declination of the HTL

//this defines a tolerance range for the cardinal direction
#define compass_tolerance 2.0

//define for each roll, pitch, yaw and throttle value the middle, max, min value
//the whole flight process does not need the roll so roll is always at 1500
#define roll_value 1500

#define pitch_middle_value 1500
#define pitch_max_value 1750
#define pitch_min_value 1250

#define yaw_middle_value 1500
#define yaw_max_value 1750
#define yaw_min_value 1250

#define throttle_middle_value 1500
#define throttle_max_value 1750
#define throttle_min_value 1250

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Variables Area                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

//ALL CALLED SERCOM, TC FUNCTIONS are functions of the PIC Libraries

//this doubles are for the value which will be set in the setup process
double start_lat, start_lon, end_lat, end_lon, altitude_start_position;
double himmelsrichtung, entfernung;
double payload = 0;     //define a double variable as payload

double azimuth = 0;

//Create a double variable for the return at read_distance

//for the switch case function the variable will define the current state
//in the fly process
char process_state = 0;

//this variable is for the switch case function of the setup process
char setup_state = 0;

//This variable is for the switch case function of the setup process at the back
//flight
char backflight_setup_state = 0;

//this variable is for the switch case function of the takeoff process
char takeoff_process = 0;

//this variable is for the switch case funtion of the backflight takeoff process
char backflight_takeoff_process = 0;

//this variables define how the uart messages has to start, so the program knows
//which message it has to edit during the fly and setup process
const char* coords_prefix = "$COORDS"; 
const char* gps_prefix = "$GNGGA,";
const char* satelite_prefix = "$GPGSV";

//will be set everytime when a gps message comes in and is required
//because is the value less than 8 the coords will not be exactly
int satelites_connected = 0;

//this messages are strings for the incoming messages from all uarts
uint8_t receive_bt[250] = "";
uint8_t receive_gps[250] = "";
uint8_t receive_load_cell[250] = "";

//message to send to the bluetooth modul to tell the user,
//that the weight is too heavy
uint8_t message_overweight[100] = "Das Gewicht ist zu schwer!";

//message to send to the bluetooth modul to tell the user,
//that the setup is finished and ready for the flight
uint8_t message_ready_to_start[100] = "Die Drohne ist bereit zum abfliegen!";

//message to send to the bluetooth modul to tell the user,
//that the flight is starting now
uint8_t message_fly_starts[100] = "Der Flug startet jetzt!";

//tihs variable is to fill in the roll, pitch, yaw and throttle value
//which will be send over uart to the flight controller
uint8_t message_to_fly_controller[100] = "";

//message for the comparison with the income bluetooth start message and the 
//start signal to be sure it is the right message to start
uint8_t start_signal[100] = "$FLYSTART";

//the empty_string variable is for the comparison with the income bluetooth 
//message over uart.
//this is necessary, because the uart from bluetooth does not send a message
//the whole time, just when the user send something over with the phone.
uint8_t empty_string[250] = "";

//this variable is required at each new setup run to exit the setup
bool setup_complete = false;

//this variable is required for the main class to know if there is a flight 
//process or not
bool flight_process = false;

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: extra functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */

//this function creates any milliseconds delay
//useful to make sure that a function has been completed 
//or to introduce a delay in the program flow.
void delay_ms(uint16_t ms) {
    uint16_t t;
    for(t=0;t<ms;t++) {
        TC0_TimerStart();
        while(!TC0_TimerPeriodHasExpired());
        TC0_TimerStop();
    }
}

//this function creates any microseconds delay
//useful to make sure that a function has been completed 
//or to introduce a delay in the program flow.
void delay_us(uint16_t us) {
    uint16_t t;
    for(t=0;t<us;t++) {
        TC1_TimerStart();
        while(!TC1_TimerPeriodHasExpired());
        TC1_TimerStop();
    }
}

//this function resets all necessary variables for a restart flight process
//and also to start a new process
void change_flugprozess_variable(void) {
    
    //Change some variables for a restart
    process_state = 0;
    setup_state = 0;
    satelites_connected = 0;
    payload = 0;
    takeoff_process = 0;
    backflight_setup_state = 0;
    backflight_takeoff_process = 0;
    
    //memset is there to set at every position of the string a 0 to clear it
    memset(receive_bt, 0, sizeof(receive_bt));
    memset(receive_gps, 0, sizeof(receive_gps));
    memset(receive_load_cell, 0, sizeof(receive_load_cell));
    memset(message_to_fly_controller, 0, sizeof(message_to_fly_controller));
    
    //reset the booleans to false
    setup_complete = false;
    flight_process = false;
}

//To change the format from degrees minutes to degrees
//The gps sends the positions in the degrees minutes format
//so this has to be changed to degrees system
double change_degree_minutes_to_degree(double tochange) {
    double split = tochange;
    double minutes;
    int degrees = (int)split/100;
    minutes = split - degrees*100;
    double solution = (minutes/60) + degrees;
    return solution;
}

//this function change a value in radians to a value in degrees
//this is useful for the courseTO and distance function
double radians_to_degrees(double rad) {     //Rad zu Grad
    return (rad * 180.0) / M_PI;
}

//this function change a value in degrees to a value in radians
//this is useful for the courseTO and distance function
double degrees_to_radians(double degrees) { //Grad zu Rad
    return ( degrees * M_PI) / 180;
}

//this function calculates the distance between the start position and the end
//position in meters.
double distance(double start_lat, double start_lon, double end_lat, double end_lon) {
    double latitude_mean = (start_lat + end_lat) / 2 * one_degree_in_radians; //returns a radiant value and not degree
    double dx = latitude_distance * cos(latitude_mean) * (start_lon - end_lon); //changing value of longitude
    double dy = latitude_distance * (start_lat - end_lat);      //changing value of latitude
    double distance = (sqrt(dx * dx + dy * dy)) * 1000;     //distance in meter and no negativ distance
    
    return distance;    //return the double distance 
}

//this function calculates the angle for the cardinal direction in which
//cardinal direction the drone has to fly from the start position to the
//end position. The unit is in degrees
double courseTO(double lat1, double lon1, double lat2, double lon2) {   //calculate the compass direction
    double dlon = degrees_to_radians(lon2-lon1);   //Calc the differenz longitude in radians
    lat1 = degrees_to_radians(lat1);               //Latitudes in radians calculation
    lat2 = degrees_to_radians(lat2);
    double tangency = sin(dlon) * cos(lat2);  //get the tangency 
    double partial = sin(lat1) * cos(lat2) * cos(dlon);  //calculate the partial value to 
    partial = cos(lat1) * sin(lat2) - partial;
    double direction_radiant = atan2(tangency, partial);    //calculat direction in radians
    if(direction_radiant < 0.0) {
        direction_radiant += TWO_PI;
    }
    return radians_to_degrees(direction_radiant);     //returns in which direction the drone has to fly
}

//this is a datatype of 4 variables with the name
//single_satelite_data
//this is to safe for each connected satellite their data
//typedef struct {
   // int amount;
   // int elevation;
   // int azimuth;
   // int signal_strength;
//}single_satelite_data;

//this function calculates the strenght of the signal of the gps modul
//to the satellites in dB. This is necessary because to calculate the
//cardinal direction, it would be better to use the data from the satellites
//with the strongest connection
int split_satelites_data(char* data, single_satelite_data* satelites) {
    int amountSatelites = 0;
    char* key;
    char* saveing;
    
    key = strtok_r(data,",", &saveing);
    if((strcmp(key, satelite_prefix) != 0) || key == NULL)
        return 0;
    
    key = strtok_r(NULL, ",", &saveing);
    if(key == NULL)
        return 0;
    
    amountSatelites = atoi(key);
    if(amountSatelites > 12)
        amountSatelites = 12;
    
    for(int i = 0; i < amountSatelites; i++) {
        key = strtok_r(NULL, ",", &saveing);
        if(key == NULL)
            return 0;
        
        satelites[i].amount = atoi(key);
        key = strtok_r(NULL, ",", &saveing);
        if(key == NULL)
            return 0;
        
        satelites[i].elevation = atoi(key);
        key = strtok_r(NULL, ",", &saveing);
        if(key == NULL)
            return 0;
        
        satelites[i].azimuth = atoi(key);
        key = strtok_r(NULL, ",", &saveing);
        if(key == NULL)
            return 0;
        
        satelites[i].signal_strength = atoi(key);
    }
    return amountSatelites;
}

//this function calculates the cardinal direction of the drone
//it will return a double in the unit degrees
double compass_direction(single_satelite_data* satelites, int amountSatelites) {
    double full_aszi = 0;
    double full_signal = 0;
    int used_satelites = 0;
    for(int i = 0; i < amountSatelites; i++) {
        if(satelites[i].signal_strength >= 10) {
            full_aszi += satelites[i].azimuth * pow(10, 
                    satelites[i].signal_strength / 10.0);
            full_signal += pow(10, satelites[i].signal_strength / 10.0);
            used_satelites++;
        }
    }
    
    if(used_satelites == 0)
        return -1;
    
    double azimuth = full_aszi / full_signal;
    azimuth = fmod(azimuth+360, 360); //change the azimuth range to 0 to 360
    azimuth += magnetic_declination; //for the HTL add the magnetic declination
    azimuth = fmod(azimuth+360, 360); //change the azimuth range to 0 to 360  
    
    return azimuth;
}

//this function is for the calculation of the current high of the drone and has
//to be a double be more precise. The unit is in meters
double read_current_altitude(void) {
    //create a double variable for the return at read_current_altitude
    double current_altitude;
    
    //$GNGGA,100855.00,4804.43802,N,01617.42769,E,1,12,0.96,259.8,M,42.1,M,,*4C
    
    //Read out the received data from gps as long as the message does not
    //start with the gps prefix
    while(memcmp(receive_gps, gps_prefix, strlen(gps_prefix)) != 0) {
        //Set the receiving data value back to 0 to make it free
        memset(receive_gps, 0, sizeof(receive_gps));
        //read the uart received data
        SERCOM3_USART_Read(receive_gps, sizeof(receive_gps));
    }
    
    //this dummy values are placeholders to be able to split the received
    //string into its variables
    char dummy_word1[20] = "";
    char dummy_word2[5] = "";
    char dummy_word3[5] = "";
    double dummy_double1 = 0;
    double dummy_double2 = 0;
    double dummy_double3 = 0;
    double dummy_double4 = 0;
    int dummy_int1 = 0;
    int dummy_int2 = 0;
    
    
   //split the received string into its dummy variables and the current altitude
    sscanf((const char*)receive_gps, "%s,%lf,%lf,%s,%lf,%s,%d,%d,%lf,%lf", 
            dummy_word1, &dummy_double1,&dummy_double2, dummy_word2, 
            &dummy_double3, dummy_word3, &dummy_int1, &dummy_int2, 
            &dummy_double4, &current_altitude);
    
    //return the value of the double current_altitude
    return current_altitude;
}

//this function calculates the current distance of the drone to the end position
//unit of this is in meters and has to be a double because to be more precise
double read_current_distance(void) {
    //create new variables for the calculation of the current_distance
    double current_distance, current_latitude, current_longitude;
    
    //$GNGGA,100855.00,4804.43802,N,01617.42769,E,1,12,0.96,259.8,M,42.1,M,,*4C
    
    //Read out the received data from gps as long as the message does not
    //start with the gps prefix
    while(memcmp(receive_gps, gps_prefix, strlen(gps_prefix)) != 0) {
        //Set the receiving data value back to 0 to make it free
        memset(receive_gps, 0, sizeof(receive_gps));
        //read the uart received data
        SERCOM3_USART_Read(receive_gps, sizeof(receive_gps));
    }
    
    //this dummy values are placeholders to be able to split the received
    //string into its variables
    char dummy_word1[20] = "";
    char dummy_word2[5] = "";
    char dummy_word3[5] = "";
    double dummy_double1 = 0;
    
    //split the received string into 
    //its dummy variables and the current latitude and longitude
    sscanf((const char*)receive_gps, "%s,%lf,%lf,%s,%lf,%s",
            dummy_word1, &dummy_double1, &current_latitude, dummy_word2,
            &current_longitude, dummy_word3);
    
    //change the value from "degrees, minute system" to "dezimaldegree system"
    current_latitude = change_degree_minutes_to_degree(current_latitude);
    current_longitude = change_degree_minutes_to_degree(current_longitude);
    
    //use the distance function from above to calculate the distance between
    //the current position and the end positon
    current_distance = distance(current_latitude, current_longitude,
            end_lat, end_lon);
    
    //return the current distance to the end coordinates
    return current_distance;
}

//this function will return the current latitude in degrees
double read_current_latitude(void) {
    //create a variable for the return
    double current_latitude;
    
    //$GNGGA,100855.00,4804.43802,N,01617.42769,E,1,12,0.96,259.8,M,42.1,M,,*4C
    
    //Read out the received data from gps as long as the message does not
    //start with the gps prefix
    while(memcmp(receive_gps, gps_prefix, strlen(gps_prefix)) != 0) {
        //Set the receiving data value back to 0 to make it free
        memset(receive_gps, 0, sizeof(receive_gps));
        //read the uart received data
        SERCOM3_USART_Read(receive_gps, sizeof(receive_gps));
    }
    
    //this dummy values are placeholders to be able to split the received
    //string into its variables
    char dummy_word1[20] = "";
    double dummy_double1 = 0;
    
    //split the received string into 
    //its dummy variables and the current latitude and longitude
    sscanf((const char*)receive_gps, "%s,%lf,%lf",
            dummy_word1, &dummy_double1, &current_latitude);
    
    //change the value from degrees, minute system to dezimaldegree system
    current_latitude = change_degree_minutes_to_degree(current_latitude);
    
    //Return the value of the current latitude variable
    return current_latitude;
}

//this function will return the current longitude in degrees
double read_current_longitude(void) {
    //create a variable for the return
    double current_longitude;
    
    //$GNGGA,100855.00,4804.43802,N,01617.42769,E,1,12,0.96,259.8,M,42.1,M,,*4C
    
    //Read out the received data from gps as long as the message does not
    //start with the gps prefix
    while(memcmp(receive_gps, gps_prefix, strlen(gps_prefix)) != 0) {
        //Set the receiving data value back to 0 to make it free
        memset(receive_gps, 0, sizeof(receive_gps));
        //read the uart received data
        SERCOM3_USART_Read(receive_gps, sizeof(receive_gps));
    }
    
    //this dummy values are placeholders to be able to split the received
    //string into its variables
    char dummy_word1[20] = "";
    char dummy_word2[5] = "";
    double dummy_double1 = 0;
    double dummy_double2 = 0;
    
    //split the received string into 
    //its dummy variables and the current latitude and longitude
    sscanf((const char*)receive_gps, "%s,%lf,%lf,%s,%lf",
            dummy_word1, &dummy_double1, &dummy_double2, dummy_word2,
            &current_longitude);
    
    //change the value from degrees, minute system to dezimaldegree system
    current_longitude = change_degree_minutes_to_degree(current_longitude);
    
    //return the value of the current_longitude variable
    return current_longitude;
}

//function to create the message which will be send over uart at SERCOM1
//to the flight controller with the values of the roll, pitch, yaw and throttle
void write_flight_controller(double roll, double pitch, double yaw, 
        double throttle) {
    
    //put the double variables into a string
    sprintf((char*)message_to_fly_controller, "%lf %lf %lf %lf", roll, pitch, yaw, throttle);
    
    //write the data of the message to over uart to the flight controller
    SERCOM1_USART_Write(message_to_fly_controller, 
            sizeof(message_to_fly_controller));
}

//this function is for the end process and will be called when the flight
//process is ready for the reset
void end_of_flight_process(void) {
    //Set all necessary variables for the flight process to 0
    change_flugprozess_variable();
}

//this function is necessary to be able to get the state of the
//flight_process boolean
bool get_fly_process() {
    return flight_process;
}

//this function is necessary to be able to change the fly_process
//boolean
void set_fly_process(bool set) {
    flight_process = set;
}

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Main Fly process area                                             */
/* ************************************************************************** */
/* ************************************************************************** */

//this function controlls the full fly protocol and the setup
void fly_process(void) {
    switch(process_state) { //proces for the full fly process
        case(0): {      //setup process
            if(setup_complete) {    //if the setup is complete
                process_state = 1;  //switch to the next state
            } else {
            
            switch(setup_state) {   //For the Setup all Steps
                
                case(0): {          //Case for Bluetooth Coords receiving
                    
                    controll_LED_Set();
                    
                    //While there is no receiving data from sercom2
                    while(memcmp(receive_bt, empty_string, 250) == 0) {
                        //read the uart received data from sercom2 
                        SERCOM2_USART_Read(receive_bt, sizeof(receive_bt));
                    }
                    //delay of 500ms to be sure data could be finished reading
                    delay_ms(500);
                    
                    //Coords will look like: $COORDS 000.00000 000.00000
                    //check if the message starts with the coords prefix
                    if(memcmp(receive_bt, coords_prefix
                            , strlen(coords_prefix)) == 0) {
                        //create a variable for the coods prefix COORDS
                        char dummy_data[100];
                        //split up the coords into doubles and a dummy part
                        sscanf((const char*)receive_bt, "%s %lf %lf", 
                                dummy_data, &end_lat, &end_lon);
                        //delay of 50ms 
                        delay_ms(50);
                        //reset dummy value
                        memset(dummy_data, 0, sizeof(dummy_data));
                        //step to the next setup process
                        setup_state = 1;
                    }
                    //Reset String if it was false or finish with reading
                    memset(receive_bt, 0, sizeof(receive_bt));
                    //end this case
                    break;
                }
                
                case(1): {     //Case for getting first time the start position
                    //while receiving message from gps is empty
                    while(memcmp(receive_gps, empty_string, 250) == 0) {
                        //read the uart received data
                        SERCOM3_USART_Read(receive_gps, sizeof(receive_gps));
                    }
                    //while the message has not the gps prefix
                    while(memcmp(receive_gps, gps_prefix, strlen(gps_prefix))
                            != 0) {
                        //Set the receiving data value back to 0 to make it free
                        memset(receive_gps, 0, sizeof(receive_gps));
                        //read the uart received data
                        SERCOM3_USART_Read(receive_gps, sizeof(receive_gps));
                    }
                    //delay of 500ms to be sure that all data could be finished
                    delay_ms(500);
                    
//$GNGGA,100855.00,4804.43802,N,01617.42769,E,1,12,0.96,259.8,M,42.1,M,,*4C
                    //placeholder just for upper example the prefix GNGGA
                    char dummy_data[100];
                    //placeholder just for upper example N
                    char dummy_word1[10];
                    //placeholder just for upper example E
                    char dummy_word2[10];
                    //placeholder just for upper example 100855.00
                    float dummy_time;
                    //placeholder just for upper example 0.96
                    float dummy_float1;
                    //placeholder just for upper example 1 
                    int dummy_int1;
                    //split all variables with a form schematic
                    sscanf((const char*)receive_gps, 
                            "%s,%f,%lf,%s,%lf,%s,%d,%d,%f,%lf", dummy_data, 
                            &dummy_time, &start_lat, dummy_word1, &start_lon,
                            dummy_word2, &dummy_int1, &satelites_connected,
                            &dummy_float1, &altitude_start_position);
                    //delay of 50 ms to be sure everything could be done
                    delay_ms(50);
                    
                    //change start latitude to the degrees system
                    start_lat = change_degree_minutes_to_degree(start_lat);
                    //change start longitude to the degrees system
                    start_lon = change_degree_minutes_to_degree(start_lon);
                    
                    //set the value back to 0 to clear it
                    memset(receive_gps, 0, sizeof(receive_gps));
                    
                    //when more than 8 satelites are connected then the program
                    //can move to the next state
                    //This safety query is required
                    //because the gps module is not exactly when there less
                    //than 8 satelites are connected
                    if(satelites_connected >= 8) {
                        //step to the next state
                        setup_state = 2;
                    }
                    
                    //end this case
                    break;
                }
                
                case(2): { //Case for calculating the distance and the direction
                    
                    //Calculate the direction with nord is 0 and west is 270
                    himmelsrichtung = courseTO(start_lat, start_lon,
                            end_lat, end_lon);
                    
                    //Calculate the distance between this two coords
                    entfernung = distance(start_lat, start_lon,
                            end_lat, end_lon);
                    
                    //step to the next process
                    setup_state = 3;
                    //end this case
                    break;
                }
                
                case(3): {  //Case to check the weight is not more than the max
                    //while the payload is null or to heavy 
                    while(payload == 0 || payload > max_weight) {
                        
                        //read the uart message from sercom0 to get the
                        //load cell weight
                        SERCOM0_I2C_Read(711, receive_load_cell, 
                                sizeof(receive_load_cell));
                        
                        //transfer the value of the load cell from the 
                        //receive_load_cell string to the double variable
                        sscanf((const char*)receive_load_cell, "%lf", &payload);
                        
                        //delay of 50ms to be sure that the variable has been
                        //overwritten correctly
                        delay_ms(50);
                        
                        //if the load cell weight is higher than the max weight
                        if(payload > max_weight) {
                            
                            //write over uart to the bluetooth modul, that the
                            //weight is to heavy
                            SERCOM2_USART_Write(message_overweight,
                                    sizeof(message_overweight));   
                            delay_ms(3000); //delay of 3 seconds
                        } else {
                            setup_state = 4;    //step to the next process
                            
                            //write over uart to the bluetooth modul, that the
                            //drone is ready for the flight
                            SERCOM2_USART_Write(message_ready_to_start,
                                    sizeof(message_ready_to_start));
                            delay_ms(50);   //delay of 50ms
                        }
                    }
                    
                    //end this case
                    break;
                }
                
                case(4): {  //case to calculate the direction of the compass
                    
                    single_satelite_data satelites[12];
                    int amoutSatelites = 0;
                    char buffer[128];
                    
                    for(int i = 0; i< 12; i++) {
                        while(memcmp(receive_gps, satelite_prefix, 
                                strlen(satelite_prefix)) != 0) {
                            SERCOM3_USART_Read(receive_gps, 
                                    sizeof(receive_gps));
                        }
                        
                        amoutSatelites = split_satelites_data(buffer, satelites);
                        if(amoutSatelites > 0) {
                            azimuth = compass_direction(satelites, 
                                    amoutSatelites);
                        }
                    }
                    
                    if(amoutSatelites > 0) {    //when there are connected sats
                        setup_state = 5;    //move to the next setup state
                    }
                    
                    //end this case
                    break;
                }
                
                case(5): {  //case to wait for the start signal from the user
                    //while the received data from bluetooth over uart
                    //is empty, the microcontroller will continue 
                    //reading the data
                    while(memcmp(receive_bt, empty_string,
                            250) == 0) {
                        SERCOM2_USART_Read(receive_bt, sizeof(receive_bt));
                    }
                    
                    //when the received data contains the start signal
                    //in the message, then the microcontroller
                    //will tell the user that the flightprocess begins
                    if(memcmp(receive_bt, start_signal, sizeof(start_signal)) 
                            == 0) {
                        SERCOM2_USART_Write(message_fly_starts,
                                sizeof(message_fly_starts));
                        delay_ms(50);   //delay of 50ms
                    }
                    
                    //set the boolean setup complete true because the setup
                    //has been finished and the program can move on to the
                    //next flight process
                    setup_complete = true;
                        
                    //Reset String if it was false or finish with reading
                    memset(receive_bt, 0, sizeof(receive_bt));
                    
                    //turn off the LED of the battery state
                    controll_LED_Clear();
                    
                    //end this case
                    break;
                }
            }
            }
            //end this case
            break;
        }
        
        case(1): {  //Takeoff case
            
            //switch case function for the takeoff process to controll 
            //single steps which are necessary for the takeoff
            switch(takeoff_process) {
                case(0): {  //case to fly up on the maximum high
                    
                    //while the current high of the drone is lower than
                    //the max flight high the drone flies up
                    while(read_current_altitude() <= 
                            (altitude_start_position + delta_limited_high)) {
                        
                        //write function for the flight controller
                        //to let the drone fly higher with a middle gas
                        write_flight_controller(roll_value, pitch_middle_value, 
                                yaw_middle_value, throttle_max_value);
                    }
                    //write function for the flight controller
                    //to hold the current positon in the air
                    write_flight_controller(roll_value,pitch_middle_value,
                            yaw_middle_value, throttle_middle_value);
                    
                    //move to the next step of the takeoff process
                    takeoff_process = 1;
                    
                    //end this case
                    break;
                }
                
                case(1): {  //case to rotate the drone in the right direction
                    
                    //while the cardinal direction is not in the direction
                    //where the end position is
                    while(azimuth <= (himmelsrichtung - compass_tolerance) || 
                            azimuth >= (himmelsrichtung + compass_tolerance)) {
                        
                        //to allow the access to the structure
                        single_satelite_data satelites[12];
                        
                        //this variable defines the amount of satellites 
                        //which are connected with the the gps modul
                        int amoutSatelites = 0;
                        char buffer[128];
                        
                        //this for function will run the internal code 12 times
                        for(int i = 0; i< 12; i++) {
                            //read the received gps data from uart as long as
                            //the right gps data are not there
                            while(memcmp(receive_gps, satelite_prefix, 
                                strlen(satelite_prefix)) != 0) {
                                //read the received uart data from SERCOM3 GPS
                                SERCOM3_USART_Read(receive_gps, 
                                        sizeof(receive_gps));
                            }
                            
                            //to get the satellites with the best connection
                            //to the gps modul
                            amoutSatelites = split_satelites_data(buffer, 
                                    satelites);
                            
                            //when the gps modul has connected satellites
                            //then calculate the direction (azimuth) of the gps
                            if(amoutSatelites > 0) {
                                azimuth = compass_direction(satelites, 
                                        amoutSatelites);
                            }
                        }
                        
                        //write function for the flight controller 
                        //to rotate right around in the air
                        write_flight_controller(roll_value, pitch_middle_value, 
                                1600, throttle_middle_value);
                    }
                    
                    //write function for the flight controller
                    //to hold the current positon in the air
                    write_flight_controller(roll_value, pitch_middle_value, 
                            yaw_middle_value, throttle_middle_value);
                    
                    //when the cardinal direction is in the tolerance range
                    if(azimuth > (himmelsrichtung - compass_tolerance) || 
                            azimuth < (himmelsrichtung + compass_tolerance)) {
                        //move on to the next step of the full fligth process
                        process_state = 2;
                    }
                    //end this case
                    break;
                }
            }
            //end this case
            break;
        }
        
        case(2): {  //fly to the endposition case
            
            //while the distance to the end position is bigger than 10meters the
            //drone can fly faster to the position to save energy
            while(read_current_distance() > 10.0) {
                write_flight_controller(roll_value, 1700, yaw_middle_value,
                        1700);
            }
            
            //while the distance is less than 10.0 meters and higher or equal 2
            //meters, the drone should fly slower to the end position
            while(read_current_distance() <= 10.0 
                    && read_current_distance() >= 2.0) {
                write_flight_controller(roll_value, 1600, yaw_middle_value,
                        1600);
            }
            
            //while the distance is less than 2.0 meters and higher or equal 0.2
            //meters, the drone should fly slower to the end position
            //to land more precise at the exat end position
            while(read_current_distance() >= 0.2 
                    && read_current_distance() <= 2.0) {
                write_flight_controller(roll_value, 1550, yaw_middle_value,
                        1550);
            }
            
            //when the drone is in a distance of the tolerance range of 0.2
            //than stop in the air hold the position and move on to the 
            //next flight process
            if(read_current_distance() < 0.2) {
                write_flight_controller(roll_value, 1500, yaw_middle_value,
                        1500);
                
                //move to the next step of the flight process
                process_state = 3;
            }
            
            //end this case
            break;
        }
        
        case(3): {  //case for the landing process
            
            //while the drone is landing and its high is higher than
            //1 meter the drone can land faster than when the high is less than
            //1 meter
            while(read_current_altitude() > (altitude_start_position + 1)) {
                write_flight_controller(roll_value, pitch_middle_value,
                        yaw_middle_value, 1300);
            }
            
            //while the drone is landing and its high is less than 1 meter
            //the speed of landing has to be slower
            while(read_current_altitude() <= (altitude_start_position + 1)) {
                write_flight_controller(roll_value, pitch_middle_value,
                        yaw_middle_value, 1400);
            }
            
            //when the drone is 5cm or less away from the ground the drone
            //set the throttle variable to the minimum start speed
            if(read_current_altitude() < (altitude_start_position + 0.05)) {
                write_flight_controller(roll_value, pitch_middle_value,
                        yaw_middle_value, throttle_min_value);
                
                //Wait 2 minutes to take the package out of the box
                delay_ms(60000);
                delay_ms(60000);
                //move to the next flight process step
                process_state = 4;
            }
            
            //end this case
            break;
        }
        
        //in this case all necessary variables to fly back will be swapped
        case(4): {  //setup process for the return flight
            
            //switch case function for the backflight setup
            switch(backflight_setup_state) {
                case(0): {  //case for swapping the position variables
                    //to safe the value of the start position between two steps
                    double step_lat, step_lon;

                    //safe the start position in a cache to use them later as the 
                    //coords for the back flight
                    step_lat = start_lat;
                    step_lon = start_lon;

                    //set the current position as the start coords
                    start_lat = read_current_latitude();
                    start_lon = read_current_longitude();

                    //set the safed values from the cache in for the end position
                    //because the drone has to fly back
                    end_lat = step_lat;
                    end_lon = step_lon;
                    
                    //change the start_altitude value to the altitude 
                    //of the new position
                    altitude_start_position = read_current_altitude();
                    
                    //change the distance variable entfernung
                    entfernung = distance(start_lat, start_lon,
                            end_lat, end_lon);
                    
                    //change the direction in which the drone has to fly
                    himmelsrichtung = courseTO(start_lat, start_lon,
                            end_lat, end_lon);
                    
                    //move on to the next step
                    backflight_setup_state = 1;
                    
                    //end this case
                    break;
                }
                
                case(1): {  //check if the payload is okay
                    
                   //while the payload is null or to heavy 
                    while(payload == 0 || payload > max_weight) {
                        
                        //read the uart message from sercom0 to get the
                        //load cell weight
                        SERCOM0_I2C_Read(711, receive_load_cell, 
                                sizeof(receive_load_cell));
                        
                        //transfer the value of the load cell from the 
                        //receive_load_cell string to the double variable
                        sscanf((const char*)receive_load_cell, "%lf", &payload);
                    }
                    
                        //delay of 50ms to be sure that the variable has been
                        //overwritten correctly
                        delay_ms(50);
                        
                        //if the load cell weight is less than the max weight
                        if(payload < max_weight) {
                            process_state = 5;
                        }
                    //end this case
                    break;
                }
            }
            
            //end this case
            break;
        }
        
        case(5): {  //takeoff process for the backflight
            
            //switch case function for the takeoff process to controll 
            //single steps which are necessary for the takeoff
            switch(backflight_takeoff_process) {
                case(0): {  //case to fly up on the maximum high
                    
                    //while the current high of the drone is lower than
                    //the max flight high the drone flies up
                    while(read_current_altitude() <= 
                            (altitude_start_position + delta_limited_high)) {
                        
                        //write function for the flight controller
                        //to let the drone fly higher with a middle gas
                        write_flight_controller(roll_value, pitch_middle_value, 
                                yaw_middle_value, throttle_max_value);
                    }
                    //write function for the flight controller
                    //to hold the current positon in the air
                    write_flight_controller(roll_value,pitch_middle_value,
                            yaw_middle_value, throttle_middle_value);
                    
                    //move to the next step of the takeoff process
                    backflight_takeoff_process = 1;
                    
                    //end this case
                    break;
                }
                
                case(1): {  //case to rotate the drone in the right direction
                    
                    //while the cardinal direction is not in the direction
                    //where the end position is
                    while(azimuth <= (himmelsrichtung - compass_tolerance) || 
                            azimuth >= (himmelsrichtung + compass_tolerance)) {
                        
                        //to allow the access to the structure
                        single_satelite_data satelites[12];
                        
                        //this variable defines the amount of satellites 
                        //which are connected with the the gps modul
                        int amoutSatelites = 0;
                        char buffer[128];
                        
                        //this for function will run the internal code 12 times
                        for(int i = 0; i< 12; i++) {
                            //read the received gps data from uart as long as
                            //the right gps data are not there
                            while(memcmp(receive_gps, satelite_prefix, 
                                strlen(satelite_prefix)) != 0) {
                                //read the received uart data from SERCOM3 GPS
                                SERCOM3_USART_Read(receive_gps, 
                                        sizeof(receive_gps));
                            }
                            
                            //to get the satellites with the best connection
                            //to the gps modul
                            amoutSatelites = split_satelites_data(buffer, 
                                    satelites);
                            
                            //when the gps modul has connected satellites
                            //then calculate the direction (azimuth) of the gps
                            if(amoutSatelites > 0) {
                                azimuth = compass_direction(satelites, 
                                        amoutSatelites);
                            }
                        }
                        
                        //write function for the flight controller 
                        //to rotate right around in the air
                        write_flight_controller(roll_value, pitch_middle_value, 
                                1600, throttle_middle_value);
                    }
                    
                    //write function for the flight controller
                    //to hold the current positon in the air
                    write_flight_controller(roll_value, pitch_middle_value, 
                            yaw_middle_value, throttle_middle_value);
                    
                    //when the cardinal direction is in the tolerance range
                    if(azimuth > (himmelsrichtung - compass_tolerance) || 
                            azimuth < (himmelsrichtung + compass_tolerance)) {
                        //move on to the next step of the full fligth process
                        process_state = 6;
                    }
                    //end this case
                    break;
                }
            }
            
            //end this case
            break;
        }
        
        case(6): {  //case for the flight to the end position
            
            //while the distance to the end position is bigger than 10meters the
            //drone can fly faster to the position to save energy
            while(read_current_distance() > 10.0) {
                write_flight_controller(roll_value, 1700, yaw_middle_value,
                        1700);
            }
            
            //while the distance is less than 10.0 meters and higher or equal 2
            //meters, the drone should fly slower to the end position
            while(read_current_distance() <= 10.0 
                    && read_current_distance() >= 2.0) {
                write_flight_controller(roll_value, 1600, yaw_middle_value,
                        1600);
            }
            
            //while the distance is less than 2.0 meters and higher or equal 0.2
            //meters, the drone should fly slower to the end position
            //to land more precise at the exat end position
            while(read_current_distance() >= 0.2 
                    && read_current_distance() <= 2.0) {
                write_flight_controller(roll_value, 1550, yaw_middle_value,
                        1550);
            }
            
            //when the drone is in a distance of the tolerance range of 0.2
            //than stop in the air hold the position and move on to the 
            //next flight process
            if(read_current_distance() < 0.2) {
                write_flight_controller(roll_value, 1500, yaw_middle_value,
                        1500);
                
                //move to the next step of the flight process
                process_state = 7;
            }
            
            //end this case
            break;
        }
        
        case(7): {
            
            //while the drone is landing and its high is higher than
            //1 meter the drone can land faster than when the high is less than
            //1 meter
            while(read_current_altitude() > (altitude_start_position + 1)) {
                write_flight_controller(roll_value, pitch_middle_value,
                        yaw_middle_value, 1300);
            }
            
            //while the drone is landing and its high is less than 1 meter
            //the speed of landing has to be slower
            while(read_current_altitude() <= (altitude_start_position + 1)) {
                write_flight_controller(roll_value, pitch_middle_value,
                        yaw_middle_value, 1400);
            }
            
            //when the drone is 5cm or less away from the ground the drone
            //set the throttle variable to the minimum start speed
            if(read_current_altitude() < (altitude_start_position + 0.05)) {
                write_flight_controller(roll_value, pitch_middle_value,
                        yaw_middle_value, throttle_min_value);
                
                //Wait 10 seconds to end the whole flight process
                delay_ms(10000);
                
                //call the function to end the whole flight process
                end_of_flight_process();
            }
            
            //end this case
            break;
        }
    }
}
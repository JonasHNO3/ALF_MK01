/*******************************************************************************
  Main Source File

  Company:
    Schindelar

  File Name:
    main.c

  Summary:
    This file contains the "main" function for the whole project.

  Description:
    This file contains the "main" function for the whole project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes
#include "flugprotokoll.h"              //defines the flight process functions

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

int main ( void )
{
    /* Initialize all modules */
    SYS_Initialize ( NULL );
    
    //set the flight process to true to be able to start a new flight
    set_fly_process(true);
    
    //endless loob
    while ( true )
    {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks ( );
        
        //while there is a flight process
        while(get_fly_process()) {
            SYS_Tasks ( );
           
            //use the full flight protocoll function from flugprotokoll.c
            fly_process();
        }
        
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}


/*******************************************************************************
 End of File
*/


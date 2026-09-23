#include "Application.h"
#include "FlightController.h"


int main()
{
    // Flight controller on Core1, isolated from the tracker on Core0.
    // Launch before the app starts so Core1 is up before FreeRTOS claims
    // Core0's inter-core FIFO interrupt.
    FcInit();
    FcLaunch();

    App<Application>::Run();

    return 0;
}

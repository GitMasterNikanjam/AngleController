// For compile and build:
// mkdir -p ./bin &&  g++ -o ./bin/ex1 ex1.cpp ../AngleController.cpp ../../TimerControl/TimerControl.cpp -Wall -Wextra -std=c++17

// For run:
// sudo ./bin/ex1

// #######################################################################################

#include <iostream>
#include "../AngleController.h"
#include "../../TimerControl/TimerControl.h"
#include <math.h>

using namespace std;

AngleController_SingleDrive controller;
AngleControllerNamespace::SingleDriveParams params;
AngleControllerNamespace::Inputs inputs;
AngleControllerNamespace::Outputs outputs;

TimerControl timer;

uint64_t T;

int main()
{

    params.ANG_P = 1;
    params.RAT_P = 1;
    params.PRIM_RANGE = 100;
    params.PRIM_DEADZONE = 10;
    params.PRIM_MAX = 30;
    params.FRQ = 100;
    params.ANG_UP_LIMIT = 10;

    if(!controller.setParams(params))
    {
        cout << controller.errorMessage << endl;
        return 1;
    }

    if(!controller.init())
    {
        cout << controller.errorMessage << endl;
        return 1;
    }

    // Get parameters
    controller.getParams(&params);

    for(int i=0; i<=600; i++)
    {
        T = timer.micros();

        // Update inputs of controller
        inputs.ang = 45.0*sin((float)T/1000000.0);
        inputs.rat_1 = 0;
        inputs.rat_2 = 0;
        inputs.angDes = 0;
        inputs.ratDes = 0;
        controller.setInputs(inputs);

        // Update outputs of controller
        controller.update(T);

        // Get outputs of controller.
        outputs = controller.outputs;

        // Use outputs of controller.
        cout << controller.getError() << ": " << outputs.primaryOutput * outputs.dir << endl;
    }

        // Print last instance of frequency for controller.
        cout<< "Controller Frequency: " <<  controller.getFrq() << endl;
    return 0;
}
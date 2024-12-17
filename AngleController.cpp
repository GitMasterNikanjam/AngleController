// Attach libraries:

#include "AngleController.h"   

using namespace AngleControllerNamespace;

// ###################################################################

#define _2PI                                6.283185307            // 2*3.14159 = 2*pi

// #############################################################################################

SingleDriveParams::SingleDriveParams()
{
    /* #1 */ ANG_P = 0;
    /* #2 */ ANG_I = 0;
             ANG_IEXP = 0;
             ANG_IZONE = 0;
    /* #3 */ RAT_P = 0;
    /* #4 */ RAT_I = 0;
    /* #5 */ RAT_D = 0;
    /* #6 */ FF1 = 0;
    /* #7 */ FF2 = 0;
    /* #8 */ FLTT = 0;
    /* #9 */ FLTD = 0;
    /* #10 */ FLTO = 0;
    /* #11 */ ANG_LIMIT_ENA = false;
    /* #12 */ ANG_UP_LIMIT = 0;
    /* #13 */ ANG_DOWN_LIMIT = 0;
    /* #14 */ ANG_IMAX = 0;
    /* #15 */ RAT_IMAX = 0;
    /* #16 */ RAT_MAX = 0;
    /* #17 */ RAT_FAST = 0;
    /* #18 */ RAT_SLOW = 0;
    /* #19 */ RAT_SLEWRATE = 0;
    /* #20 */ FF1_MAX = 0;
    /* #21 */ FF2_MAX = 0;
    /* #22 */ FRQ = 0;
    /* #23 */ UPDATE_MODE = 0;
    /* #24 */ PRIM_DEADZONE = 0;
    /* #25 */ PRIM_MAX = 0;
    /* #26 */ PRIM_RANGE = 0;
    /* #27 */ SECON_RANGE = 0;
    /* #28 */ DIR_POL = 0;
}

DualDriveParams::DualDriveParams()
{
    BIAS = 0;
    SLAVE_DIR_POL = 0;
}

DualDriveEqParams::DualDriveEqParams()
{
    /* #1 */ basicParams.ANG_P = 0;
    /* #2 */ basicParams.ANG_I = 0;
             basicParams.ANG_IEXP = 0;
             basicParams.ANG_IZONE = 0;
    /* #3 */ basicParams.RAT_P = 0;
    /* #4 */ basicParams.RAT_I = 0;
    /* #5 */ basicParams.RAT_D = 0;
    /* #6 */ basicParams.FF1 = 0;
    /* #7 */ basicParams.FF2 = 0;
    /* #8 */ basicParams.FLTT = 0;
    /* #9 */ basicParams.FLTD = 0;
    /* #10 */ basicParams.FLTO = 0;
    /* #11 */ basicParams.ANG_LIMIT_ENA = false;
    /* #12 */ basicParams.ANG_UP_LIMIT = 0;
    /* #13 */ basicParams.ANG_DOWN_LIMIT = 0;
    /* #14 */ basicParams.ANG_IMAX = 0;
    /* #15 */ basicParams.RAT_IMAX = 0;
    /* #16 */ basicParams.RAT_MAX = 0;
    /* #17 */ basicParams.RAT_FAST = 0;
    /* #18 */ basicParams.RAT_SLOW = 0;
    /* #19 */ basicParams.RAT_SLEWRATE = 0;
    /* #20 */ basicParams.FF1_MAX = 0;
    /* #21 */ basicParams.FF2_MAX = 0;
    /* #22 */ basicParams.FRQ = 0;
    /* #23 */ basicParams.UPDATE_MODE = 0;
    /* #24 */ basicParams.PRIM_DEADZONE = 0;
    /* #25 */ basicParams.PRIM_MAX = 0;
    /* #26 */ basicParams.PRIM_RANGE = 0;
    /* #27 */ basicParams.SECON_RANGE = 0;
    /* #28 */ basicParams.DIR_POL = 0;

    BIAS = 0;
    RAT_EQ_P = 0;
    RAT_EQ_I = 0;
    RAT_EQ_D = 0;
    RAT_EQ_IMAX = 0;
    FLTDQ = 0;
    SLAVE_DIR_POL = 0;
}

Inputs::Inputs()
{
    direct = 0;
    angle = 0;
    rateMaster = 0;
    rateSlave = 0;
    angleDes = 0;
    rateDes = 0;
}

Outputs::Outputs()
{
    primaryOutput = 0;
    secondaryOutput = 0;
    dir = 0;
}

// #############################################################################################
// General Functions:

double AngleControllerNamespace::limit(const double &input, const double &upLimit, const double &downLimit)
{
    if(input > upLimit)
        return upLimit;
    else if(input < downLimit)
        return downLimit;

    return input;
}

double AngleControllerNamespace::limit(const double &input, const double &limit)
{
    double absLimit = abs(limit);

    if(input > absLimit)
        return absLimit;
    else if(input < -absLimit)
        return -absLimit;

    return input;
}

double AngleControllerNamespace::limitUp(const double &input, const double &upLimit)
{
    if(input > upLimit)
        return upLimit;

    return input;
}

double AngleControllerNamespace::limitDown(const double &input, const double &downLimit)
{
    if(input < downLimit)
        return downLimit;

    return input;
}

// #############################################################################################
// LimitSlewRate class:

bool AngleControllerNamespace::LimitSlewRate::setLimit(double value)
{
    if(value < 0)
    {
        return false;
    }

    SLEWRATE = value;

    return true;
}

double AngleControllerNamespace::LimitSlewRate::updateByTime(const double &input, const double &dt)
{
    if(dt <= 0)
    {
        return output;
    }

    // Calculate derivative of input value.
    double d = (input - _input) / dt;

    _calculate(input, d, 1.0/dt);

    _input = input;

    return output;
}

double AngleControllerNamespace::LimitSlewRate::updateByFrequency(const double &input, const double &frq)
{
    if(frq <= 0)
    {
        return output;
    }

    // Calculate derivative of input value.
    double d = (input - output) * frq;

    _calculate(input, d, frq);

    _input = output;

    return output;
}

void AngleControllerNamespace::LimitSlewRate::clear(void)
{
    output = 0;
}

void AngleControllerNamespace::LimitSlewRate::_calculate(const double &input, const double &derivative, const double &frq)
{
    if(SLEWRATE == 0)
    {
        output = input;
    }

    if(derivative > SLEWRATE)
    {
        output = output + SLEWRATE / frq;
        return;
    }
    else if(derivative < (-SLEWRATE))
    {
        output = output - SLEWRATE / frq;
        return;
    }
    else
    {
        output = input;
    }
}

// ###############################################################################################
// LPF and LPFLimit class:

LPF::LPF()
{
    parameters.FRQ = 0;
    _alpha = 1;

    clear();
}

double LPF::updateByTime(const double &input, const double &dt)
{
    if( (dt <= 0) || (parameters.FRQ < 0) )
    {
        // Not update output.;
        return output;
    }

    if(parameters.FRQ == 0)
    {
        output = input;
        return output;
    }

    _alpha = 1.0/(1.0 + 1.0 / (_2PI * parameters.FRQ * dt));

    output = (1.0 - _alpha) * output + _alpha * input;

    return output;
}

double LPF::updateByFrequency(const double &input, const double &frq)
{
    if( (frq <= 0) || (parameters.FRQ < 0) )
    {
        // Not update output.;
        return output;
    }

    if(parameters.FRQ == 0)
    {
        output = input;
        return output;
    }

    _alpha = 1.0/(1.0 + frq / (_2PI * parameters.FRQ));

    output = (1.0 - _alpha) * output + _alpha * input;

    return true;
}

bool LPF::setFrequency(const double &frq)
{
    if(frq < 0)
    {
        return false;
    }

    parameters.FRQ = frq;

    return true;
}

void LPF::setOutputDirect(const double &data)
{
    output = data;
}

void LPF::clear(void)
{
    output = 0;
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// LPFLimit class derivated from LPF:

LPFLimit::LPFLimit()
{
    parameters.DOWN_LIMIT = 0;
    parameters.DOWN_LIMIT_EN = false;
    parameters.FRQ = 0;
    parameters.UP_LIMIT = 0;
    parameters.UP_LIMIT_EN = true;
}

double LPFLimit::updateByTime(const double &input, const double &dt)
{
    if( (dt <= 0) || (parameters.FRQ < 0) )
    {
        // Not update output.;
        return output;
    }

    if(parameters.FRQ == 0)
    {
        output = input;
        return output;
    }

    float inputLimited = _limitInput(input);

    _alpha = 1.0/(1.0 + 1.0 / (_2PI * parameters.FRQ * dt));

    output = (1.0 - _alpha) * output + _alpha * inputLimited;

    return output;
}

double LPFLimit::updateByFrequency(const double &input, const double &frq)
{
    if( (frq <= 0) || (parameters.FRQ < 0) )
    {
        // Not update output.;
        return output;
    }

    if(parameters.FRQ == 0)
    {
        output = input;
        return output;
    }

    double inputLimited = _limitInput(input);

    _alpha = 1.0/(1.0 + frq / (_2PI * parameters.FRQ));

    output = (1.0 - _alpha) * output + _alpha * inputLimited;

    return output;
}

void LPFLimit::setInputLimit(double limit)
{
    parameters.UP_LIMIT = abs(limit);
    parameters.DOWN_LIMIT = -abs(limit);
}

void LPFLimit::setInputLimit(double upLimit, double DownLimit)
{
    parameters.UP_LIMIT = upLimit;
    parameters.DOWN_LIMIT = DownLimit;
}

double LPFLimit::_limitInput(const double &input)
{
    double inputLimited = input;

    if(parameters.UP_LIMIT_EN == true)
    {
        if(parameters.DOWN_LIMIT_EN == true)
        {
            inputLimited = limit(input, parameters.UP_LIMIT, parameters.DOWN_LIMIT);
        }
        else
        {
            inputLimited = limitUp(input, parameters.UP_LIMIT);
        }
    }
    else if(parameters.DOWN_LIMIT_EN == true)
    {
        inputLimited = limitDown(input, parameters.DOWN_LIMIT);
    }

    return inputLimited;
}

// ###############################################################################################
// PID class:

PController::PController()
{
    parameters.P = 0;
    parameters.FF = 0;
    parameters.FFMAX = 0;
}

double PController::update(const double &desiredState, const double &state)
{
    // Calculte error state.
    _e = desiredState - state;

    output = desiredState * parameters.FF;

    if(parameters.FFMAX != 0)
    {
        output = limit(output , parameters.FFMAX); 
    }

    output = output + parameters.P * _e;

    return output;
}

float PController::getOutput(void)
{
    return output;
}

float PController::getError(void)
{
    return _e;
}

bool PController::_checkParams(void)
{
    if( (parameters.P < 0) || (parameters.FF < 0) || (parameters.FFMAX < 0) )
    {
        errorMessage = "Error PController: one or some parameters are not valid.";
        return false;
    } 

    return true;
}

void PController::clear(void)
{
    output = 0;
    _e = 0;
}

bool PController::init(void)
{
    if(!_checkParams())
    {
        return false;
    }

    clear();

    return true;
}
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

PIController::PIController()
{
    parameters.I = 0;
    parameters.IMAX = 0;
    parameters.IEXP = 0;
    parameters.IZONE = 0;
    _ISum = 0;
}

bool PIController::_checkParams(void)
{
    if( (parameters.P < 0) || (parameters.I < 0) || (parameters.FF < 0) || (parameters.FFMAX < 0) || (parameters.IMAX < 0) ||
        (parameters.IZONE < 0) || (parameters.IEXP < 0) )
    {
        errorMessage = "Error PIController: one or some parameters are not valid.";
        return false;
    } 

    return true;
}

void PIController::clear(void)
{
    output = 0;
    _e = 0;
    _ISum = 0;
}

bool PIController::init(void)
{
    if(!_checkParams())
    {
        return false;
    }

    clear();

    return true;
}

double PIController::updateByTime(const double &desiredState, const double &state, const double &dt)
{
    if(dt == 0)
    {
        errorMessage = "Error PIController: Time step can not be zero or negative.";
        return output;
    }

    double frq = 1.0/dt;

    output = updateByFrequency(desiredState, state, frq);

    return output;
}

double PIController::updateByFrequency(const double &desiredState, const double &state, const double &frq)
{
    if(frq == 0)
    {
        errorMessage = "Error PIController: Frequency of updatation can not be zero or negative.";
        return output;
    }

    // Calculte error state.
    _e = desiredState - state;

    float Ki_dynamic = parameters.I;

    if(parameters.IEXP > 0)
    {
        Ki_dynamic = Ki_dynamic * exp(-abs(_e) * parameters.IEXP);
    }

    // Integral section.
    if(parameters.IZONE > 0)
    {
        if(abs(_e) < parameters.IZONE)
        {
            _ISum = (_ISum + Ki_dynamic * _e / frq);
        }
    }
    else
    {
        _ISum = (_ISum + Ki_dynamic * _e / frq);
    }

    // Limitation for integral controller.
    if(parameters.IMAX != 0)
    {
        _ISum = limit(_ISum, parameters.IMAX);
    }

    output = desiredState * parameters.FF;
    
    if(parameters.FFMAX != 0)
    {
        output = limit(output , parameters.FFMAX); 
    }

    output = output + (_e * parameters.P) + _ISum; 

    return output;
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

PIDController::PIDController()
{
    parameters.D = 0;
    parameters.FLTD = 0; 
    _ePast = 0;
    output = 0;
}

bool PIDController::_checkParams(void)
{
    if( (parameters.P < 0) || (parameters.I < 0) || (parameters.D < 0) || (parameters.FF < 0) || (parameters.FFMAX < 0) || (parameters.IMAX < 0) || (parameters.FLTD < 0) ||
        (parameters.IZONE <0) || (parameters.IEXP <0) )
    {
        errorMessage = "Error PIDController: one or some parameters are not valid.";
        return false;
    } 

    return true;
}

void PIDController::clear(void)
{
    output = 0;
    _e = 0;
    _ISum = 0;
    _ePast = 0;
    _LPFD.setOutputDirect(0);
}

bool PIDController::init(void)
{
    if(!_checkParams())
    {

        return false;
    }

    clear();

    return true;

    _LPFD.setFrequency(parameters.FLTD);
}

double PIDController::updateByTime(const double &desiredState, const double &state, const double &dt)
{
    if(dt == 0)
    {
        errorMessage = "Error PIDController: Time step can not be zero or negative.";
        return output;
    }

    // calculate update frequency. [Hz]
    float frq = 1.0 / dt;

    output = updateByFrequency(desiredState, state, frq);

    return output;
}

double PIDController::updateByFrequency(const double &desiredState, const double &state, const double &frq)
{
    if(frq == 0)
    {
        errorMessage = "Error PIDController: frequency of updatation can not be zero or negative.";
        return output;
    }

    // Calculte error state.
    _e = desiredState - state;

    // Integral section.
    _ISum = (_ISum + parameters.I * _e / frq);

    // Limitation for integral controller.
    if(parameters.IMAX != 0)
    {
        _ISum = limit(_ISum, parameters.IMAX);
    }

    output = desiredState * parameters.FF;

    if(parameters.FFMAX != 0)
    {
        output = limit(output , parameters.FFMAX); 
    }

    output = output + (_e * parameters.P) + _ISum; 
 
    float derivative_input_past = _LPFD.output;
    float derivative_input = _LPFD.updateByFrequency(_e - _ePast, frq); 
    
    // Derivative controller.
    float derivative_output = parameters.D * (derivative_input - derivative_input_past) * frq; 

    // Low pass filter for derivative controller.

    output = output + derivative_output;

    _ePast = _e;

    return output;
}

// #######################################################################################
// Map class:

bool MAP::_checkParams(void)
{
    if( (parameters.PRIM_DEADZONE < 0) || (parameters.PRIM_MAX < 0) || (parameters.PRIM_RANGE < 0) ||
        (parameters.SECON_RANGE < 0) || (parameters.DIR_POL > 1) )
    {
        errorMessage = "Error Map: One or Some parameters are not valid.";
        printf("PRIM_DEADZONE: %f\n", parameters.PRIM_DEADZONE);
        printf("PRIM_MAX: %f\n", parameters.PRIM_MAX);
        printf("PRIM_RANGE: %f\n", parameters.PRIM_RANGE);
        printf("SECON_RANGE: %f\n", parameters.SECON_RANGE);
        printf("DIR_POL: %d\n", parameters.DIR_POL);
        return false;
    }

    if(parameters.PRIM_RANGE > 0)
    {
        if(parameters.PRIM_MAX > parameters.PRIM_RANGE)
        {
            errorMessage = "Error Map: One or Some parameters are not valid.";
            return false;
        }
    }

    if(parameters.PRIM_MAX != 0)
    {
        if((parameters.PRIM_DEADZONE > parameters.PRIM_MAX))
        {
            errorMessage = "Error Map: PRIM_DEADZONE parameter is more than PRIM_MAX parameter.";
            return false;
        }
    }
    
    return true;
}

bool MAP::init(void)
{
    // Check parameters validation.
    if(!_checkParams())
    {
        return false;
    }
    
    // Calculate Slope of line for primary input/output mapping.
    if(parameters.PRIM_RANGE != 0)
    {
        _m_map = (1.0 - (parameters.PRIM_DEADZONE / parameters.PRIM_RANGE));
    }
    else
    {
       _m_map = 1.0; 
    }
    
    if(parameters.DIR_POL == 0)
    {
        _positiveDir = 1;
    }
    else if(parameters.DIR_POL == 1)
    {
        _positiveDir = -1;
    }

    return true;
}

AngleControllerNamespace::Outputs MAP::update(const double &input)
{
    if(input == 0)
    {
        outputs.dir = 0;
    }
    else if(input > 0)
    {
        outputs.dir = _positiveDir;
    }
    else
    {
        outputs.dir = -_positiveDir;
    }
    
    outputs.primaryOutput = parameters.PRIM_DEADZONE + _m_map * abs(input);

    if(parameters.PRIM_MAX != 0)
    {
        outputs.primaryOutput = limit(outputs.primaryOutput, parameters.PRIM_MAX);
    }
    
    if(parameters.PRIM_RANGE != 0)
    {
        outputs.secondaryOutput = parameters.SECON_RANGE * outputs.primaryOutput / parameters.PRIM_RANGE;
    }
    else
    {
        outputs.secondaryOutput = 0;
    }
     
    return outputs;
}

void MAP::clear(void)
{
    outputs.dir = 0;
    outputs.primaryOutput = 0;
    outputs.secondaryOutput = 0;
}

// ###############################################################################################
// AngleController_SingleDrive Class:
 
void AngleController_SingleDrive::clear(void)
{
    outputs.dir = 0;
    outputs.primaryOutput = 0;
    outputs.secondaryOutput = 0;

    _inputs.angle = 0;
    _inputs.direct = 0;
    _inputs.angleDes = 0;
    _inputs.rateMaster = 0;
    _inputs.rateSlave = 0;
    _inputs.rateDes = 0;

    _frq = 0;
    _targetTime = 0;
    _T[0] = 0;
    _T[1] = 0;
    _dT = 0;

    _map.clear();
    _LPFO.clear();
    _LPFT.clear();
    _PIDRate.clear();
    _PIAngle.clear();
    _limitSlewRate.clear();
}

AngleController_SingleDrive::AngleController_SingleDrive(void)
{
    /* #1 */ parameters.ANG_P = 0;
    /* #2 */ parameters.ANG_I = 0;
    /* #3 */ parameters.ANG_IEXP = 0;
    /* #4 */ parameters.ANG_IZONE = 0;
    /* #5 */ parameters.RAT_P = 0;
    /* #6 */ parameters.RAT_I = 0;
    /* #7 */ parameters.RAT_D = 0;
    /* #8 */ parameters.FF1 = 0;
    /* #9 */ parameters.FF2 = 0;
    /* #10 */ parameters.FLTT = 0;
    /* #11 */ parameters.FLTD = 0;
    /* #12 */ parameters.FLTO = 0;
    /* #13 */ parameters.ANG_LIMIT_ENA = false;
    /* #14 */ parameters.ANG_UP_LIMIT = 0;
    /* #15 */ parameters.ANG_DOWN_LIMIT = 0;
    /* #16 */ parameters.ANG_IMAX = 0;
    /* #17 */ parameters.RAT_IMAX = 0;
    /* #18 */ parameters.RAT_MAX = 0;
    /* #19 */ parameters.RAT_FAST = 0;
    /* #20 */ parameters.RAT_SLOW = 0;
    /* #21 */ parameters.RAT_SLEWRATE = 0;
    /* #22 */ parameters.FF1_MAX = 0;
    /* #23 */ parameters.FF2_MAX = 0;
    /* #24 */ parameters.FRQ = 0;
    /* #25 */ parameters.UPDATE_MODE = 0;
    /* #26 */ parameters.PRIM_DEADZONE = 0;
    /* #27 */ parameters.PRIM_MAX = 0;
    /* #28 */ parameters.PRIM_RANGE = 0;
    /* #29 */ parameters.SECON_RANGE = 0;
    /* #30 */ parameters.DIR_POL = 0;

    _mode = AngleController_Mode_None;

    outputs.dir = 0;
    outputs.primaryOutput = 0;
    outputs.secondaryOutput = 0;

    _eAngle = 0;
    _eRate = 0;
    _rateDemanded = 0;

    _inputs.angle = 0;
    _inputs.angleDes = 0;
    _inputs.direct = 0;
    _inputs.rateDes = 0;
    _inputs.rateMaster = 0;
    _inputs.rateSlave = 0;

    clear();
}


bool AngleController_SingleDrive::init(void)
{
           
    if(!_checkParameters(parameters))
    {
        return false;
    }

    clear();

    if(parameters.FRQ > 0)
    {
        _targetTime = 1000000.0/(float)parameters.FRQ;    
    }
    else
    {
        _targetTime = 0;
    }
    
    _map.parameters.PRIM_DEADZONE = parameters.PRIM_DEADZONE;
    _map.parameters.PRIM_MAX = parameters.PRIM_MAX;
    _map.parameters.PRIM_RANGE = parameters.PRIM_RANGE;
    _map.parameters.SECON_RANGE = parameters.SECON_RANGE;
    _map.parameters.DIR_POL = parameters.DIR_POL;
    
    if(!_map.init())
    {
        errorMessage = "Error AngleController: " + _map.errorMessage;
        return false;
    }

    _LPFT.setFrequency(parameters.FLTT);
    _LPFO.setFrequency(parameters.FLTO);

    if(parameters.RAT_MAX != 0)
    {
        _LPFT.parameters.UP_LIMIT_EN = true;
        _LPFT.parameters.DOWN_LIMIT_EN = true;
    }
    else
    {
        _LPFT.parameters.UP_LIMIT_EN = false;
        _LPFT.parameters.DOWN_LIMIT_EN = false;
    }
    
    if(parameters.PRIM_MAX != 0)
    {
        _LPFO.parameters.UP_LIMIT_EN = true;
        _LPFO.parameters.DOWN_LIMIT_EN = true;
    }
    else
    {
        _LPFO.parameters.UP_LIMIT_EN = false;
        _LPFO.parameters.DOWN_LIMIT_EN = false;
    }

    _LPFO.setInputLimit(parameters.PRIM_MAX);
    _LPFT.setInputLimit(parameters.RAT_MAX);

    _PIDRate.parameters.P = parameters.RAT_P;
    _PIDRate.parameters.I = parameters.RAT_I;
    _PIDRate.parameters.IEXP = 0;
    _PIDRate.parameters.IZONE = 0;
    _PIDRate.parameters.D = parameters.RAT_D;
    _PIDRate.parameters.FF = 0;
    _PIDRate.parameters.FFMAX = 0;
    _PIDRate.parameters.IMAX = parameters.RAT_IMAX;
    _PIDRate.parameters.FLTD = parameters.FLTD;
    
    if(!_PIDRate.init())
    {
        errorMessage = _PIDRate.errorMessage;
        return false;
    }

    _PIAngle.parameters.P = parameters.ANG_P;
    _PIAngle.parameters.I = parameters.ANG_I;
    _PIAngle.parameters.IEXP = parameters.ANG_IEXP;
    _PIAngle.parameters.IZONE = parameters.ANG_IZONE;
    _PIAngle.parameters.FF = 0;
    _PIAngle.parameters.FFMAX = 0;
    _PIAngle.parameters.IMAX = parameters.ANG_IMAX;
    
    if(!_PIAngle.init())
    {
        errorMessage = _PIAngle.errorMessage;
        return false;
    }

    _limitSlewRate.setLimit(parameters.RAT_SLEWRATE);

    return true;
}

bool AngleController_SingleDrive::_checkParameters(const AngleControllerNamespace::SingleDriveParams &data)
{
    bool param_cond = (data.ANG_P >= 0) && 
                      (data.ANG_I >= 0) &&
                      (data.ANG_IEXP >= 0) &&
                      (data.ANG_IZONE >= 0) &&             
                      (data.RAT_P >= 0) &&               
                      (data.RAT_I >= 0) &&             
                      (data.RAT_D >= 0) &&             
                      (data.FF1 >= 0) &&                 
                      (data.FF2 >= 0) && 
                      (data.FLTT >= 0) &&                
                      (data.FLTD >= 0) &&              
                      (data.FLTO >= 0) &&    
                      (data.ANG_IMAX >= 0) &&            
                      (data.RAT_IMAX >= 0) &&           
                      (data.RAT_MAX >= 0) &&             
                      (data.RAT_FAST >= 0) &&            
                      (data.RAT_SLOW >= 0) &&            
                      (data.RAT_SLEWRATE >= 0) &&      
                      (data.FF1_MAX >= 0) &&             
                      (data.FF2_MAX >= 0) &&             
                      (data.FRQ >= 0) &&                      
                      (data.PRIM_DEADZONE >= 0) &&      
                      (data.PRIM_MAX >= 0) &&            
                      (data.PRIM_RANGE >= 0) && 
                      (data.SECON_RANGE >= 0);                   

    
    if(!param_cond)
    {
        errorMessage = "Error AngleController: one or some parameters have not correct value.";
        return false;
    }
    
    if(data.PRIM_RANGE > 0)
    {
        if(data.PRIM_MAX > data.PRIM_RANGE)
        {
            errorMessage = "Error AngleController: PRIM_MAX is more than PRIM_RANGE.";
            return false;
        }

        if(data.PRIM_DEADZONE > data.PRIM_RANGE)
        {
            errorMessage = "Error AngleController: PRIM_DEADZONE is more than PRIM_RANGE.";
            return false;
        }
    }

    if(data.PRIM_MAX != 0)
    {
        if(data.PRIM_DEADZONE > data.PRIM_MAX)
        {
            errorMessage = "Error AngleController: PRIM_DEADZONE is more than PRIM_MAX.";
            return false;
        }
    }

    if(data.DIR_POL > 1)
    {
        errorMessage = "Error AngleController: DIR_POL must be 0 or 1.";
        return false;
    }

    if(data.UPDATE_MODE > 1)
    {
        errorMessage = "Error AngleController: UPDATE_MODE must be 0 or 1.";
        return false;
    }

    if(data.ANG_UP_LIMIT < data.ANG_DOWN_LIMIT)
    {
        errorMessage = "Error AngleController: Up angle limitation must be more than Down angle limitation.";
        return false;
    }

    return true;
}

bool AngleController_SingleDrive::update(const uint64_t &T_now)
{
    if( (T_now <= _T[0]) )
    {
        if(T_now == _T[0])
        {
            return true;
        }
        errorMessage = "Error AngleController_SingleDrive: The time differences in the controller cannot be negative.";
        return false;
    }

    if(parameters.FRQ != 0)
    {
        if(T_now < (_T[0] + _targetTime))
        {
            // std::this_thread::sleep_for(std::chrono::microseconds(uint64_t(_T[1] + _targetTime - T_now)));
            // _T[0] = _T[1] + _targetTime;
            return true;
        }
    }

    _T[1] = _T[0];
    _T[0] = T_now;

    _dT = _T[0] - _T[1];
    
    _frq = (1000000.0 / (double)_dT);

    double temp = 0;

    switch(_mode)
    {
        case AngleController_Mode_None:
            _eAngle = 0;
            _eRate = 0;
            temp = 0;
            _PIAngle.clear();
            _LPFT.updateByFrequency(0, _frq);
            _limitSlewRate.updateByFrequency(0, _frq);
            _PIDRate.clear();
            _LPFO.updateByFrequency(0, _frq);
            outputs = _map.update(0);
            return true;
        break;
        case AngleController_Mode_Direct:
            _eAngle = 0;
            _eRate = 0;
            temp = _inputs.direct;
            _PIAngle.clear();
            _LPFT.updateByFrequency(0, _frq);
            _limitSlewRate.updateByFrequency(0, _frq);
            _PIDRate.clear();
            temp = _LPFO.updateByFrequency(temp, _frq);
        break;
        case AngleController_Mode_Angle:
            _eAngle = _inputs.angleDes - _inputs.angle;
            _eRate = 0;
            temp = _PIAngle.updateByFrequency(_inputs.angleDes, _inputs.angle, _frq);
        break;
        case AngleController_Mode_Rate:
            _eRate = _inputs.rateDes - _inputs.rateMaster;
            _eAngle = 0;
            temp = _inputs.rateDes;
        break;
        case AngleController_Mode_Tracking:
            _eAngle = _inputs.angleDes - _inputs.angle;
            _eRate = _inputs.rateDes - _inputs.rateMaster;
            temp = _inputs.rateDes * (double)parameters.FF1;

            if(parameters.FF1_MAX > 0)
            {
                temp = limit(temp, parameters.FF1_MAX);
            }

            temp = temp + _PIAngle.updateByFrequency(_inputs.angleDes, _inputs.angle, _frq);
        break;
        default:
            errorMessage = "Error AngleController_SingleDrive: The mode number of controller is not correct value.";
            return false;
    }
    
    if(_mode != AngleController_Mode_Direct)
    {
        temp = _LPFT.updateByFrequency(temp, _frq);
        temp = _limitSlewRate.updateByFrequency(temp, _frq);
        _PIDRate.updateByFrequency(temp, _inputs.rateMaster, _frq);
        temp = temp * parameters.FF2;
        if(parameters.FF2_MAX > 0)
        {
            temp = limit(temp, parameters.FF2_MAX);
        }
        temp = temp + _PIDRate.output;
        temp = _LPFO.updateByFrequency(temp, _frq);
    }

    if(parameters.ANG_LIMIT_ENA == true)
    {
        if(_inputs.angle >= parameters.ANG_UP_LIMIT)
        {
            if(temp > 0)
            {
                temp = 0;
            }
        }
        else if(_inputs.angle <= parameters.ANG_DOWN_LIMIT)
        {
            if(temp < 0)
            {
                temp = 0;
            }
        }
    }

    outputs = _map.update(temp);

    return true;
}

void AngleController_SingleDrive::getParams(AngleControllerNamespace::SingleDriveParams *params)
{
    *params = parameters;
}

bool AngleController_SingleDrive::setParams(const AngleControllerNamespace::SingleDriveParams &data)
{
    if(_checkParameters(data))
    {
        return false;
    }

    parameters = data;
    return true;
}

bool AngleController_SingleDrive::setMode(const uint8_t &mode)
{
    if(mode > 4)
    {
        errorMessage = "Error AngleController: The mode value is not correct. The mode value must be 0: None, 1:Direct, 2:Angle, 3:Rate and 4:Tracking";
        return false;
    }

    // Check change mode condition
    if(mode != _mode)
    {
       _mode = mode;
    }

    return true;
}

uint8_t AngleController_SingleDrive::getMode(void)
{
    return _mode;
}

void AngleController_SingleDrive::setInputs(const AngleControllerNamespace::Inputs &data)
{   
    _inputs = data;
    
    if(parameters.ANG_LIMIT_ENA == true)
    {
        _inputs.angleDes = limit(_inputs.angleDes, parameters.ANG_UP_LIMIT, parameters.ANG_DOWN_LIMIT);

        if(_inputs.angle >= parameters.ANG_UP_LIMIT)
        {
            if(_inputs.rateDes > 0)
            {
                _inputs.rateDes = 0;
            }
        }
        else if(_inputs.angle <= parameters.ANG_DOWN_LIMIT)
        {
            if(_inputs.rateDes < 0)
            {
                _inputs.rateDes = 0;
            }
        }
    }

    if(parameters.RAT_MAX != 0)
    {
        _inputs.rateDes = limit(_inputs.rateDes, parameters.RAT_MAX);
    }

}

float AngleController_SingleDrive::getFrq(void) 
{
    return _frq;
} 

float AngleController_SingleDrive::getError(void)
{
    switch(_mode)
    {
        case AngleController_Mode_None:
            return 0;
        break;
        case AngleController_Mode_Direct:
            return 0;
        break;
        case AngleController_Mode_Angle:
            return _eAngle;
        break;
        case AngleController_Mode_Rate:
            return _eRate;
        break;
        case AngleController_Mode_Tracking:
            return _eAngle;
        break;
        default:
            return 0;
    }

    return 0;
}

float AngleController_SingleDrive::getRateDemanded(void)
{
    if( (_mode == AngleController_Mode_Direct) || (_mode == AngleController_Mode_None) )
    {
        return 0;
    }

    return _rateDemanded;
}

// ############################################################################################

AngleController_DualDriveEq::AngleController_DualDriveEq()
{
    /*1*/   parameters.basicParams.ANG_P = 0; 
    /*2*/   parameters.basicParams.ANG_I = 0;  
    /*3*/   parameters.basicParams.ANG_IEXP = 0;
    /*4*/   parameters.basicParams.ANG_IZONE = 0;            
    /*5*/   parameters.basicParams.RAT_P = 0;               
    /*6*/   parameters.basicParams.RAT_I = 0;               
    /*7*/   parameters.basicParams.RAT_D = 0;           
    /*8*/   parameters.basicParams.FF1 = 0;                    
    /*9*/   parameters.basicParams.FF2 = 0;                 
    /*10*/  parameters.basicParams.FLTT = 0;              
    /*11*/  parameters.basicParams.FLTD = 0;                 
    /*12*/  parameters.basicParams.FLTO = 0;  
    /*13*/  parameters.basicParams.ANG_LIMIT_ENA = false;
    /*14*/  parameters.basicParams.ANG_UP_LIMIT = 0;
    /*15*/  parameters.basicParams.ANG_DOWN_LIMIT = 0;  
    /*16*/  parameters.basicParams.ANG_IMAX = 0;            
    /*17*/  parameters.basicParams.RAT_IMAX = 0;    
    /*18*/  parameters.basicParams.RAT_MAX = 0;  
    /*19*/  parameters.basicParams.RAT_FAST = 0;
    /*20*/  parameters.basicParams.RAT_SLOW = 0;
    /*21*/  parameters.basicParams.RAT_SLEWRATE = 0;
    /*22*/  parameters.basicParams.FF1_MAX = 0;            
    /*23*/  parameters.basicParams.FF2_MAX = 0;            
    /*24*/  parameters.basicParams.FRQ = 0;                      
    /*25*/  parameters.basicParams.UPDATE_MODE = 0;
    /*26*/  parameters.basicParams.PRIM_DEADZONE = 0;  
    /*27*/  parameters.basicParams.PRIM_MAX = 0;  
    /*28*/  parameters.basicParams.PRIM_RANGE = 0;          
    /*29*/  parameters.basicParams.SECON_RANGE = 0;          
    /*30*/  parameters.basicParams.DIR_POL = 0; 

    parameters.BIAS = 0;
    parameters.FLTDQ = 0;
    parameters.RAT_EQ_D = 0;
    parameters.RAT_EQ_I = 0;
    parameters.RAT_EQ_IMAX = 0;
    parameters.RAT_EQ_P = 0;
    parameters.SLAVE_DIR_POL = 0;

    _mode = AngleController_Mode_None;

    clear();
}

void AngleController_DualDriveEq::clear(void)
{
    outputs.dir = 0;
    outputs.primaryOutput = 0;
    outputs.secondaryOutput = 0;

    outputsSlave.dir = 0;
    outputsSlave.primaryOutput = 0;
    outputsSlave.secondaryOutput = 0;

    _inputs.angle = 0;
    _inputs.angleDes = 0;
    _inputs.rateMaster = 0;
    _inputs.rateSlave = 0;
    _inputs.rateDes = 0;

    _frq = 0;
    _targetTime = 0;
    _T[0] = 0;
    _T[1] = 0;
    _dT = 0;

    _map.clear();
    _mapSlave.clear();
    _LPFO.clear();
    _LPFT.clear();
    _PIDRate.clear();
    _PIDEq.clear();
    _PIDRateSlave.clear();
    _limitSlewRate.clear();
}

void AngleController_DualDriveEq::setInputs(const AngleControllerNamespace::Inputs &data)
{
     _inputs = data;
    
    if(parameters.basicParams.ANG_LIMIT_ENA == true)
    {
        _inputs.angleDes = limit(_inputs.angleDes, parameters.basicParams.ANG_UP_LIMIT, parameters.basicParams.ANG_DOWN_LIMIT);

        if(_inputs.angle >= parameters.basicParams.ANG_UP_LIMIT)
        {
            if(_inputs.rateDes > 0)
            {
                _inputs.rateDes = 0;
            }
        }
        else if(_inputs.angle <= parameters.basicParams.ANG_DOWN_LIMIT)
        {
            if(_inputs.rateDes < 0)
            {
                _inputs.rateDes = 0;
            }
        }
    }

    if(parameters.basicParams.RAT_MAX != 0)
    {
        _inputs.rateDes = limit(_inputs.rateDes, parameters.basicParams.RAT_MAX);
    }
} 

bool AngleController_DualDriveEq::setParams(const AngleControllerNamespace::DualDriveEqParams &data)
{
    if(!_checkParameters(data))
    {
        return false;
    }

    parameters = data;

    return true;
}

void AngleController_DualDriveEq::getParams(AngleControllerNamespace::DualDriveEqParams *data)
{
    *data = parameters;
}

bool AngleController_DualDriveEq::_checkParameters(const AngleControllerNamespace::DualDriveEqParams &data)
{

   bool param_cond = AngleController_SingleDrive::_checkParameters(data.basicParams);                 

    param_cond = param_cond && (parameters.BIAS >= 0) && (parameters.FLTDQ >= 0) && (parameters.RAT_EQ_D >= 0) &&
                 (parameters.RAT_EQ_I >= 0) && (parameters.RAT_EQ_IMAX >= 0) && (parameters.RAT_EQ_P >= 0);

    
    if(!param_cond)
    {
        errorMessage = "Error AngleController: one or some parameters have not correct value.";
        return false;
    }

    return true;
}

bool AngleController_DualDriveEq::init(void)
{   
    if(!_checkParameters(parameters))
    {
        return false;
    }

    clear();

    if(parameters.basicParams.FRQ > 0)
    {
        _targetTime = 1000000.0/(float)parameters.basicParams.FRQ;    
    }
    else
    {
        _targetTime = 0;
    }
    
    _map.parameters.PRIM_DEADZONE = parameters.basicParams.PRIM_DEADZONE;
    _map.parameters.PRIM_MAX = parameters.basicParams.PRIM_MAX;
    _map.parameters.PRIM_RANGE = parameters.basicParams.PRIM_RANGE;
    _map.parameters.SECON_RANGE = parameters.basicParams.SECON_RANGE;
    _map.parameters.DIR_POL = parameters.basicParams.DIR_POL;

    _mapSlave.parameters = _map.parameters;

    _mapSlave.parameters.DIR_POL = parameters.SLAVE_DIR_POL;
    
    if(!_map.init())
    {
        errorMessage = "Error AngleController: " + _map.errorMessage;
        return false;
    }

    if(!_mapSlave.init())
    {
        errorMessage = "Error AngleController: " + _mapSlave.errorMessage;
        return false;
    }

    _LPFT.setFrequency(parameters.basicParams.FLTT);
    _LPFO.setFrequency(parameters.basicParams.FLTO);

    if(parameters.basicParams.RAT_MAX != 0)
    {
        _LPFT.parameters.UP_LIMIT_EN = true;
        _LPFT.parameters.DOWN_LIMIT_EN = true;
    }
    else
    {
        _LPFT.parameters.UP_LIMIT_EN = false;
        _LPFT.parameters.DOWN_LIMIT_EN = false;
    }
    
    if(parameters.basicParams.PRIM_MAX != 0)
    {
        _LPFO.parameters.UP_LIMIT_EN = true;
        _LPFO.parameters.DOWN_LIMIT_EN = true;
    }
    else
    {
        _LPFO.parameters.UP_LIMIT_EN = false;
        _LPFO.parameters.DOWN_LIMIT_EN = false;
    }

    _LPFO.setInputLimit(parameters.basicParams.PRIM_MAX);
    _LPFT.setInputLimit(parameters.basicParams.RAT_MAX);

    _PIDRate.parameters.P = parameters.basicParams.RAT_P;
    _PIDRate.parameters.I = parameters.basicParams.RAT_I;
    _PIDRate.parameters.IEXP = 0;
    _PIDRate.parameters.IZONE = 0;
    _PIDRate.parameters.D = parameters.basicParams.RAT_D;
    _PIDRate.parameters.FF = 0;
    _PIDRate.parameters.FFMAX = 0;
    _PIDRate.parameters.IMAX = parameters.basicParams.RAT_IMAX;
    _PIDRate.parameters.FLTD = parameters.basicParams.FLTD;

    _PIDRateSlave.parameters = _PIDRate.parameters;

    _PIDEq.parameters.P = parameters.RAT_EQ_P;
    _PIDEq.parameters.I = parameters.RAT_EQ_I;
    _PIDEq.parameters.IEXP = 0;
    _PIDEq.parameters.IZONE = 0;
    _PIDEq.parameters.D = parameters.RAT_EQ_D;
    _PIDEq.parameters.FF = 0;
    _PIDEq.parameters.FFMAX = 0;
    _PIDEq.parameters.IMAX = parameters.RAT_EQ_IMAX;
    _PIDEq.parameters.FLTD = parameters.FLTDQ;

    
    if(!_PIDRate.init())
    {
        errorMessage = _PIDRate.errorMessage;
        return false;
    }

    if(!_PIDRateSlave.init())
    {
        errorMessage = _PIDRateSlave.errorMessage;
        return false;
    }

    if(!_PIDEq.init())
    {
        errorMessage = _PIDEq.errorMessage;
        return false;
    }
    
    _limitSlewRate.setLimit(parameters.basicParams.RAT_SLEWRATE);
    
    return true;
}

bool AngleController_DualDriveEq::update(const uint64_t &T_now)
{
    if( (T_now <= _T[0]) )
    {
        if(T_now == _T[0])
        {
            return true;
        }
        errorMessage = "Error AngleController: The time differences in the controller cannot be negative.";
        return false;
    }

    if(parameters.basicParams.FRQ != 0)
    {
        if(T_now < (_T[0] + _targetTime))
        {
            // std::this_thread::sleep_for(std::chrono::microseconds(uint64_t(_T[1] + _targetTime - T_now)));
            // _T[0] = _T[1] + _targetTime;
            return true;
        }
    }

    _T[1] = _T[0];
    _T[0] = T_now;

    _dT = _T[0] - _T[1];
    
    _frq = (1000000.0 / (double)_dT);
    
    double temp;

    switch(_mode)
    {
        case AngleController_Mode_None:
            _eAngle = 0;
            _eRate = 0;
            temp = 0;
            _LPFT.updateByFrequency(0, _frq);
            _limitSlewRate.updateByFrequency(0, _frq);
            _PIDRate.clear();
            _PIDRateSlave.clear();
            _PIDEq.clear();
            _LPFO.updateByFrequency(0, _frq);
            outputs = _map.update(0);
            outputsSlave = _map.update(0);
            return true;
        break;
        case AngleController_Mode_Direct:
            _eAngle = 0;
            _eRate = 0;
            temp = _inputs.direct;
            _LPFT.updateByFrequency(0, _frq);
            _limitSlewRate.updateByFrequency(0, _frq);
            _PIDRate.clear();
            _PIDRate.clear();
            _PIDEq.clear();
            temp = _LPFO.updateByFrequency(temp, _frq);
        break;
        case AngleController_Mode_Angle:
            // Error angle signal
            _eAngle = _inputs.angleDes - _inputs.angle;
            _eRate = 0;
            temp = _eAngle * parameters.basicParams.ANG_P;
        break;
        case AngleController_Mode_Rate:
            _eRate = _inputs.rateDes - _inputs.rateMaster;
            _eAngle = 0;
            temp = _inputs.rateDes;
        break;
        case AngleController_Mode_Tracking:
            _eAngle = _inputs.angleDes - _inputs.angle;
            _eRate = _inputs.rateDes - _inputs.rateMaster;
            temp = _inputs.rateDes * (double)parameters.basicParams.FF1;

            if(parameters.basicParams.FF1_MAX > 0)
            {
                temp = limit(temp, parameters.basicParams.FF1_MAX);
            }

            temp = temp + _eAngle * parameters.basicParams.ANG_P;
        break;
        default:
            errorMessage = "Error AngleController: The mode number of controller is not correct value.";
            return false;
    }
    
    if(_mode != AngleController_Mode_Direct)
    {
        if(parameters.basicParams.RAT_MAX > 0)
        {
            temp = limit(temp, parameters.basicParams.RAT_MAX);
        }

        if(parameters.basicParams.ANG_LIMIT_ENA == true)
        {
            if( (abs(_inputs.angle - parameters.basicParams.ANG_DOWN_LIMIT) <= 10) || (abs(_inputs.angle - parameters.basicParams.ANG_UP_LIMIT) <= 10) )
            {
                temp = limit(temp, 1);
            }
        }
        
        temp = _LPFT.updateByFrequency(temp, _frq);
        temp = _limitSlewRate.updateByFrequency(temp, _frq);
        _rateDemanded = temp;

        float FF2 = parameters.basicParams.FF2 * temp;

        if(parameters.basicParams.FF2_MAX != 0)
        {
            FF2 = limit(FF2, parameters.basicParams.FF2_MAX);
        }

        temp = _PIDRate.updateByFrequency(temp, _inputs.rateMaster, _frq);
        
        double tempSalve;

        tempSalve = _PIDRateSlave.updateByFrequency(temp, _inputs.rateSlave + _PIDEq.output, _frq);

        _PIDEq.updateByFrequency(tempSalve, temp, _frq);

        temp = (temp + tempSalve) * 0.5 + FF2;
        temp = _LPFO.updateByFrequency(temp, _frq);
    }

    if(parameters.basicParams.PRIM_DEADZONE > 0)
    {
        if(abs(temp) < parameters.basicParams.PRIM_DEADZONE)
        {
            if(temp > 0)
            {
                temp = parameters.basicParams.PRIM_DEADZONE;
            }
            else if(temp < 0)
            {
                temp = -parameters.basicParams.PRIM_DEADZONE;
            }
            else
            {
                temp = 0;
            }
        }
    }

    if(parameters.basicParams.ANG_LIMIT_ENA == true)
    {
        if(_inputs.angle >= parameters.basicParams.ANG_UP_LIMIT)
        {
            if(temp > 0)
            {
                temp = 0;
            }
        }
        else if(_inputs.angle <= parameters.basicParams.ANG_DOWN_LIMIT)
        {
            if(temp < 0)
            {
                temp = 0;
            }
        }
    }

    outputs = _map.update(temp + parameters.BIAS);
    outputsSlave = _mapSlave.update(temp - parameters.BIAS);

    return true;
}
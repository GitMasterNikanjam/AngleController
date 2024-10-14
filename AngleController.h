#ifndef ANGLE_CONTROLLER_H
#define ANGLE_CONTROLLER_H

// ########################################################
// Include libraries:

#include <iostream>
#include <thread>
#include <chrono>

using namespace std;
// ######################################################
// AngleController_SingleDrive(Mode:Angle): Controller Shematic:  

/* 
                                                                                                |->Limit(*FF2) ------------------------------------->|(+)|                                           |-> PrimaryOutput
Limit(Rate_des[deg/s]*FF1)------------------>|(+)|                                              |       |*KP_rate ---------------------------------->|(+)|                                           |-> SecondaryOutput 
(Limit(Angle_des[deg])-Angle[deg])*KP_angle->|(+)|->Limit->Lowpass_Filter(FLTT)->SlewRateLimit->|-Rate->|Integral_Limit(*KI_rate) ------------------>|(+)|->Limit->Lowpass_Filter(FLTO)->Map_Function|-> Direction
                                                                                                        |Derivative(*KD_rate)->Lowpass_Filter(FLTD)->|(+)|                                           
*/

// #####################################################
// AngleController_SingleDrive(Mode:Rate): Controller Shematic:  

/* 
                                                             |->Limit(*FF2) ------------------------------------->|(+)|                                           |-> PrimaryOutput
                                                             |       |*KP_rate ---------------------------------->|(+)|                                           |-> SecondaryOutput 
Rate_des[deg/s]->Limit->Lowpass_Filter(FLTT)->SlewRateLimit->|-Rate->|Integral_Limit(*KI_rate) ------------------>|(+)|->Limit->Lowpass_Filter(FLTO)->Map_Function|-> Direction
                                                                     |Derivative(*KD_rate)->Lowpass_Filter(FLTD)->|(+)|                                           
*/


// ######################################################
// AngleController_DualDrive(Mode:Angle)(Without equalizer): Controller Shematic: 

/*                                
                                                                                                                                                                                                                  |-> PrimaryOutput                                                
                                                                                                                                                                                                                  |-> SecondaryOutput                                                                                                                      
                                                                                                |->Limit(*FF2) -------------------------------------->|(+)|                            BIAS->|(-)|->Map_Function->|-> Direction    
Limit(Rate_des[deg/s]*FF1)------------------>|(+)|                                              |        |*KP_rate ---------------------------------->|(+)|                               |->|(+)|                
(Limit(Angle_des[deg])-Angle[deg])*KP_angle->|(+)|->Limit->Lowpass_Filter(FLTT)->SlewRateLimit->|-Rate1->|Integral_Limit(*KI_rate) ------------------>|(+)|->Limit->Lowpass_Filter(FLTO)->|                                    
                                                                                                         |Derivative(*KD_rate)->Lowpass_Filter(FLTD)->|(+)|                               |->|(+)|                |-> PrimaryOutput              
                                                                                                                                                                                       BIAS->|(+)|->Map_Function->|-> SecondaryOutput             
                                                                                                                                                                                                                  |-> Direction                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
*/

// ######################################################
// AngleController_DualDrive(Mode:Rate)(Without equalizer): Controller Shematic: 

/* 
                                                                                                                                                                               |-> PrimaryOutput                                                
                                                                                                                                                                               |-> SecondaryOutput                                                                                                                      
                                                             |->Limit(*FF2) -------------------------------------->|(+)|                            BIAS->|(-)|->Map_Function->|-> Direction    
                                                             |        |*KP_rate ---------------------------------->|(+)|                               |->|(+)|                
Rate_des[deg/s]->Limit->Lowpass_Filter(FLTT)->SlewRateLimit->|-Rate1->|Integral_Limit(*KI_rate) ------------------>|(+)|->Limit->Lowpass_Filter(FLTO)->|                                    
                                                                      |Derivative(*KD_rate)->Lowpass_Filter(FLTD)->|(+)|                               |->|(+)|                |-> PrimaryOutput              
                                                                                                                                                    BIAS->|(+)|->Map_Function->|-> SecondaryOutput             
                                                                                                                                                                               |-> Direction               
                                                                                                                                                                                                                           
*/

// ######################################################
// AngleController_DualDrive(Mode:Angle)(With equalizer): Controller Shematic: 

/* 
                                                                                                |->Limit(*FF2) -------------------------------------------------------------------------|                                   
Limit(Rate_des[deg/s]*FF1)------------------>|(+)|                                              |        |*KP_rate ---------------------------------->|(+)|                             |                 
(Limit(Angle_des[deg])-Angle[deg])*KP_angle->|(+)|->Limit->Lowpass_Filter(FLTT)->SlewRateLimit->|-Rate1->|Integral_Limit(*KI_rate) ------------------>|(+)|------------->|              |                                                              |-> PrimaryOutput
                                                                                                |        |Derivative(*KD_rate)->Lowpass_Filter(FLTD)->|(+)|              |              |                                                              |-> SecondaryOutput 
                                                                                                |                                                                        |              |                                   BIAS->|(-)|->Map_Function->|-> Direction
                                                                                                |              |(+)|<------------------------------------*KP_eq|  |(-)|<-|->|(+)|       |->|(+)|                               |->|(+)|               
                                                                                                |      |<------|(+)|<--------------------Integral_Limit(*KI_eq)|<-|   |     |   |->(*0.5)->|(+)|->Limit->Lowpass_Filter(FLTO)->|
                                                                                                |      |       |(+)|<-Lowpass_Filter(FLTDQ)<-Derivative(*KD_eq)|  |(+)|<-|->|(+)|                                              |->|(+)|                |-> PrimaryOutput
                                                                                                |      |                                                                 |                                                  BIAS->|(+)|->Map_Function->|-> SecondaryOutput
                                                                                                |      |->|(-)|                                                          |                                                                             |-> Direction
                                                                                                |         |   |->|*KP_rate ---------------------------------->|(+)|      |                                                                     
                                                                                                |-Rate2|->|(+)|  |Integral_Limit(*KI_rate) ------------------>|(+)|----->|
                                                                                                                 |Derivative(*KD_rate)->Lowpass_Filter(FLTD)->|(+)|                                               
*/

// ######################################################
// AngleController_DualDrive(Mode:Rate)(With equalizer): Controller Shematic: 

/* 
                                                             |->Limit(*FF2) ------------------------------------------------------------------------|                                   
                                                             |        |*KP_rate ---------------------------------->|(+)|                            |                   
Rate_des[deg/s]->Limit->Lowpass_Filter(FLTT)->SlewRateLimit->|-Rate1->|Integral_Limit(*KI_rate) ------------------>|(+)|------------>|              |                                                              |-> PrimaryOutput
                                                             |        |Derivative(*KD_rate)->Lowpass_Filter(FLTD)->|(+)|             |              |                                                              |-> SecondaryOutput 
                                                             |                                                                       |              |                                   BIAS->|(-)|->Map_Function->|-> Direction
                                                             |              |(+)|<-----------------------------------*KP_eq|  |(-)|<-|->|(+)|       |->|(+)|                               |->|(+)|                
                                                             |      |<------|(+)|<-------------------Integral_Limit(*KI_eq)|<-|   |     |   |->(*0.5)->|(+)|->Limit->Lowpass_Filter(FLTO)->|
                                                             |      |       |(+)|<-Lowpass_Filter(FLTDQ)<-Derivative(*KD_eq)| |(+)|<-|->|(+)|                                              |->|(+)|                |-> PrimaryOutput
                                                             |      |                                                                |                                                  BIAS->|(+)|->Map_Function->|-> SecondaryOutput
                                                             |      |->|(-)|                                                         |                                                                             |-> Direction
                                                             |         |   |->|*KP_rate ---------------------------------->|(+)|     |                                                                      
                                                             |-Rate2|->|(+)|  |Integral_Limit(*KI_rate) ------------------>|(+)|---->|
                                                                              |Derivative(*KD_rate)->Lowpass_Filter(FLTD)->|(+)|                                               
*/

// #####################################################
// Define Macros:

#define AngleController_UpdateMode_Manual        0   // Controller output updated when call update function.
#define AngleController_UpdateMode_Auto          1   // Controller output updated automatically at separated thread. ** _FRQ parameter must be non zero, otherwise this mode not worked.

#define AngleController_Mode_Angle               0
#define AngleController_Mode_Rate                1

// #####################################################
// AngleController essential struct variables:

namespace AngleControllerNamespace
{
    // Full parameters struct for SingleDrive AngleController
    struct SingleDriveParams
    {
        
        // /*1*/
        // [-]. Angle controller P gain
        float ANG_P; 

        // /*2*/
        // [-]. Rate controller P gain               
        float RAT_P; 

        // /*3*/
        // [-]. Rate controller I gain               
        float RAT_I;  
                
        // /*4*/
        // [-]. Rate controller D gain              
        float RAT_D; 

        // /*5*/
        // [-]. controller feed forward gain type1. (For angle controller).               
        float FF1; 

        // /*6*/
        // [-]. controller feed forward gain type2. (for rate controller)                 
        float FF2;  

        // /*7*/
        // [Hz]. Lowpass filter frequency for target rate. **Hint: 0 value is equal to disable                
        float FLTT;                 
        
        // /*8*/   
        // [Hz]. Lowpass filter frequency for derivative of rate. **Hint: 0 value is equal to disable
        float FLTD;                  
        
        // /*9*/   
        // [-]. Lowpass filter frequency for controller output. **Hint: 0 value is equal to disable 
        float FLTO;                 
        
        // /*10*/  
        // [deg]. Upper limitation angle for controller. It works at all modes.
        float ANG_UP_LIMIT;          
        
        // /*11*/  
        // [deg]. Down limitation angle for controller. it works at all modes.
        float ANG_DOWN_LIMIT;       
        
        // /*12*/  
        // [-]. Max integral control output for controller output. Hint: 0 Value is equal to Disable
        float RAT_IMAX;             
        
        // /*13*/  
        // [deg/s]. Max Target rate. **Hint: 0 value is equal to disable
        float RAT_MAX;              
        
        // /*14*/  
        // [deg/s]. Target rate magnitude for fast movement state.
        float RAT_FAST;             
        
        // /*15*/  
        // [deg/s]. Target rate magnetude for slow movement state.
        float RAT_SLOW;             
        
        // /*16*/  
        // [deg/s^2]. angle acceleration/decleration magnetude limitation.
        float RAT_SLEWRATE;         
        
        // /*17*/  
        // [deg/s]. Max feedForward control output for target rate. **Hint: 0 value is equal to disable 
        float FF1_MAX;              
        
        // /*18*/ 
        // [-]. Max feedforward control output for controller output. **Hint: 0 value is equal to disable 
        float FF2_MAX;              

        // /*19*/
        // [Hz]. Controller loop frequency.
        // 0: Free Frequency-> time and frequency changed when controller become update.
        // Non zero: Lock Frequency-> time and frequency of controller update locked to FRQ value.
        float FRQ;                      

        // /*20*/
        // [-]. Controller update mode.
        // 0: Manual update-> Controller output updated when call update function.
        // 1: Auto update-> Controller output updated automatically at separated thread. ** FRQ parameter must be non zero, otherwise this mode not worked.
        uint8_t UPDATE_MODE;
        
        // /*21*/  
        // [-]. PrimaryOutput deadzone value for maped controller output. **Hint: 0 value is equal to disable
        float PRIM_DEADZONE;       
        
        // /*22*/ 
        // [-]. Max PrimaryOutput value for maped controller output. **Hint: 0 value is equal to disable 
        float PRIM_MAX;            
        
        // /*23*/  
        // [-]. Range PrimaryOutput value for maped controller output. **Hint: 0 value is equal to disable
        float PRIM_RANGE;          
       
        // /*24*/  
        // [-]. Range SecondaryOutput value for maped controller output. **Hint: 0 value is equal to disable
        float SECON_RANGE;         
        
        // /*25*/  
        // [-]. Controller direction polarity. normal direction:0, reverse direction:1
        uint8_t DIR_POL;           
    };

    // Full parameters struct for simple DualDrive AngleController without equalizer.
    struct DualDriveParams
    {
        // Basic parameters of SingleDriveParams
        SingleDriveParams basicParams;

        // [-]. BIAS magnitude value for primaryOutput bias.
        float BIAS;                         

        // [-]. Controller direction polarity for slave driver. normal direction:0, reverse direction:1
        uint8_t SLAVE_DIR_POL;              
    };

    // Full parameters struct for advanced DualDrive AngleController with equalizer.
    struct DualDriveEqParams
    {
        // Basic parameters of SingleDriveParams
        SingleDriveParams basicParams;      

        // [-]. BIAS magnitude value for primaryOutput bias.
        float BIAS;              

        // [-]. Rate controller P gain for equalization           
        float RAT_EQ_P;          

        // [-]. Rate controller I gain for equalization           
        float RAT_EQ_I;             

        // [-]. Rate controller D gain for equalization        
        float RAT_EQ_D;      

         // [-]. Max integral control output for equalization controller output. Hint: 0 Value is equal to Disable.               
        float RAT_EQ_IMAX;  

        // [Hz]. Lowpass filter frequency for derivative of rate for equalization. **Hint: 0 value is equal to disable.               
        float FLTDQ;     

        // [-]. Controller direction polarity for slave driver. normal direction:0, reverse direction:1                   
        uint8_t SLAVE_DIR_POL;              
    };

    // Full input variables for AngleController 
    struct Inputs
    {
        float ang;                          // [deg]. Angle value of plant.
        float rat_1;                        // [deg/s]. First rate observer value of plant.
        float rat_2;                        // [deg/s]. Second rate observer value of plant.
        float angDes;                       // [deg]. Desired angle value for controller. 
        float ratDes;                       // [deg/s]. Desired rate value for controller.
    };

    // Full output variables for AngleController
    struct Outputs
    {
        float primaryOutput;                // [-]. primaryOutput value output from AngleController.
        float secondaryOutput;              // [-]. secondaryOutput value output from AngleController. Depends on primaryOutput value.
        int8_t dir;                         // [-]. Direction signal value output from AngleController. {-1, 0, 1}
    };
}

// ##################################################################################
// General Functions:

namespace AngleControllerNamespace
{
    /** 
     * General upper limit function.
     * @param input: input data.
     * @param upLimit: up limitation value.
     * @return Limited value. 
    */ 
    float limitUp(const float &input, const float &upLimit);

    /** 
     * General lower limit function.
     * @param input: input data.
     * @param downLimit: down limitation value.
     * @return Limited value. 
    */ 
    float limitDown(const float &input, const float &downLimit);

    /** 
     * General limit function.
     * @param input: input data.
     * @param upLimit: up limitation value.
     * @param downLimit: down limitation value.
     * @return Limited value. 
    */ 
    float limit(const float &input, const float &upLimit, const float &downLimit);

    /** 
     * General limit function.
     * @param input: input data.
     * @param limit: up/down limitation value. up_limit = +abs(limit), down_limit = -abs(limit).
     * @return Limited value. 
    */         
    float limit(const float &input, const float &limit);

}

// #######################################################################################
// LimitSlewRate class:
// This class is for change rate limit.

namespace AngleControllerNamespace
{
    // Limit for slew rate class. 
    class LimitSlewRate
    {
        public:

            LimitSlewRate()
            {   
                _input = 0;
                output = 0;
                SLEWRATE = 0;
            }

            // Buffer for output of slewRate value that used in update method.
            float output;                           

            /**
             * Set slew rate parameter value.
             * @param vlaue: must be positive.
             * @return true if successed.
             */
            bool setLimit(float value);

            /**
             * Update output value of slew rate limit object.
             * @param input: new value.
             * @param dt: [sec]. Time step duration for update.
             * @return output of limitation.
             */
            float updateByTime(const float &input, const float &dt);

            /**
             * Update output value of slew rate limit object.
             * @param input: new value.
             * @param frq: [Hz]. Frequency updatation.
             * @return output of limitation.
             */
            float updateByFrequency(const float &input, const float &frq);

            // Clear any buffer on variables.
            void clear(void); 
        
        private:

            // Buffer for input of slewRate value that used in update method.
            float _input;     

            // slew rate parameter value. *Hint: 0 value means disable it.
            float SLEWRATE;        

            // calculate and update limited output by input signal and derivative of input signal.
            void _calculate(const float &input, const float &derivative);
    };
}
// #######################################################################################
// LPF and LPFLimit class:

namespace AngleControllerNamespace
{
    // Low pass filter class
    class LPF
    {
        public:
            
            struct LPF_Parameters
            {
                float FRQ;                   // [Hz]. Cut off frequency parameter of low pass filter. *Hint: 0 value means disable filter.
            }parameters;

            // Default constructor. Init some variables.
            LPF();

            // Buffer memory for output of lowpass filter.
            float output;              

            /**
             * Low pass filter updatation. Calculate output signal relative to input signal and time step.
             * Store value of filtered output signal.
             * @param input: Value of input signal.
             * @param dt: Time step duration. [sec].
             * @return output of filter.
             */
            virtual float updateByTime(const float &input, const float &dt);

            /**
             * Low pass filter updatation. Calculate output signal relative to input signal and frequency of update step.
             * Store value of filtered output signal.
             * @param input: Value of input signal.
             * @param frq: Frequency updatation step. [Hz].
             * @return output of filter
             */
            virtual float updateByFrequency(const float &input, const float &frq);

            /**
             * Set Cut off frequency of low pass filter.
             * @param frq: Cut off frequency. [Hz]
             * @return true if successed.
             *  */  
            bool setFrequency(const float &frq);

            /**
             * Set output directely without filter.
             * Use this function for init value of output signal.
             */
            void setOutputDirect(const float &data);

            // Clear any buffer on variables.
            void clear(void); 

        protected:

            /**
             * Low Pass filter gain:
             * Filter in continuse space: G(s) = output/input = 1/(T*s + 1)
             * T: Time constant for lowpass filter.
             * Filter cut off frequency (frq_c) = 1/(2pi*T)
             * alpha gain = 1/( 1 + 1/(dt/T) ) = 1/( 1 + 1/(2*pi*frq_c*dt) )
             * dt: Time step duration. [sec]
             * output_(n) = (1 - alpha) * output_(n-1) + alpha * input_(n)
             *  */ 
            float _alpha;           

    };

    // Low pass filter class with input limitation
    class LPFLimit: public LPF
    {
        public:

            struct LPFLimit_Parameters : public LPF_Parameters
            {
                bool UP_LIMIT_EN;           // Input upper limitation enable/disable.
                bool DOWN_LIMIT_EN;         // Input Down limitation enable/disable.
                float UP_LIMIT;             // Input upper limit value.
                float DOWN_LIMIT;           // Input down limit value.
            }parameters;

            // Default constructor. Init some varibles.
            LPFLimit();

            /**
             * Low pass filter updatation. Calculate output signal relative to input signal and time step.
             * Store value of filtered output signal.
             * @param input: Value of input signal.
             * @param dt: Time step duration. [sec].
             * @return output of filter.
             */
            virtual float updateByTime(const float &input, const float &dt) override;

            /**
             * Low pass filter updatation. Calculate output signal relative to input signal and frequency of update step.
             * Store value of filtered output signal.
             * @param input: Value of input signal.
             * @param frq: Time step duration. [Hz].
             * @return output of filter.
             */
            virtual float updateByFrequency(const float &input, const float &frq) override;

            /** 
             * Set input limitation values.
             * @param limit: up/down limitation value. up_limit = +abs(limit), down_limit = -abs(limit).
            */   
            void setInputLimit(float limit);

            /** 
             * Set input limitation values.
             * @param upLimit: up limitation value.
             * @param downLimit: down limitation value.
            */ 
            void setInputLimit(float upLimit, float DownLimit);

        private:

            /**
             * Limit input data.
             * @return Limited value.
             */
            float _limitInput(const float &input);
    };

}

// #######################################################################################
// PID class:

namespace AngleControllerNamespace
{

    class PController
    {
        public:

            // Last error for controller.
            std::string errorMessage;

            // Output signal value of controller. [output_unit]
            float output;          

            struct PController_Parameters
            {
                // [-]. P or proportional gain. It must be positive.
                float P;            

                // [-]. Feedforward gain. It must be positive.
                float FF;           

                // [Output_unit]. Max feedforward magnitude value. *Hint: zero value means disable it. It must be positeve.
                float FFMAX;        
            }parameters;

            // Default constructor. Init some variables.
            PController();

            // Get output data signal from controller.
            float getOutput(void);

            // Return controller error value. e = desiredState - state.
            float getError(void);

            // ------------------------------------------
            // Virtual functions:

            /**
             * Update and calculate output signal of controller.
             * @param desiredState: Desired state for controller.
             * @param state: Feedback state from plant system.
             * @return output value of controller.
             */
            virtual float update(const float &desiredState, const float &state);

            /* 
                Check params for valid values.
                Calculate some variavles depends on other Parameters.
                Clear any buffer.
                @return true if successed.
            */
            virtual bool init(void); 

            // Clear any buffer on variables.
            virtual void clear(void); 

        protected:

            // [Input_unit]. Input error signal value. _e = desiredState - state;        
            float _e;       

        private:

            /**
             * Check params for valid values.
             * @return true if successed.
             *  */ 
            bool _checkParams(void);
    };

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    class PIController : public PController
    {
        public:

            struct PIController_Parameters : public PController_Parameters
            {
                // [-]. I gain for integral controller. It must be positive.
                float I;                

                // [outputs_unit]. Max magnitude output value of integral controller.  It must be positive.
                float IMAX;             
            }parameters;

            // Default constructor. Init some variables.
            PIController();

            // ------------------------------------------
            // Virtual functions:

            /* 
                Check params for valid values.
                Calculate some variavles depends on other Parameters.
                Clear any buffer.
                @return true if successed.
            */
            virtual bool init(void) override;  

            // Clear any buffer on variables.
            virtual void clear(void) override; 

            /**
             * Update and calculate output signal of controller using time step.
             * @param desiredState: Desired state for controller.
             * @param state: Feedback state from plant system.
             * @param dt: Time step duration. [sec]
             * @return output value of controller.
             */
            virtual float updateByTime(const float &desiredState, const float &state, const float &dt);

            /**
             * Update and calculate output signal of controller using update frequency.
             * @param desiredState: Desired state for controller.
             * @param state: Feedback state from plant system.
             * @param frq: Frequency updatation. [Hz]
             * @return output value of controller.
             */
            virtual float updateByFrequency(const float &desiredState, const float &state, const float &frq);

        protected:

            float _ISum;        // buffer memory for output of integral summation controller.

        private: 

            /**
             * Check params for valid values.
             * @return true if successed.
             *  */ 
            bool _checkParams(void);

    };

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    class PIDController : public PIController
    {
        public:

            struct PIDController_Parameters : public PIController_Parameters
            {
                // [-]. D or derivative gain for controller. it must be positive.
                float D;                    

                // [Hz]. Filter frequency for derivative controller.
                float FLTD;                 
            }parameters;

            // Default constructor. Init some variables.
            PIDController();

            // ------------------------------------------
            // Virtual functions:

            /**  
                Check params for valid values.
                Calculate some variavles depends on other Parameters.
                Clear any buffer.
                @return true if successed.
            */
            virtual bool init(void) override;  

            // Clear any buffer on variables.
            virtual void clear(void) override; 

            /**
             * Update and calculate output signal of controller using time step.
             * @param desiredState: Desired state for controller.
             * @param state: Feedback state from plant system.
             * @param dt: Time step duration. [sec]
             * @return output value of controller.
             */
            virtual float updateByTime(const float &desiredState, const float &state, const float &dt) override;

            /**
             * Update and calculate output signal of controller using update frequency.
             * @param desiredState: Desired state for controller.
             * @param state: Feedback state from plant system.
             * @param frq: Frequency updatation. [Hz]
             * @return output value of controller.
             */
            virtual float updateByFrequency(const float &desiredState, const float &state, const float &frq) override;

        protected:

            AngleControllerNamespace::LPF _LPFD;     // LPF object for low pass filter of derivative controller.
            float _ePast;                            // buffer memory for past error.

        private:

            /**
             * Check params for valid values.
             * @return true if successed.
             *  */ 
            bool _checkParams(void);
    };

}

// #######################################################################################
// Map class:

namespace AngleControllerNamespace
{
    class MAP
    {
        public:

            std::string errorMessage;
            
            AngleControllerNamespace::Outputs outputs;       // Output object

            // -------------------------
            // MAP parameters:

            struct MAP_Parameters
            {
                // PrimaryOutput deadzone value.
                float PRIM_DEADZONE;

                // PrimaryOutput max limitation value.
                float PRIM_MAX;

                // PrimaryOutput range for set map scale.
                float PRIM_RANGE;

                // SecondaryOutput range for set map scale.
                float SECON_RANGE;

                // Output polarity of mapping. 0: normal, 1: reverse.
                uint8_t DIR_POL;
            }parameters;

            // -------------------------

            MAP()
            {
                parameters.DIR_POL = 0;
                parameters.PRIM_DEADZONE = 0;
                parameters.PRIM_MAX = 0;
                parameters.PRIM_RANGE = 0;
                parameters.SECON_RANGE = 0;

                _m_map = 1;
                _positiveDir = 1;

                clear();
            }

            /**  
                Check params for valid values.
                Calculate some variavles depends on other Parameters.
                Clear any buffer.
                @return true if successed.
            */
            bool init(void);

            /**
             * Update output value of map.
             * @param input: output signal from controller. 
             * @return outputs object of the map.
             */
            AngleControllerNamespace::Outputs update(const float &input);

            // Clear any buffer on variables.
            void clear(void);

        private:

            // Slope of line for primary input/output mapping.
            float _m_map;

            // Positive vlaue depends on output polarity.
            // Output polarity 0 -> _positiveDir = 1; Output polarity 1 -> _positiveDir = -1; 
            int8_t _positiveDir;

            /**
             * Check params for valid values.
             * @return true if successed.
             *  */ 
            bool _checkParams(void);
    };
}

// #######################################################################################
// AngleController_SingleDrive class:

class AngleController_SingleDrive
{
    public:

        // parameter struct
        AngleControllerNamespace::SingleDriveParams parameters;  

        // Outputs struct
        AngleControllerNamespace::Outputs outputs;

        // Inputs struct
        AngleControllerNamespace::Inputs inputs;

        // Last error for object.
        std::string errorMessage;

        /*
            Constructor. Init parameters by default values.
        */
        AngleController_SingleDrive();

        // Check certain params for allowable values.
        // @return true if successed.
        bool checkParams(const AngleControllerNamespace::SingleDriveParams &params);

        // Set controller mode. 0:Angle, 1:Rate.
        bool setMode(const uint8_t &mode);

        // Set inputs data for controller.
        void setInputs(const AngleControllerNamespace::Inputs &data);
        
        // Return instance frequency of update controller variable. It can changed by rate of call the update function.
        float getFrq(void);    

        // Return controller error value. error_angle or error_rate depends on controller mode.
        float getError(void);  

        // Set method for parameters value.
        bool setParams(const AngleControllerNamespace::SingleDriveParams &parameters);

        // Get method for return parameters value:
        void getParams(AngleControllerNamespace::SingleDriveParams *params);

        // ------------------------------------------
        // Virtual functions:

        /* 
            Check params for allowable values.
            Calculate some variavles depends on other Parameters.
            @return true if successed.
        */
        virtual bool init(void);  

        /** 
            Update controller outputs.
            @param T_now: Time at the update moment. [us]
            @return true if successed.
        */
        virtual bool update(const uint64_t &T_now);

        // Clear any buffer date on variables.
        virtual void clear(void);      

    protected:

        // Error angle of controller.
        float _eAngle;

        // Error rate of controller.
        float _eRate;

        // [Hz]. Measured controller Loop Frequency for update. Depends on executation of update function frequency. 
        float _frq;    

        // [us]. Target Time duration for control loop update. Depends on params.FRQ.                
        float _targetTime;              

        uint64_t _T[2];                // Time Vector:{T_now, T_Last}.  [us]

        uint64_t _dT;                  // Time Deference :{T_now-T_last}. [us]

        // Controller mode. 0:Angle, 1:Rate.
        uint8_t _mode; 

        // Map object for controller output mapping.
        AngleControllerNamespace::MAP _map; 

        // LPF object for low pass filter of target rate signal.
        AngleControllerNamespace::LPFLimit _LPFT;

        // LPF pbject for low pass filter of output signal.
        AngleControllerNamespace::LPFLimit _LPFO;

        // PID controller object for rate control.
        AngleControllerNamespace::PIDController _PIDRate;

        // Slew rate object for angle acceleration limitation.
        AngleControllerNamespace::LimitSlewRate _limitSlewRate;

};

// #######################################################################################
// AngleController_DualDriveEq class:

class AngleController_DualDriveEq : public AngleController_SingleDrive
{
    public:

        // parameter struct
        AngleControllerNamespace::DualDriveEqParams parameters;  

        // Outputs struct of slave
        AngleControllerNamespace::Outputs outputsSlave;

        /*
            Constructor. Init parameters by default values.
        */
        AngleController_DualDriveEq();

        // Check certain params for allowablw values.
        // @return true if successed.
        bool checkParams(const AngleControllerNamespace::DualDriveEqParams &params);


        // Get method for return parameters value:
        void getParams(AngleControllerNamespace::DualDriveEqParams *params);
                                                         
        // Set method for parameters value.
        bool setParams(const AngleControllerNamespace::DualDriveEqParams &parameters);

        // -------------------------------------------
        // Virtual functions:

        /* 
            Check params for allowable values.
            Calculate some variavles depends on other Parameters.
            @return true if successed.
        */
        virtual bool init(void) override;  

        /** 
            Update controller outputs.
            @param T_now: Time at the update moment. [us]
            @return true if successed.
        */
        virtual bool update(const uint64_t &T_now) override; 

        // Clear any buffer date on variables.
        virtual void clear(void) override;      

    protected:

        // Map object for controller output mapping for slave driver.
        AngleControllerNamespace::MAP _mapSlave; 

        // PID controller object for rate control for slave driver.
        AngleControllerNamespace::PIDController _PIDRateSlave;

        // PID controller object for rate control for equalizer.
        AngleControllerNamespace::PIDController _PIDEq;

};


#endif
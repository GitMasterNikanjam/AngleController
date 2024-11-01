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

/**
 * @namespace AngleControllerNamespace
 */
namespace AngleControllerNamespace
{
    /**
     * @defgroup AngleController_Essential_Structure_Variables AngleController Essential Structure Variables
     * @{
     */

    /**
     * @struct SingleDriveParams
     * @brief Full parameters struct for SingleDrive AngleController
     */
    struct SingleDriveParams
    {
        /**
         * #### Param #1
         * @brief Angle controller P gain. [-]
         * @note - Convert error angle [deg] to internal demanded rate [deg/s].
         */
        float ANG_P; 

        /**
         * #### Param #2
         * @brief Rate controller P gain. [-]
         * @note - Convert error rate [deg/s] to primary output control signal.
         */       
        float RAT_P; 

        /**
         * #### Param #3
         * @brief Rate controller I gain. [-]
         * @note - Convert error rate [deg/s] to primary output control signal.
         */           
        float RAT_I;  

        /**
         * #### Param #4
         * @brief Rate controller D gain. [-]
         * @note - Convert error rate [deg/s] to primary output control signal.
         *  */                 
        float RAT_D; 

        /**
         * #### Param #5
         * @brief Controller feed forward gain type1. (For angle controller). [-]
         * @note - Convert target rate [deg/s] at angle mode to internal demanded rate [deg/s].
         */          
        float FF1; 

        /**
         * #### Param #6
         * @brief Controller feed forward gain type2. (for rate controller)
         * @note - Convert internal demanded rate [deg/s] to primary output control signal.
         */               
        float FF2;  

        /**
         * #### Param #7
         * @brief Lowpass filter frequency for target rate. [Hz]
         * @note - Value 0 disabled filter.
         */              
        float FLTT;                 
        
        /**
         * #### Param #8
         * @brief Lowpass filter frequency for derivative of rate. [Hz]
         * @note - Value 0 disabled filter. 
         */
        float FLTD;                  
        
        /**
         * @brief Lowpass filter frequency for controller output. [Hz]
         * @note - Value 0 disabled filter.
         */
        float FLTO;                 
        
        /**
         * #### Param #10
         * @brief Upper limitation angle for controller. [deg]
         * @note - It works at all modes.
         */
        float ANG_UP_LIMIT;          
        
        /**
         * #### Param #11
         * @brief Down limitation angle for controller. [deg]
         * @note - It works at all modes.
         */
        float ANG_DOWN_LIMIT;       
        
        /**
         * #### Param #12
         * @brief Max integral control output for rate controller output. [-]
         * @note - Value 0 means Disabled.
         */
        float RAT_IMAX;             
        
        /**
         * #### Param #13
         * @brief Max internal demanded rate rate. [deg/s]
         * @note - Value 0 means Disabled.
         */
        float RAT_MAX;              
        
        /**
         * #### Param #14
         * @brief Target rate magnitude for fast movement state. [deg/s]
         */
        float RAT_FAST;             
        
        /**
         * #### Param #15
         * @brief Target rate magnetude for slow movement state. [deg/s]
         */
        float RAT_SLOW;             
        
        /**
         * #### Param #16
         * @brief Acceleration/Decleration magnetude limitation. [deg/s^2]
         * @note
         */
        float RAT_SLEWRATE;         
        
        /**
         * #### Param #17
         * @brief Max feedForward control output for feedforward type1. [deg/s]
         * @note - Value 0 means Disabled.
         */
        float FF1_MAX;              
        
        /**
         * #### Param #18
         * @brief Max feedforward control output for feedforward type2. [-]
         * @note - Value 0 means Disabled.
         */
        float FF2_MAX;              

        /**
         * #### Param #19
         * @brief  Controller loop frequency. [Hz]
         * @note - Value: 0 -> Free Frequency-> time and frequency changed when controller become update.
         * @note - Value: non zero -> Lock Frequency-> time and frequency of controller update locked to FRQ value.
         *  */ 
        float FRQ;                      

        /**
         * #### Param #20
         * @brief Controller update mode. [-]
         * @note - 0: Manual update-> Controller output updated when call update function.
         * @note - 1: Auto update-> Controller output updated automatically at separated thread. 
         * @note - For Auto update mode **FRQ** parameter must be non zero, otherwise Auto update mode not worked.
         */
        uint8_t UPDATE_MODE;
        
        /**
         * #### Param #21
         * @brief PrimaryOutput deadzone value for maped controller output.
         * @note - Value 0 means Disabled.
         */
        float PRIM_DEADZONE;       
        
        /**
         * #### Param #22
         * @brief Max PrimaryOutput value for maped controller output.
         * @note - Value 0 means Disabled.
         */
        float PRIM_MAX;            
        
        /**
         * #### Param #23
         * @brief Range PrimaryOutput value for maped controller output.
         * @note - Value 0 means Disabled.
         */
        float PRIM_RANGE;          
        
        /**
         * #### Param #24
         * @brief Range SecondaryOutput value for maped controller output.
         * @note - Value 0 means Disabled.
         */
        float SECON_RANGE;         
        
        /**
         * #### Param #25
         * @brief Controller direction polarity. normal direction:0, reverse direction:1
         */
        uint8_t DIR_POL;           
    };

    /**
     * @struct DualDriveParams
     * @brief Full parameters struct for simple DualDrive AngleController without equalizer.
     */
    struct DualDriveParams
    {
        /**
         * @brief Basic parameters of SingleDriveParams.
         */
        SingleDriveParams basicParams;

        /**
         * @brief BIAS magnitude value for primaryOutput bias. [-]
         */
        float BIAS;                         

        /**
         * @brief Controller direction polarity for slave driver. normal direction:0, reverse direction:1
         */
        uint8_t SLAVE_DIR_POL;              
    };

    /**
     * @struct DualDriveEqParams
     * @brief Full parameters struct for advanced DualDrive AngleController with equalizer.
     */
    struct DualDriveEqParams
    {
        /**
         * @brief Basic parameters of SingleDriveParams
         */
        SingleDriveParams basicParams;      

        /**
         * @brief BIAS magnitude value for primaryOutput bias. [-]
         */
        float BIAS;              

        /** 
         * @brief Rate controller P gain for equalization. [-]
         */       
        float RAT_EQ_P;          

        /**
         * @brief Rate controller I gain for equalization. [-]
         */         
        float RAT_EQ_I;             

        /**
         * Rate controller D gain for equalization. [-]
         */    
        float RAT_EQ_D;      

        /**
         * @brief Max integral control output for equalization controller output. [-]
         * @note - 0 Value is equal to Disable.
         */            
        float RAT_EQ_IMAX;  

        /**
         * @brief Lowpass filter frequency for derivative of rate for equalization.
         * @note - 0 value is equal to disable. 
         */            
        float FLTDQ;     

        /**
         * @brief Controller direction polarity for slave driver. normal direction:0, reverse direction:1 
         */             
        uint8_t SLAVE_DIR_POL;              
    };

    /**
     * @struct Inputs
     * @brief Full input variables for AngleController
     */
    struct Inputs
    {
        float ang;                          ///< [deg]. Angle value of plant.
        float rat_1;                        ///< [deg/s]. Rate observer value of plant from master driver.
        float rat_2;                        ///< [deg/s]. Rate observer value of plant from slave driver.
        float angDes;                       ///< [deg]. Desired angle value for controller. 
        float ratDes;                       ///< [deg/s]. Desired rate value for controller.
    };

    /**
     * @struct Outputs
     * @brief Data structure for output values of any controllers.
     */
    struct Outputs
    {
        float primaryOutput;                ///< [-]. primaryOutput value output from AngleController.
        float secondaryOutput;              ///< [-]. secondaryOutput value output from AngleController. Depends on primaryOutput value.
        int8_t dir;                         ///< [-]. Direction signal value output from AngleController. {-1, 0, 1}
    };

/** @} */  // end of AngleController_Essential_Structs

}

// ##################################################################################
// General Functions:

/**
 * @namespace AngleControllerNamespace
 */
namespace AngleControllerNamespace
{
    /**
     * @defgroup AngleController_General_Functions AngleController_General_Functions
     * @{
     */

    /** 
     * @brief General upper limit function.
     * @param input: input data.
     * @param upLimit: up limitation value.
     * @return Limited value. 
    */ 
    float limitUp(const float &input, const float &upLimit);

    /** 
     * @brief General lower limit function.
     * @param input: input data.
     * @param downLimit: down limitation value.
     * @return Limited value. 
    */ 
    float limitDown(const float &input, const float &downLimit);

    /** 
     * @brief General limit function.
     * @param input: input data.
     * @param upLimit: up limitation value.
     * @param downLimit: down limitation value.
     * @return Limited value. 
    */ 
    float limit(const float &input, const float &upLimit, const float &downLimit);

    /** 
     * @brief General limit function.
     * @param input: input data.
     * @param limit: up/down limitation value. up_limit = +abs(limit), down_limit = -abs(limit).
     * @return Limited value. 
    */         
    float limit(const float &input, const float &limit);

    /** @} */  // end of AngleController_General_Functions

}

// #######################################################################################
// LimitSlewRate class:
// This class is for change rate limit.

/**
 * @defgroup AngleController_Special_classes  AngleController Special classes
 */

/**
 * @namespace AngleControllerNamespace
 */
namespace AngleControllerNamespace
{
    /**
     * @class LimitSlewRate
     * @ingroup AngleController_Special_classes
     * @brief Limit for slew rate class.  
     */
    class LimitSlewRate
    {
        public:

            /**
             * @brief Default constructor. Init values and parameters.
             */
            LimitSlewRate()
            {   
                _input = 0;
                output = 0;
                SLEWRATE = 0;
            }

            // Buffer for output of slewRate value that used in update method.
            float output;                           

            /**
             * @brief Set slew rate parameter value.
             * @param vlaue: must be positive.
             * @return true if successed.
             */
            bool setLimit(float value);

            /**
             * @brief Update output value of slew rate limit object.
             * @param input: new value.
             * @param dt: [sec]. Time step duration for update.
             * @return output of limitation.
             */
            float updateByTime(const float &input, const float &dt);

            /**
             * @brief Update output value of slew rate limit object.
             * @param input: new value.
             * @param frq: [Hz]. Frequency updatation.
             * @return output of limitation.
             */
            float updateByFrequency(const float &input, const float &frq);

            /**
             * @brief Clear any buffer on variables.
             */
            void clear(void); 
        
        private:

            /**
             * @brief Buffer for input of slewRate value that used in update method.
             */
            float _input;     

            /**
             * @brief Slew rate parameter value. *Hint: 0 value means disable it.
             */
            float SLEWRATE;        

            /**
             * @brief Calculate and update limited output by input signal and derivative of input signal.
             */
            void _calculate(const float &input, const float &derivative, const float &frq);
    };
}
// #######################################################################################
// LPF and LPFLimit class:

namespace AngleControllerNamespace
{
    /**
     * @class LPF
     * @ingroup AngleController_Special_classes
     * @brief Low pass filter class
     */
    class LPF
    {
        public:
            
            /**
             * @brief Parameters structure.
             */
            struct LPF_Parameters
            {
                float FRQ;                   // [Hz]. Cut off frequency parameter of low pass filter. *Hint: 0 value means disable filter.
            }parameters;

            /**
             * @brief Default constructor. Init some variables.
             */
            LPF();

            /**
             * @brief Buffer memory for output of lowpass filter.
             */
            float output;              

            /**
             * @brief Low pass filter updatation. Calculate output signal relative to input signal and time step.
             * Store value of filtered output signal.
             * @param input: Value of input signal.
             * @param dt: Time step duration. [sec].
             * @return output of filter.
             */
            virtual float updateByTime(const float &input, const float &dt);

            /**
             * @brief Low pass filter updatation. Calculate output signal relative to input signal and frequency of update step.
             * Store value of filtered output signal.
             * @param input: Value of input signal.
             * @param frq: Frequency updatation step. [Hz].
             * @return output of filter
             */
            virtual float updateByFrequency(const float &input, const float &frq);

            /**
             * @brief Set Cut off frequency of low pass filter.
             * @param frq: Cut off frequency. [Hz]
             * @return true if successed.
             *  */  
            bool setFrequency(const float &frq);

            /**
             * @brief Set output directely without filter.
             * Use this function for init value of output signal.
             */
            void setOutputDirect(const float &data);

            /**
             * @brief Clear any buffer on variables.
             */
            void clear(void); 

        protected:

            /**
             * @brief Low Pass filter gain.
             * @note - Filter in continuse space: G(s) = output/input = 1/(T*s + 1)
             * @note - T: Time constant for lowpass filter.
             * @note -  Filter cut off frequency (frq_c) = 1/(2pi*T)
             * @note -  alpha gain = 1/( 1 + 1/(dt/T) ) = 1/( 1 + 1/(2*pi*frq_c*dt) )
             * @note -  dt: Time step duration. [sec]
             * @note -  output_(n) = (1 - alpha) * output_(n-1) + alpha * input_(n)
             *  */ 
            float _alpha;           

    };

    /**
     * @class LPFLimit
     * @ingroup AngleController_Special_classes
     * @brief Low pass filter class with input limitation
     */
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

            /**
             * @brief Default constructor. Init some varibles.
             */
            LPFLimit();

            /**
             * @brief Low pass filter updatation. Calculate output signal relative to input signal and time step.
             * Store value of filtered output signal.
             * @param input: Value of input signal.
             * @param dt: Time step duration. [sec].
             * @return output of filter.
             */
            virtual float updateByTime(const float &input, const float &dt) override;

            /**
             * @brief Low pass filter updatation. Calculate output signal relative to input signal and frequency of update step.
             * Store value of filtered output signal.
             * @param input: Value of input signal.
             * @param frq: Time step duration. [Hz].
             * @return output of filter.
             */
            virtual float updateByFrequency(const float &input, const float &frq) override;

            /** 
             * @brief Set input limitation values.
             * @param limit: up/down limitation value. up_limit = +abs(limit), down_limit = -abs(limit).
            */   
            void setInputLimit(float limit);

            /** 
             * @brief Set input limitation values.
             * @param upLimit: up limitation value.
             * @param downLimit: down limitation value.
            */ 
            void setInputLimit(float upLimit, float DownLimit);

        private:

            /**
             * @brief Limit input data.
             * @return Limited value.
             */
            float _limitInput(const float &input);
    };

}

// #######################################################################################
// PID class:

namespace AngleControllerNamespace
{
    /**
     * @class PController
     * @ingroup AngleController_Special_classes
     * @brief General P controller class for any higher level controller.
     */
    class PController
    {
        public:

            /// @brief Last error for controller.
            std::string errorMessage;

            /// @brief Output signal value of controller. [output_unit]
            float output;          

            /**
             * @brief Parameters structure.
             */
            struct PController_Parameters
            {
                // [-]. P or proportional gain. It must be positive.
                float P;            

                // [-]. Feedforward gain. It must be positive.
                float FF;           

                // [Output_unit]. Max feedforward magnitude value. *Hint: zero value means disable it. It must be positeve.
                float FFMAX;        
            }parameters;

            /**
             * @brief Default constructor. Init some variables.
             */
            PController();

            /**
             * @brief Get output data signal from controller.
             */
            float getOutput(void);

            /**
             * @brief Return controller error value. e = desiredState - state.
             */
            float getError(void);

            // ------------------------------------------
            // Virtual functions:

            /**
             * @brief Update and calculate output signal of controller.
             * @param desiredState: Desired state for controller.
             * @param state: Feedback state from plant system.
             * @return output value of controller.
             */
            virtual float update(const float &desiredState, const float &state);

            /**
             * @brief Check params for valid values.
             * Calculate some variavles depends on other Parameters.
             * Clear any buffer.
             * @return true if successed.
             */
            virtual bool init(void); 

            /**
             * @brief Clear any buffer on variables.
             */
            virtual void clear(void); 

        protected:

            /// @brief [Input_unit]. Input error signal value. _e = desiredState - state;        
            float _e;       

        private:

            /**
             * @brief Check params for valid values.
             * @return true if successed.
             *  */ 
            bool _checkParams(void);
    };

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    /**
     * @class PIController
     * @ingroup AngleController_Special_classes
     * @brief General PI controller class for any higher level controller.
     */
    class PIController : public PController
    {
        public:

            /**
             * @brief Parameters structure.
             */
            struct PIController_Parameters : public PController_Parameters
            {
                // [-]. I gain for integral controller. It must be positive.
                float I;                

                // [outputs_unit]. Max magnitude output value of integral controller.  It must be positive.
                float IMAX;             
            }parameters;

            /**
             * @brief Default constructor. Init some variables.
             */
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

    /**
     * @class PIDController
     * @ingroup AngleController_Special_classes
     * @brief General PID controller class for any higher level controller.
     */
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

            AngleControllerNamespace::LPF _LPFD;          // LPF object for low pass filter of derivative controller.
            float _ePast;                                 // buffer memory for past error.

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
    /**
     * @class MAP
     * @ingroup AngleController_Special_classes
     * @brief Map class for convert outputs of any controller signals to custom acceptabe values.
     */
    class MAP
    {
        public:

            /**
             * @brief Last error accured for object.
             */
            std::string errorMessage;
            
            /**
             * @brief Data structure for output values of mapping signal controller.
             */
            AngleControllerNamespace::Outputs outputs;     

            // -------------------------
            // MAP parameters:

            /**
             * @struct ParametersStructure
             * @brief Data structure for parameters of MAP object.
             */
            struct ParametersStructure
            {
                /**
                 * @brief PrimaryOutput deadzone value.
                 */
                float PRIM_DEADZONE;

                /**
                 * @brief PrimaryOutput max limitation value.
                 */
                float PRIM_MAX;

                /**
                 * @brief PrimaryOutput range for set map scale.
                 */
                float PRIM_RANGE;

                /**
                 * @brief SecondaryOutput range for set map scale.
                 */
                float SECON_RANGE;

                /**
                 * @brief Output polarity of mapping. 0: normal, 1: reverse.
                 */
                uint8_t DIR_POL;
            }parameters;

            // -------------------------

            /**
             * @brief Default Constructor. Initial parameters and vriables by default values.
             */
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
             * @brief Check params for valid values.
             * Calculate some variavles depends on other Parameters.
             * Clear any buffer.
             * @return true if successed.
             */
            bool init(void);

            /**
             * @brief Update output value of map.
             * @param input: output signal from controller. 
             * @return outputs object of the map.
             */
            AngleControllerNamespace::Outputs update(const float &input);

            /**
             * @brief Clear any buffer on variables.
             */
            void clear(void);

        private:

            /**
             * @brief Slope of line for primary input/output mapping.
             */
            float _m_map;

            /**
             * @brief Positive vlaue depends on output polarity.
             * @note - Output polarity 0 -> _positiveDir = 1
             * @note - Output polarity 1 -> _positiveDir = -1;
             */
            int8_t _positiveDir;

            /**
             * @brief Check params for valid values.
             * @return true if successed.
             *  */ 
            bool _checkParams(void);
    };
}

// #######################################################################################
// AngleController_SingleDrive class:

/**
 * @defgroup AngleController_Public_Classes  AngleController Public Classes
 */

/**
 * @class AngleController_SingleDrive
 * @ingroup AngleController_Public_Classes
 * @brief Controller class for single drive system.
 */
class AngleController_SingleDrive
{
    public:

        // parameter struct
        AngleControllerNamespace::SingleDriveParams parameters;  

        // Outputs struct
        AngleControllerNamespace::Outputs outputs;

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

        // Inputs struct
        AngleControllerNamespace::Inputs _inputs;

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

/**
 * @class AngleController_DualDriveEq
 * @ingroup AngleController_Public_Classes
 * @brief Controller class for dual drive (with equalizer) system.
 */
class AngleController_DualDriveEq : public AngleController_SingleDrive
{
    public:

        /**
         * @brief Data structure for dual drive (with equalizer) controller parameters. 
         */
        AngleControllerNamespace::DualDriveEqParams parameters;  

        /**
         * @brief Data structure for output values of slave driver.
         */
        AngleControllerNamespace::Outputs outputsSlave;

        /**
         * @brief Default Constructor. Initial parameters and variables by default values.
         */
        AngleController_DualDriveEq();

        /**
         * @brief Check certain params for allowable values.
         * @return true if successed.
         */
        bool checkParams(const AngleControllerNamespace::DualDriveEqParams &data);

        /**
         * @brief Get method for return parameters value.
         */
        void getParams(AngleControllerNamespace::DualDriveEqParams *data);

        /**
         * @brief Set method for parameters value.
         * @return true if successed.
         *  */                                                 
        bool setParams(const AngleControllerNamespace::DualDriveEqParams &data);

        // -------------------------------------------
        // Virtual functions:

        /**
         * @brief Initial controller. Check parameters value.
         * Calculate some variables depends on other Parameters.
         * @return true if successed.
         */
        virtual bool init(void) override;  

        /**
         * @brief Update controller outputs.
         * @param T_now: Time at the update moment. [us]
         * @return true if successed.
         */
        virtual bool update(const uint64_t &T_now) override; 

        /**
         * @brief Clear any buffer date on variables.
         */
        virtual void clear(void) override;      

    protected:

        /**
         * @brief Map object for controller output mapping for slave driver.
         */
        AngleControllerNamespace::MAP _mapSlave; 

        /**
         * @brief PID controller object for rate control for slave driver.
         */
        AngleControllerNamespace::PIDController _PIDRateSlave;

        /**
         * @brief PID controller object for rate control for equalizer.
         */
        AngleControllerNamespace::PIDController _PIDEq;
};

#endif
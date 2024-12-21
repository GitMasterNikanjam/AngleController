#ifndef ANGLE_CONTROLLER_H
#define ANGLE_CONTROLLER_H

/**
 * @defgroup AngleController_Essential_Structure_Variables AngleController Essential Structure Variables
 */

/**
 * @defgroup AngleController_General_Functions AngleController General Functions
 */

/**
 * @defgroup AngleController_Classes  AngleController classes
 */

// ########################################################
// Include libraries:

#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

using namespace std;

// ###############################################################################################
// AngleController_SingleDrive(Mode:0:Direct): Controller Shematic:  

/**  
*                                                      |-> PrimaryOutput
*                                                      |-> SecondaryOutput 
*   Throttel->Limit->Lowpass_Filter(FLTO)->Map_Function|-> Direction
*                                                                                                                            
*/

// ################################################################################################
// AngleController_SingleDrive(Mode:1:Angle): Controller Shematic:  

/**  
*                                                                                                                    |->Limit(*FF2) ------------------------------------->|(+)|                                           |-> PrimaryOutput
*   Limit(Rate_des[deg/s]*FF1)----------------------------------->|(+)|                                              |       |*KP_rate ---------------------------------->|(+)|                                           |-> SecondaryOutput 
*   Limit(Angle_des[deg])-Angle[deg]->|*KP_angle----------------->|(+)|->Limit->Lowpass_Filter(FLTT)->SlewRateLimit->|-Rate->|Integral_Limit(*KI_rate) ------------------>|(+)|->Limit->Lowpass_Filter(FLTO)->Map_Function|-> Direction
*                                     |Integral_Limit(*KI_angle)->|(+)|                                                      |Derivative(*KD_rate)->Lowpass_Filter(FLTD)->|(+)|                                           
*/

// #################################################################################################
// AngleController_SingleDrive(Mode:2:Rate): Controller Shematic:  

/**  
*                                                                |->Limit(*FF2) ------------------------------------->|(+)|                                           |-> PrimaryOutput
*                                                                |       |*KP_rate ---------------------------------->|(+)|                                           |-> SecondaryOutput 
*   Rate_des[deg/s]->Limit->Lowpass_Filter(FLTT)->SlewRateLimit->|-Rate->|Integral_Limit(*KI_rate) ------------------>|(+)|->Limit->Lowpass_Filter(FLTO)->Map_Function|-> Direction
*                                                                        |Derivative(*KD_rate)->Lowpass_Filter(FLTD)->|(+)|                                           
*/


// ######################################################################################################
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

/**
 * @brief No mode for angle controller.
 * @note The controller is inactive in this mode.
 *  */  
#define AngleController_Mode_None                0

/**
 * @brief Direct mode for angle controller.
 * @note - In direct control mode, the value is directly transferred to the primary value of the output signal controller.
 * 
 * @note - The output limitation, output filter frequency, and controller map functions are active in this mode.
 * 
 * @note - The desired angle and rate are neglected in this mode.
 */
#define AngleController_Mode_Direct              1

/**
 * @brief The angle mode for angle controller.
 * @note - In this mode, the controller tries to control the angle value of the system. 
 * 
 * @note - The desired rate is neglected in this mode.
 */
#define AngleController_Mode_Angle               2

/**
 * @brief The rate mode for angle controller.
 * @note - In this mode, the controller tries to control the rate value of the system.
 * 
 * @note - The desired position is neglected in this mode.
 */
#define AngleController_Mode_Rate                3

/**
 * @brief The tracking mode for angle controller.
 * @note - In this mode, the controller tries to tracking the desired position and rate.
 */
#define AngleController_Mode_Tracking            4

// #####################################################
// AngleController essential struct variables:

/**
 * @namespace AngleControllerNamespace
 */
namespace AngleControllerNamespace
{
    /**
     * @struct SingleDriveParams
     * @brief Full parameters struct for SingleDrive AngleController
     * @ingroup AngleController_Essential_Structure_Variables
     */
    struct SingleDriveParams
    {
        /**
         * @brief Default constructor. Init parameters value to default.
         */
        SingleDriveParams();
        
        /**
         * #### Param #1
         * @brief Angle controller P gain. [-]
         * @note - Convert error angle [deg] to internal demanded rate [deg/s].
         */
        float ANG_P; 

        /**
         * #### Param #2
         * @brief Angle controller I gain. [-]
         * @note - Convert error Integral of error [deg] to internal demanded rate [deg/s].
         */    
        float ANG_I;

        /**
         * #### Param #3
         * @brief Angle controller IEXP gain for dynamic exponentional integral gain. [-]
         */    
        float ANG_IEXP;

        /**
         * #### Param #4
         * @brief Angle controller IZONE gain for zone of activeted integral controller. [-]
         */    
        float ANG_IZONE;

        /**
         * #### Param #5
         * @brief Rate controller P gain. [-]
         * @note - Convert error rate [deg/s] to primary output control signal.
         */       
        float RAT_P; 

        /**
         * #### Param #6
         * @brief Rate controller I gain. [-]
         * @note - Convert error rate [deg/s] to primary output control signal.
         */           
        float RAT_I;  

        /**
         * #### Param #7
         * @brief Rate controller D gain. [-]
         * @note - Convert error rate [deg/s] to primary output control signal.
         *  */                 
        float RAT_D; 

        /**
         * #### Param #8
         * @brief Controller feed forward gain type1. (For angle controller). [-]
         * @note - Convert target rate [deg/s] at angle mode to internal demanded rate [deg/s].
         */          
        float FF1; 

        /**
         * #### Param #9
         * @brief Controller feed forward gain type2. (for rate controller)
         * @note - Convert internal demanded rate [deg/s] to primary output control signal.
         */               
        float FF2;  

        /**
         * #### Param #10
         * @brief Lowpass filter frequency for target rate. [Hz]
         * @note - The value of 0 meanse it is disabled.
         */              
        float FLTT;                 
        
        /**
         * #### Param #11
         * @brief Lowpass filter frequency for derivative of rate. [Hz]
         * @note - The value of 0 meanse it is disabled.
         */
        float FLTD;                  
        
        /** #### Param #12
         * @brief Lowpass filter frequency for controller output. [Hz]
         * @note - The value of 0 meanse it is disabled.
         */
        float FLTO;                 
        
        /**
         * #### Param #13
         * @brief Enable/Disable angle limitation function.
         * @note - It works at all modes.
         */
        bool ANG_LIMIT_ENA;  

        /**
         * #### Param #14
         * @brief Upper limitation angle for controller. [deg]
         * @note - It works at all modes.
         */
        float ANG_UP_LIMIT;          
        
        /**
         * #### Param #15
         * @brief Down limitation angle for controller. [deg]
         * @note - It works at all modes.
         */
        float ANG_DOWN_LIMIT;       
        
        /**
         * #### Param #16
         * @brief Max integral control output for angle controller output. [deg/s]
         * @note - The value of 0 meanse it is disabled.
         */
        float ANG_IMAX;  

        /**
         * #### Param #17
         * @brief Max integral control output for rate controller output. [-]
         * @note - The value of 0 meanse it is disabled.
         */
        float RAT_IMAX;             
        
        /**
         * #### Param #18
         * @brief Max internal demanded rate rate. [deg/s]
         * @note - The value of 0 meanse it is disabled.
         */
        float RAT_MAX;              
        
        /**
         * #### Param #19
         * @brief Target rate magnitude for fast movement state. [deg/s]
         */
        float RAT_FAST;             
        
        /**
         * #### Param #20
         * @brief Target rate magnetude for slow movement state. [deg/s]
         */
        float RAT_SLOW;             
        
        /**
         * #### Param #21
         * @brief Acceleration/Decleration magnetude limitation. [deg/s^2]
         * @note
         */
        float RAT_SLEWRATE;         
        
        /**
         * #### Param #22
         * @brief Max feedForward control output for feedforward type1. [deg/s]
         * @note - The value of 0 meanse it is disabled.
         */
        float FF1_MAX;              
        
        /**
         * #### Param #23
         * @brief Max feedforward control output for feedforward type2. [-]
         * @note - The value of 0 meanse it is disabled.
         */
        float FF2_MAX;              

        /**
         * #### Param #24
         * @brief  Controller loop frequency. [Hz]
         * @note - Value: 0 -> Free Frequency-> time and frequency changed when controller become update.
         * @note - Value: non zero -> Lock Frequency-> time and frequency of controller update locked to FRQ value.
         *  */ 
        float FRQ;                      

        /**
         * #### Param #25
         * @brief Controller update mode. [-]
         * @note - 0: Manual update-> Controller output updated when call update function.
         * @note - 1: Auto update-> Controller output updated automatically at separated thread. 
         * @note - For Auto update mode **FRQ** parameter must be non zero, otherwise Auto update mode not worked.
         */
        uint8_t UPDATE_MODE;
        
        /**
         * #### Param #26
         * @brief PrimaryOutput deadzone value for maped controller output.
         * @note - The value of 0 meanse it is disabled.
         */
        float PRIM_DEADZONE;       
        
        /**
         * #### Param #27
         * @brief Max PrimaryOutput value for maped controller output.
         * @note - The value of 0 meanse it is disabled.
         */
        float PRIM_MAX;            
        
        /**
         * #### Param #28
         * @brief Range PrimaryOutput value for maped controller output.
         * @note - The value of 0 meanse it is disabled.
         */
        float PRIM_RANGE;          
        
        /**
         * #### Param #29
         * @brief Range SecondaryOutput value for maped controller output.
         * @note - The value of 0 meanse it is disabled.
         */
        float SECON_RANGE;         
        
        /**
         * #### Param #30
         * @brief Controller direction polarity. normal direction:0, reverse direction:1
         * @warning The direction polarity of the output signal control must be set carefully, otherwise the system will malfunction.
         */
        uint8_t DIR_POL;           
    };

    /**
     * @struct DualDriveParams
     * @brief Full parameters struct for simple DualDrive AngleController without equalizer.
     * @ingroup AngleController_Essential_Structure_Variables
     */
    struct DualDriveParams
    {
        /**
         * @brief Default constructor. Init parameters value to default.
         */
        DualDriveParams();

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
     * @ingroup AngleController_Essential_Structure_Variables
     */
    struct DualDriveEqParams
    {
        /**
         * @brief Default constructor. Init parameters value to default.
         */
        DualDriveEqParams();

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
     * @ingroup AngleController_Essential_Structure_Variables
     */
    struct Inputs
    {
        /**
         * @brief Default constructor. Init variables value to default.
         */
        Inputs();

        double direct;                         ///! [-]. Direct value for direct mode of controller.      
        double angle;                          ///< [deg]. Angle value of plant.
        double rateMaster;                     ///< [deg/s]. Rate observer value of plant from master driver.
        double rateSlave;                      ///< [deg/s]. Rate observer value of plant from slave driver.
        double angleDes;                       ///< [deg]. Desired angle value for controller. 
        double rateDes;                        ///< [deg/s]. Desired rate value for controller.
    };

    /**
     * @struct Outputs
     * @brief Data structure for output values of any controllers.
     * @ingroup AngleController_Essential_Structure_Variables
     */
    struct Outputs
    {
        /**
         * @brief Default constructor. Init variables value to default.
         */
        Outputs();

        double primaryOutput;                ///< [-]. primaryOutput value output from AngleController.
        double secondaryOutput;              ///< [-]. secondaryOutput value output from AngleController. Depends on primaryOutput value.
        int8_t dir;                         ///< [-]. Direction signal value output from AngleController. {-1, 0, 1}
    };

}

// ##################################################################################
// General Functions:

/**
 * @namespace AngleControllerNamespace
 */
namespace AngleControllerNamespace
{
    /** 
     * @brief General upper limit function.
     * @ingroup AngleController_General_Functions
     * @param input: input data.
     * @param upLimit: up limitation value.
     * @return Limited value. 
    */ 
    double limitUp(const double &input, const double &upLimit);

    /** 
     * @brief General lower limit function.
     * @ingroup AngleController_General_Functions
     * @param input: input data.
     * @param downLimit: down limitation value.
     * @return Limited value. 
    */ 
    double limitDown(const double &input, const double &downLimit);

    /** 
     * @brief General limit function.
     * @ingroup AngleController_General_Functions
     * @param input: input data.
     * @param upLimit: up limitation value.
     * @param downLimit: down limitation value.
     * @return Limited value. 
    */ 
    double limit(const double &input, const double &upLimit, const double &downLimit);

    /** 
     * @brief General limit function.
     * @ingroup AngleController_General_Functions
     * @param input: input data.
     * @param limit: up/down limitation value. up_limit = +abs(limit), down_limit = -abs(limit).
     * @return Limited value. 
    */         
    double limit(const double &input, const double &limit);

}

// #######################################################################################
// LimitSlewRate class:
// This class is for change rate limit.

/**
 * @namespace AngleControllerNamespace
 */
namespace AngleControllerNamespace
{
    /**
     * @class LimitSlewRate
     * @ingroup AngleController_Classes
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
            double output;                           

            /**
             * @brief Set slew rate parameter value.
             * @param vlaue: must be positive.
             * @return true if successed.
             */
            bool setLimit(double value);

            /**
             * @brief Update output value of slew rate limit object.
             * @param input: new value.
             * @param dt: [sec]. Time step duration for update.
             * @return output of limitation.
             */
            double updateByTime(const double &input, const double &dt);

            /**
             * @brief Update output value of slew rate limit object.
             * @param input: new value.
             * @param frq: [Hz]. Frequency updatation.
             * @return output of limitation.
             */
            double updateByFrequency(const double &input, const double &frq);

            /**
             * @brief Clear any buffer on variables.
             */
            void clear(void); 
        
        private:

            /**
             * @brief Buffer for input of slewRate value that used in update method.
             */
            double _input;     

            /**
             * @brief Slew rate parameter value. *Hint: 0 value means disable it.
             */
            double SLEWRATE;        

            /**
             * @brief Calculate and update limited output by input signal and derivative of input signal.
             */
            void _calculate(const double &input, const double &derivative, const double &frq);
    };
}
// #######################################################################################
// LPF and LPFLimit class:

namespace AngleControllerNamespace
{
    /**
     * @class LPF
     * @ingroup AngleController_Classes
     * @brief Low pass filter class
     */
    class LPF
    {
        public:
            
            /**
             * @brief Parameters structure.
             */
            struct ParametersStructure
            {
                /// @brief Cut off frequency parameter of low pass filter. [Hz]. The value of 0 means it is disabled. 
                float FRQ;                   
            }parameters;

            /**
             * @brief Default constructor. Init some variables.
             */
            LPF();

            /**
             * @brief Buffer memory for output of lowpass filter.
             */
            double output;              

            /**
             * @brief Low pass filter updatation. Calculate output signal relative to input signal and time step.
             * Store value of filtered output signal.
             * @param input: Value of input signal.
             * @param dt: Time step duration. [sec].
             * @return output of filter.
             */
            virtual double updateByTime(const double &input, const double &dt);

            /**
             * @brief Low pass filter updatation. Calculate output signal relative to input signal and frequency of update step.
             * Store value of filtered output signal.
             * @param input: Value of input signal.
             * @param frq: Frequency updatation step. [Hz].
             * @return output of filter
             */
            virtual double updateByFrequency(const double &input, const double &frq);

            /**
             * @brief Set Cut off frequency of low pass filter.
             * @param frq: Cut off frequency. [Hz]
             * @return true if successed.
             *  */  
            bool setFrequency(const double &frq);

            /**
             * @brief Set output directely without filter.
             * Use this function for init value of output signal.
             */
            void setOutputDirect(const double &data);

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
            double _alpha;           

    };

    /**
     * @class LPFLimit
     * @ingroup AngleController_Classes
     * @brief Low pass filter class with input limitation
     */
    class LPFLimit: public LPF
    {
        public:

            struct LPFLimit_Parameters : public ParametersStructure
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
            virtual double updateByTime(const double &input, const double &dt) override;

            /**
             * @brief Low pass filter updatation. Calculate output signal relative to input signal and frequency of update step.
             * Store value of filtered output signal.
             * @param input: Value of input signal.
             * @param frq: Time step duration. [Hz].
             * @return output of filter.
             */
            virtual double updateByFrequency(const double &input, const double &frq) override;

            /** 
             * @brief Set input limitation values.
             * @param limit: up/down limitation value. up_limit = +abs(limit), down_limit = -abs(limit).
            */   
            void setInputLimit(double limit);

            /** 
             * @brief Set input limitation values.
             * @param upLimit: up limitation value.
             * @param downLimit: down limitation value.
            */ 
            void setInputLimit(double upLimit, double DownLimit);

        private:

            /**
             * @brief Limit input data.
             * @return Limited value.
             */
            double _limitInput(const double &input);
    };

}
// #######################################################################################
// Path planning class:

// namespace AngleControllerNamespace
// {

//     class PathPlanner
//     {
//         public:

//             struct ParametersStructure
//             {
//                 /**
//                  * @brief
//                  */
//                 float UP_LIMIT;

//                 /**
//                  * @brief
//                  */
//                 float DOWN_LIMIT;
//             }parameters;

//             struct ValueStructure
//             {
//                 /**
//                  * @brief
//                  */
//                 float outputAngleDes;
//             }value;

//             PathPlanner();

//             bool init(void);

//             float getOutputAngleDes(float inputAngleDes, float angle);

//         private:

//             bool _checkParameters(void);
//     };
// }

// #######################################################################################
// PID class:

namespace AngleControllerNamespace
{
    /**
     * @class PController
     * @ingroup AngleController_Classes
     * @brief General P controller class for any higher level controller.
     */
    class PController
    {
        public:

            /// @brief Last error for controller.
            std::string errorMessage;

            /// @brief Output signal value of controller. [output_unit]
            double output;          

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
            virtual double update(const double &desiredState, const double &state);

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
            double _e;       

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
     * @ingroup AngleController_Classes
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

                /**
                 * @brief The gain for exponential dynamic integral gain. [-]
                 * @note - The value of 0 means it is disabled.
                 * 
                 * @note - I_dynamically = I * exp(-IEXP * abs(error))
                 */
                float IEXP;

                /**
                 * @brief The active zone for integral calculations.
                 * @note - The value of 0 means it is disabled.
                 */
                float IZONE;
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
            virtual double updateByTime(const double &desiredState, const double &state, const double &dt);

            /**
             * Update and calculate output signal of controller using update frequency.
             * @param desiredState: Desired state for controller.
             * @param state: Feedback state from plant system.
             * @param frq: Frequency updatation. [Hz]
             * @return output value of controller.
             */
            virtual double updateByFrequency(const double &desiredState, const double &state, const double &frq);

        protected:

            double _ISum;        // buffer memory for output of integral summation controller.

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
     * @ingroup AngleController_Classes
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
            virtual double updateByTime(const double &desiredState, const double &state, const double &dt) override;

            /**
             * Update and calculate output signal of controller using update frequency.
             * @param desiredState: Desired state for controller.
             * @param state: Feedback state from plant system.
             * @param frq: Frequency updatation. [Hz]
             * @return output value of controller.
             */
            virtual double updateByFrequency(const double &desiredState, const double &state, const double &frq) override;

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
     * @ingroup AngleController_Classes
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
                 * @note The value of 0 means it is disabled.
                 */
                float PRIM_MAX;

                /**
                 * @brief PrimaryOutput range for set map scale.
                 * @note The value of 0 means it is disabled.
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
            AngleControllerNamespace::Outputs update(const double &input);

            /**
             * @brief Clear any buffer on variables.
             */
            void clear(void);

        private:

            /**
             * @brief Slope of line for primary input/output mapping.
             * @note The value is 1 if the PRIME_RANGE parameter is 0.
             */
            double _m_map;

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
 * @class AngleController_SingleDrive
 * @ingroup AngleController_Classes
 * @brief Controller class for single drive system.
 */
class AngleController_SingleDrive
{
    public:

        /// @brief parameter struct
        AngleControllerNamespace::SingleDriveParams parameters;  

        /// @brief Outputs struct
        AngleControllerNamespace::Outputs outputs;

        /// @brief Last error occurred for the object.
        std::string errorMessage;

        /**
         * @brief Default constructor. Init parameters by default values.
         */
        AngleController_SingleDrive();

        /**
         * @brief Set controller mode. 0:None, 1:Direct, 2:Angle, 3:Rate, 4:Tracking.
         * @param mode is the mode value. It can used from "AngleController_Mode" macros for set mode value.
         * @return true if succeeded.
         *  */ 
        bool setMode(const uint8_t &mode);

        /**
         * @brief Get the current control mode.
         * @return 0:None, 1:Direct, 2:Angle, 3:Rate, 4:Tracking.
         */
        uint8_t getMode(void);
        
        /**
         * @brief Return instance frequency of update controller variable. It can changed by rate of call the update function.
         *  */ 
        float getFrq(void);    

        /**
         * @brief Return controller error value. error_angle or error_rate depends on controller mode.
         * @note - In None and Direct modes returns 0.
         * 
         * @note - In Tracking and Angle modes returns error of angle.
         * 
         * @note - In Rate mode retrurns error of rate.
         *  */ 
        float getError(void);  

        /**
         * @brief Get internal Rate demanded. It is calculated just by controller.
         */
        float getRateDemanded(void);

        /**
         * @brief Get method for return parameters value:
         *  */ 
        void getParams(AngleControllerNamespace::SingleDriveParams *params);

        // ------------------------------------------
        // Virtual functions:

        /**
         * @brief Check params for allowable values. Calculate some variavles depends on other Parameters.
         * @return true if succeeded.
         *  */ 
        virtual bool init(void);  

        /**
         * @brief Update controller outputs.
         * @param T_now: Time at the update moment. [us]
         * @return true if successed.
         *  */ 
        virtual bool update(const uint64_t &T_now);

        /// @brief Clear any buffer date on variables.
        virtual void clear(void);     

        /// @brief Set inputs data for controller.
        virtual void setInputs(const AngleControllerNamespace::Inputs &data); 

        /**
         * @brief Set method for parameters value.
         * @return true if succeeded.
         *  */ 
        bool setParams(const AngleControllerNamespace::SingleDriveParams &data);

    protected:

        /// @brief Error angle of controller.
        double _eAngle;

        /// @brief Error rate of controller.
        double _eRate;

        /// @brief Internal Rate demanded. It is calculated just by controller.
        double _rateDemanded;

        /// @brief Measured controller Loop Frequency for update. [Hz]. It depends on executation of update function frequency. 
        double _frq;    

        /// @brief Target Time duration for control loop update. [us]. It depends on params.FRQ.                
        double _targetTime;              

        /// @brief Time Vector:{T_now, T_Last}.  [us]
        uint64_t _T[2];                

        /// @brief Time Deference :{T_now-T_last}. [us]
        uint64_t _dT;                  

        /// @brief Controller mode. 0:None, 1:Direct, 2:Angle, 3:Rate, , 4:Tracking.
        uint8_t _mode;

        /// @brief Inputs struct
        AngleControllerNamespace::Inputs _inputs;

        /// @brief Map object for controller output mapping.
        AngleControllerNamespace::MAP _map; 

        /// @brief LPF object for low pass filter of target rate signal.
        AngleControllerNamespace::LPFLimit _LPFT;

        /// @brief LPF pbject for low pass filter of output signal.
        AngleControllerNamespace::LPFLimit _LPFO;

        /// @brief PI controller object for angle control.
        AngleControllerNamespace::PIController _PIAngle;

        /// @brief PID controller object for rate control.
        AngleControllerNamespace::PIDController _PIDRate;

        /// @brief Slew rate object for angle acceleration limitation.
        AngleControllerNamespace::LimitSlewRate _limitSlewRate;

        /**
         * @brief Check parameters for allowable values.
         * @return true if succeeded.
         *  */ 
        bool _checkParameters(const AngleControllerNamespace::SingleDriveParams &data);

};

// #######################################################################################
// AngleController_DualDriveEq class:

/**
 * @class AngleController_DualDriveEq
 * @ingroup AngleController_Classes
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
         * @brief Get method for return parameters value.
         */
        void getParams(AngleControllerNamespace::DualDriveEqParams *data);

        /**
         * @brief Set method for parameters value.
         * @return true if successed.
         *  */                                                 
         bool setParams(const AngleControllerNamespace::DualDriveEqParams &data);

         bool refreshParams(void);

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

        // Set inputs data for controller.
        virtual void setInputs(const AngleControllerNamespace::Inputs &data) override; 

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

        /**
         * @brief Check parameters for allowable values.
         * @return true if successed.
         */
        bool _checkParameters(const AngleControllerNamespace::DualDriveEqParams &data);
};

#endif











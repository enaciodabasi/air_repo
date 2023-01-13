#ifndef AMR_HWI_UTILITY_HPP
#define AMR_HWI_UTILITY_HPP

#include <sensor_msgs/JointState.h>
#include <ros/time.h>

namespace amr
{
    namespace utils
    {
        /**
         * @brief Contains angular velocity and the angular position state of the joint.
         * 
         */
        struct State
        {
            double angularVel;  /**Angular velocity of the joint. [double]*/
            double angularPos;  /**Angular position of the joint. [double]*/
        };

        class WheelState
        {
            public:

            /**
             * @brief Construct a new Wheel State object
             * 
             * @param wheel_name: Name of the wheel.
             * @param encoder_resolution: Resolution of the encoder. 
             */
            WheelState(const std::string& wheel_name, const int encoder_resolution);

            /**
             * @brief Get the state of the wheel.
             * 
             * @param current_time current_time: Current time of the incoming encoder ticks, use NodeHandle::now().
             * @param encoder_ticks Ticks of the encoder retrieved from ADS.
             * @return State object containing angular velocity and angular position.
             */
            State getState(const ros::Time& current_time, const long encoder_ticks)
            {
                return calculateWheelState(current_time, encoder_ticks);
            }

            inline void setResolution(const int encoder_resolution)
            {
                this->m_EncoderResolution = encoder_resolution;
            }

            private:

            /**
             * @brief Name of the wheel joint.
             */
            std::string m_WheelName;

            /**
             * @brief Last time the state of the joint was updated.
             */
            ros::Time m_LastUpdateTime;
            
            /**
             * @brief State of the joint.
             */
            State m_State;

            long m_PrevEncoderTicks;

            int m_EncoderResolution;

            /*
                @brief  Calculates the angular position and the velocity of a wheel.
                @param  current_time: Current time of the incoming encoder ticks, use NodeHandle::now().
                @param  encoder_ticks: Ticks of the encoder retrieved from ADS.
                @return    A State struct variable containing both the current angular velocity and position of the wheel.
            */
            State calculateWheelState(const ros::Time& current_time, const long encoder_ticks);
            
            /**
             * @brief 
             * 
             * @param ticks: Ticks from the encoder.
             * @return double
             * @returns angle: Angle of the wheel joint. 
             */
            inline double ticksToAngle(const double& ticks)
            {
                double angle = ticks * (2.0 * M_PI / (double)m_EncoderResolution);
                return angle;
            }

        };
    

    }
}

#endif
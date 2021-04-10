package org.firstinspires.ftc.teamcode;

import java.util.Set;

class PID{
    //Controller gains
    float Kp;
    float Ki;
    float Kd;

    float SetPoint;
    float Measurement;
    double Output;

    //Derivatives low pass filter time constant
    float Tau;

    //Output limits
    float LimitMin;
    float LimitMax;

    //Sample time in seconds
    float SampleTime;

    //Controller memory
    double Integrator;
    double Differentiator;

    float PrevError;
    float PrevMeasurement;

    PID()
    {
        Kp = 2.0f;
        Ki = 0.5f;
        Kd = 0.25f;

        SampleTime =  0.01f;

        SetPoint = 0f;
        Measurement = 0f;
        Output = 0f;
        Integrator = 0f;
        Differentiator = 0f;
        PrevError = 0f;
        PrevMeasurement = 0f;
        Tau = 0.02f;

        LimitMin = -10.0f;
        LimitMax = 10.0f;
    }

    void Update()
    {
        //Set Error signal
        float error = SetPoint - Measurement;

        //Proportional
        float proportional = Kp * error;

        //Integral
        Integrator = Integrator + 0.5f * Ki * SampleTime * (error + PrevError);

        //Anti-wind-up via Dynamic integration clamping
        float limitMinInt = -5.0f;
        float limitMaxInt = 5.0f;

        if(LimitMax > proportional)
        {
            limitMaxInt = LimitMax - proportional;
        }
        else
        {
            limitMaxInt = 0f;
        }

        if(LimitMin < proportional)
        {
            limitMinInt = LimitMin - proportional;
        }
        else
        {
            limitMinInt = 0f;
        }

        //Clamp integrator
        if(Integrator > limitMaxInt)
        {
            Integrator = limitMaxInt;
        }
        else if(Integrator < limitMinInt)
        {
            Integrator = limitMinInt;
        }

        //Derivative
        Differentiator = (2.0f * Kd * (Measurement - PrevMeasurement) +
                (2.0f * Tau - SampleTime) * Differentiator) /
                (2.0f * Tau + SampleTime);

        //Compute Integration limits
        Output = proportional + Integrator + Differentiator;

        if(Output > LimitMax)
        {
            Output = LimitMax;
        }
        else if(Output < LimitMin)
        {
            Output = LimitMin;
        }

        //Store error and measurement as prev
        PrevMeasurement = Measurement;
        PrevError = error;
    }

};

public class PIDController {

    //PID Controllers
    PID XPid;
    PID YPid;


    void Initialize()
    {
        XPid = new PID();
        YPid = new PID();
    }

    void Update()
    {
        XPid.Update();
        YPid.Update();
    }

}



 /*
------------------------------------------------------------------------------
~ File   : pid.c
~ Author : Majid Derhambakhsh
~ Version: V1.0.0
~ Created: 02/11/2021 03:43:00 AM
~ Brief  :
~ Support:
		   E-Mail : Majid.do16@gmail.com (subject : Embedded Library Support)

		   Github : https://github.com/Majid-Derhambakhsh
------------------------------------------------------------------------------
~ Description:

~ Attention  :

~ Changes    :
------------------------------------------------------------------------------
*/

#include "pid.h"
#include "motor.h"
#include <math.h>   // fabs
#include <stdbool.h>


/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~ Initialize ~~~~~~~~~~~~~~~~ */
void PID_Init(PID_TypeDef *uPID)
{
	/* ~~~~~~~~~~ Set parameter ~~~~~~~~~~ */
	uPID->OutputSum = *uPID->MyOutput;
	uPID->LastInput = *uPID->MyInput;
	
	if (uPID->OutputSum > uPID->OutMax)
	{
		uPID->OutputSum = uPID->OutMax;
	}
	else if (uPID->OutputSum < uPID->OutMin)
	{
		uPID->OutputSum = uPID->OutMin;
	}
	else { }
	
}

void PID(PID_TypeDef *uPID, double *Input, double *Output, double *Setpoint, double Kp, double Ki, double Kd, PIDPON_TypeDef POn, PIDCD_TypeDef ControllerDirection)
{
	/* ~~~~~~~~~~ Set parameter ~~~~~~~~~~ */
	uPID->MyOutput   = Output;
	uPID->MyInput    = Input;
	uPID->MySetpoint = Setpoint;
	uPID->InAuto     = (PIDMode_TypeDef)_FALSE;
	
	PID_SetOutputLimits(uPID, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX); //change to follow the PWM max value
	
	uPID->SampleTime = _PID_SAMPLE_TIME_MS_DEF; /* default Controller Sample Time is 0.1 seconds */
	
	PID_SetControllerDirection(uPID, ControllerDirection);
	PID_SetTunings2(uPID, Kp, Ki, Kd, POn);
	
	uPID->LastTime = GetTime() - uPID->SampleTime;
	
}

void PID2(PID_TypeDef *uPID, double *Input, double *Output, double *Setpoint, double Kp, double Ki, double Kd, PIDCD_TypeDef ControllerDirection)
{
	PID(uPID, Input, Output, Setpoint, Kp, Ki, Kd, _PID_P_ON_E, ControllerDirection);
}

/* ~~~~~~~~~~~~~~~~~ Computing ~~~~~~~~~~~~~~~~~ */
uint8_t PID_Compute(PID_TypeDef *uPID)
{
    uint32_t now;
    uint32_t timeChange;

    double input;
    double error;
    double dInput;
    double output;

    if (!uPID->InAuto) {
        return _FALSE;
    }

    now        = GetTime();
    timeChange = (now - uPID->LastTime);

    if (timeChange >= uPID->SampleTime)
    {
        input  = *uPID->MyInput;
        error  = *uPID->MySetpoint - input;
        dInput = (input - uPID->LastInput);

        // Optional: reset integral when command ~ 0 (velocity controller)
        if (fabs(*uPID->MySetpoint) < 1e-6) {
            uPID->OutputSum = 0;
        }

        // Candidate integral update
        double newISum = uPID->OutputSum + (uPID->Ki * error);

        // If P_ON_M, integral bucket also includes -Kp*dInput (same as original lib)
        if (!uPID->POnE) {
            newISum -= uPID->Kp * dInput;
        }

        // P and D (same sign as original lib)
        double P = uPID->POnE ? (uPID->Kp * error) : 0.0;
        double D = -uPID->Kd * dInput;

        // Unsaturated output using candidate integral
        double u_unsat = P + newISum + D;

        // Conditional integration anti-windup
        const double EPS = 1e-9;
        bool sat_high = (u_unsat > uPID->OutMax - EPS);
        bool sat_low  = (u_unsat < uPID->OutMin + EPS);

        if ((!sat_high && !sat_low) ||
            (sat_high && error < 0) ||
            (sat_low  && error > 0))
        {
            uPID->OutputSum = newISum;
        }

        // Clamp integrator bucket
        if (uPID->OutputSum > uPID->OutMax) uPID->OutputSum = uPID->OutMax;
        else if (uPID->OutputSum < uPID->OutMin) uPID->OutputSum = uPID->OutMin;

        // Compute final output (like original library)
        output = uPID->POnE ? (uPID->Kp * error) : 0.0;
        output += uPID->OutputSum - uPID->Kd * dInput;

        // Clamp output
        if (output > uPID->OutMax) output = uPID->OutMax;
        else if (output < uPID->OutMin) output = uPID->OutMin;

        *uPID->MyOutput = output;

        uPID->LastInput = input;
        uPID->LastTime  = now;

        return _TRUE;
    }

    return _FALSE;
}

/* ~~~~~~~~~~~~~~~~~ PID Mode ~~~~~~~~~~~~~~~~~~ */
void            PID_SetMode(PID_TypeDef *uPID, PIDMode_TypeDef Mode)
{
	
	uint8_t newAuto = (Mode == _PID_MODE_AUTOMATIC);
	
	/* ~~~~~~~~~~ Initialize the PID ~~~~~~~~~~ */
	if (newAuto && !uPID->InAuto)
	{
		PID_Init(uPID);
	}
	
	uPID->InAuto = (PIDMode_TypeDef)newAuto;
	
}
PIDMode_TypeDef PID_GetMode(PID_TypeDef *uPID)
{
	return uPID->InAuto ? _PID_MODE_AUTOMATIC : _PID_MODE_MANUAL;
}

/* ~~~~~~~~~~~~~~~~ PID Limits ~~~~~~~~~~~~~~~~~ */
void PID_SetOutputLimits(PID_TypeDef *uPID, double Min, double Max)
{
	/* ~~~~~~~~~~ Check value ~~~~~~~~~~ */
	if (Min >= Max)
	{
		return;
	}
	
	uPID->OutMin = Min;
	uPID->OutMax = Max;
	
	/* ~~~~~~~~~~ Check PID Mode ~~~~~~~~~~ */
	if (uPID->InAuto)
	{
		
		/* ..... Check out value ..... */
		if (*uPID->MyOutput > uPID->OutMax)
		{
			*uPID->MyOutput = uPID->OutMax;
		}
		else if (*uPID->MyOutput < uPID->OutMin)
		{
			*uPID->MyOutput = uPID->OutMin;
		}
		else { }
		
		/* ..... Check out value ..... */
		if (uPID->OutputSum > uPID->OutMax)
		{
			uPID->OutputSum = uPID->OutMax;
		}
		else if (uPID->OutputSum < uPID->OutMin)
		{
			uPID->OutputSum = uPID->OutMin;
		}
		else { }
		
	}
	
}

/* ~~~~~~~~~~~~~~~~ PID Tunings ~~~~~~~~~~~~~~~~ */
void PID_SetTunings(PID_TypeDef *uPID, double Kp, double Ki, double Kd)
{
	PID_SetTunings2(uPID, Kp, Ki, Kd, uPID->POn);
}
void PID_SetTunings2(PID_TypeDef *uPID, double Kp, double Ki, double Kd, PIDPON_TypeDef POn)
{
	
	double SampleTimeInSec;
	
	/* ~~~~~~~~~~ Check value ~~~~~~~~~~ */
	if (Kp < 0 || Ki < 0 || Kd < 0)
	{
		return;
	}
	
	/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
	uPID->POn    = POn;
	uPID->POnE   = (PIDPON_TypeDef)(POn == _PID_P_ON_E);

	uPID->DispKp = Kp;
	uPID->DispKi = Ki;
	uPID->DispKd = Kd;
	
	/* ~~~~~~~~~ Calculate time ~~~~~~~~ */
	SampleTimeInSec = ((double)uPID->SampleTime) / 1000;
	
	uPID->Kp = Kp;
	uPID->Ki = Ki * SampleTimeInSec;
	uPID->Kd = Kd / SampleTimeInSec;
	
	/* ~~~~~~~~ Check direction ~~~~~~~~ */
	if (uPID->ControllerDirection == _PID_CD_REVERSE)
	{
		
		uPID->Kp = (0 - uPID->Kp);
		uPID->Ki = (0 - uPID->Ki);
		uPID->Kd = (0 - uPID->Kd);
		
	}
	
}

/* ~~~~~~~~~~~~~~~ PID Direction ~~~~~~~~~~~~~~~ */
void          PID_SetControllerDirection(PID_TypeDef *uPID, PIDCD_TypeDef Direction)
{
	/* ~~~~~~~~~~ Check parameters ~~~~~~~~~~ */
	if ((uPID->InAuto) && (Direction !=uPID->ControllerDirection))
	{
		
		uPID->Kp = (0 - uPID->Kp);
		uPID->Ki = (0 - uPID->Ki);
		uPID->Kd = (0 - uPID->Kd);
		
	}
	
	uPID->ControllerDirection = Direction;
	
}
PIDCD_TypeDef PID_GetDirection(PID_TypeDef *uPID)
{
	return uPID->ControllerDirection;
}

/* ~~~~~~~~~~~~~~~ PID Sampling ~~~~~~~~~~~~~~~~ */
void PID_SetSampleTime(PID_TypeDef *uPID, int32_t NewSampleTime)
{
	
	double ratio;
	
	/* ~~~~~~~~~~ Check value ~~~~~~~~~~ */
	if (NewSampleTime > 0)
	{
		
		ratio = (double)NewSampleTime / (double)uPID->SampleTime;
		
		uPID->Ki *= ratio;
		uPID->Kd /= ratio;
		uPID->SampleTime = (uint32_t)NewSampleTime;
		
	}
	
}

/* ~~~~~~~~~~~~~ Get Tunings Param ~~~~~~~~~~~~~ */
double PID_GetKp(PID_TypeDef *uPID)
{
	return uPID->DispKp;
}
double PID_GetKi(PID_TypeDef *uPID)
{
	return uPID->DispKi;
}
double PID_GetKd(PID_TypeDef *uPID)
{
	return uPID->DispKd;
}

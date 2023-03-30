package org.firstinspires.ftc.teamcode;

public class Calculate {
    /** Variables */
    public double Error=0, Integral=0, Derivative=0, LastError=0, LastTime=0;

    public boolean Plus_Minus(double input, int check, double range) {
        return check - range < input && input < check + range;
    }

    public double AngleWrap(double radians) {
        if (radians > Math.PI)  radians -= 2 * Math.PI;
        if (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    public double PIDControl(double[] K_PID, double setpoint, double yaw){
        double CurrentTime  = (double)(System.nanoTime() * 1E-9);
        double Dt           = CurrentTime - LastTime;
        Error       = AngleWrap(setpoint - yaw);
        Integral    = Integral + (Error * Dt);
        Derivative  = (Error - LastError) / Dt;
        LastError   = Error;
        LastTime    = CurrentTime;
        return (Error * K_PID[0]) + (Integral * K_PID[1]) + (Derivative * K_PID[2]);
    }
}

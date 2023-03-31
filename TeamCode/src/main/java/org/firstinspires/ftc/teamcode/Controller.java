package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

public class Controller {
    private double Kp, Ki, Kd;
    private double Error, LastError;
    public double Integral;
    public double Derivative;
    private double Dt, LastTime;

    public Controller(double kP, double kI, double kD) {
        Kp         = kP;
        Ki         = kI;
        Kd         = kD;
        Dt = Error = Integral = Derivative = LastTime = LastError = 0;
    }

    public void setPID(double kP, double kI, double kD) {
        Kp = kP;
        Ki = kI;
        Kd = kD;
    }

    public double Calculate(double error) {
        double CurrentTime  = (double)(System.nanoTime() * 1E-9);
        if (LastTime == 0) LastTime = CurrentTime;
        Dt          = CurrentTime - LastTime;
        LastTime    = CurrentTime;
        Error       = error;  // Error = Setpoint - Current
        Integral    = Integral + (Error * Dt);
        Integral    = Range.clip(Integral, -1, 1);
        Derivative  = Math.abs(Dt) > 1E-6 ? (Error - LastError) / Dt : 0;
        LastError   = Error;
        return (Error * Kp) + (Integral * Ki) + (Derivative * Kd);
    }

    public boolean atSetpoint() {
        return Math.abs(Error) < 0.05;
    }

    public double getError() { return Error; }
}

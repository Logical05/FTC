package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

public class Controller {
    private double Kp, Ki, Kd, Kf;
    private double Setpoint;
    private double Error, LastError, ErrorTolerance;
    private double Integral;
    private double Derivative;
    private double Dt, LastTime;

    public Controller(double kp, double ki, double kd, double kf) {
        Kp         = kp;
        Ki         = ki;
        Kd         = kd;
        Kf         = kf;
        Setpoint = Dt = Error = Integral = Derivative = LastTime = LastError = 0;
        ErrorTolerance = 0.05;
    }

    public void setErrorTolerance(double tolerance) { ErrorTolerance = tolerance; }

    public void setPIDF(double kp, double ki, double kd, double kf) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        Kf = kf;
    }

    public double Calculate(double setpoint, double current) {
        Setpoint = setpoint;
        return Calculate(setpoint - current);
    }

    public double Calculate(double error) {
        double CurrentTime = System.nanoTime() * 1E-9;
        if (LastTime == 0) LastTime = CurrentTime;
        Dt          = CurrentTime - LastTime;
        LastTime    = CurrentTime;
        Error       = error;  // Error = Setpoint - Current
        Integral    = Integral + (Error * Dt);
        Integral    = Range.clip(Integral, -1, 1);
        Derivative  = Math.abs(Dt) > 1E-6 ? (Error - LastError) / Dt : 0;
        LastError   = Error;
        return (Error * Kp) + (Integral * Ki) + (Derivative * Kd) + (Setpoint * Kf);
    }

    public boolean atSetpoint() { return Math.abs(Error) < ErrorTolerance; }

    public double getError() { return Error; }

    public double getDt() { return Dt; }
}

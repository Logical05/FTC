package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

public class Controller {
    public double Kp, Ki, Kd, Kf, Setpoint, Error, LastError, ErrorTolerance;
    public double Integral, Derivative, Dt, LastTime;

    public Controller(double Kp, double Ki, double Kd, double Kf) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        Setpoint = Dt = Error = Integral = Derivative = LastTime = LastError = 0;
        ErrorTolerance = 0.05;
    }

    public void setPIDF(double Kp, double Ki, double Kd, double Kf) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
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
}

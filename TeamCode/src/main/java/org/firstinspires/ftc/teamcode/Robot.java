package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Robot {
    /** Hardware */
    public IMU IMU;
    public Servo LA, RA, K;
    public DcMotor FL, FR, BL, BR, B, LL, RL;

    /** Variables */
    public final int Max_Lift = 1000;
    public ElapsedTime PID_timer = new ElapsedTime();
    public double error, lasterror=0, integral=0;

    public void MovePower(double Front_Left, double Front_Right,
                          double Back_Left,  double Back_Right) {
        FL.setPower(Front_Left);
        FR.setPower(Front_Right);
        BL.setPower(Back_Left);
        BR.setPower(Back_Right);
    }

    public void Initialize(IMU imu, DcMotor.RunMode MoveMode,
                           DcMotor Front_Left, DcMotor Front_Right,
                           DcMotor Back_Left,  DcMotor Back_Right, DcMotor Base,
                           DcMotor Left_Lift,  DcMotor Right_Lift,
                           double Arm_pos, Servo Left_Arm, Servo Right_Arm,
                           double Keeper_pos, Servo Keeper) {
        // Add Variable
        IMU = imu;
        FL = Front_Left;
        FR = Front_Right;
        BL = Back_Left;
        BR = Back_Right;
        B  = Base;
        LL = Left_Lift;
        RL = Right_Lift;
        LA = Left_Arm;
        RA = Right_Arm;
        K  = Keeper;

        // Initialize IMU
        IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                                                                       RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        // Reverse Motors
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        LL.setDirection(DcMotorSimple.Direction.REVERSE);
        // setMode Motors
        FL.setMode(MoveMode);
        FR.setMode(MoveMode);
        BL.setMode(MoveMode);
        BR.setMode(MoveMode);
        B .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        B .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // SetBehavior Motors
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        B .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // SetPower
        MovePower(0, 0, 0, 0);
        B .setPower(0);
        LL.setPower(0);
        RL.setPower(0);

        // Reverse Servo
        RA.setDirection(Servo.Direction.REVERSE);
        // Set Servo Position
        LA.setPosition(Arm_pos);
        RA.setPosition(Arm_pos);
        K .setPosition(Keeper_pos);
    }
    public boolean Plus_Minus(double input, int check, double range) {
        return check - range < input && input < check + range;
    }

    public double AngleWrap(double radians) {
        if (radians > Math.PI)  radians -= 2 * Math.PI;
        if (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    public void Turn_Base(int angle) {
        int cpr = 1440;              // Encoder counts per revolution
        int c = (angle * cpr)/ 360;  // Encoder counts
        B.setTargetPosition(c);
        B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        B.setPower(0.5);
    }

    public double PIDControl(double setpoint, double yaw, double[] K_PID){
        double dT = PID_timer.seconds();
        error = AngleWrap(setpoint - yaw);
        integral = Plus_Minus(Math.toDegrees(error), 0, 0.45) ? 0 : integral + (error * dT);
        double derivative = (error - lasterror) / dT;
        lasterror = error;
        double output = (error * K_PID[0]) + (integral * K_PID[1]) + (derivative * K_PID[2]);
        if (0 < output && output < 0.08) output = 0.08;
        if (-0.08 < output && output < 0) output = 0.08;
        return output;
    }
}

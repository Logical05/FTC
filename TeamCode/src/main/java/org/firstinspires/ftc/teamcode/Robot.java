package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Robot {
    /** Hardware */
    public IMU imu;
    public Servo LA, RA, K;
    public DcMotor FL, FR, BL, BR, B;

    /** Variables */
    public ElapsedTime PID_timer = new ElapsedTime();
    public double yaw, error, lasterror=0, integral=0;

    public void MovePower(double Front_Left, double Front_Right,
                          double Back_Left,  double Back_Right) {
        FL.setPower(Front_Left);
        FR.setPower(Front_Right);
        BL.setPower(Back_Left);
        BR.setPower(Back_Right);
    }

    public void Initialize(IMU IMU, DcMotor.RunMode mode,
                           DcMotor Front_Left, DcMotor Front_Right,
                           DcMotor Back_Left,  DcMotor Back_Right, DcMotor Base,
                           double Arm_pos, Servo Left_Arm, Servo Right_Arm,
                           double Keeper_pos, Servo Keeper) {
        // Add Variable
        imu = IMU;
        FL = Front_Left;
        FR = Front_Right;
        BL = Back_Left;
        BR = Back_Right;
        B  = Base;
        LA = Left_Arm;
        RA = Right_Arm;
        K  = Keeper;

        // Initialize IMU
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                                                                       RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        // Reverse Motors
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        // setMode Motors
        FL.setMode(mode);
        FR.setMode(mode);
        BL.setMode(mode);
        BR.setMode(mode);
//        B .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        B .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // SetBehavior Motors
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        B .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // SetPower
        MovePower(0, 0, 0, 0);
//        B.setPower(0);

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

    public double PIDControl(double setpoint){
        double[] K_PID = {0.8, 0.3, 0.2};
        double dT = PID_timer.seconds();
        yaw = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        error = AngleWrap(setpoint - yaw);
        integral = Plus_Minus(Math.toDegrees(error), 0, 0.45) ? 0 : integral + (error * dT);
        double derivative = (error - lasterror) / dT;
        lasterror = error;
        double output = (error * K_PID[0]) + (integral * K_PID[1]) + (derivative * K_PID[2]);
        if (0 < output && output < 0.08) output = 0.08;
        if (-0.08 < output && output < 0) output = -0.08;
        return output;
    }
}

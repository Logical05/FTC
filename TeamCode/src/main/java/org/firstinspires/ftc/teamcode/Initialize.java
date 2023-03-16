package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class Initialize {
    public DcMotor FL, FR, BL, BR;
    public Servo LA, RA;
    public IMU imu;

    public void Init(double Arm_pos, IMU IMU, DcMotor Front_Left, DcMotor Front_Right, DcMotor Back_Left, DcMotor Back_Right, Servo Right_Arm, Servo Left_Arm) {
        // Add Variable
        imu = IMU;
        FL = Front_Left;
        FR = Front_Right;
        BL = Back_Left;
        BR = Back_Right;
        RA = Right_Arm;
        LA = Left_Arm;

        // Set IMU
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        // Reverse Motors
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        // setMode Motors
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // SetBehavior Motors
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse Servo
        RA.setDirection(Servo.Direction.REVERSE);
        // Set Servo
        LA.setPosition(Arm_pos);
        RA.setPosition(Arm_pos);

    }
}

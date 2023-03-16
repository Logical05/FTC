package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class Initialize extends LinearOpMode {
    public DcMotor FL, FR, BL, BR;
    public Servo LA, RA;
    public IMU imu;
    public void Init(double Arm_pos){
        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        // Initialize Motors
        FL = hardwareMap.get(DcMotor.class, "Front_Left");
        FR = hardwareMap.get(DcMotor.class, "Front_Right");
        BL = hardwareMap.get(DcMotor.class, "Back_Left");
        BR = hardwareMap.get(DcMotor.class, "Back_Right");
        // Reverse Motors
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
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

        // Initialize Servos
        LA = hardwareMap.get(Servo.class, "Left_Arm");
        RA = hardwareMap.get(Servo.class, "Right_Arm");
        // Reverse Servo
        RA.setDirection(Servo.Direction.REVERSE);
        // Set Servo
        LA.setPosition(Arm_pos);
        RA.setPosition(Arm_pos);

        // Ready to Go!
        telemetry.addLine(": Robot Ready to Go!  Press Play.");
        telemetry.update();
    }

    @Override
    public void runOpMode() {}
}

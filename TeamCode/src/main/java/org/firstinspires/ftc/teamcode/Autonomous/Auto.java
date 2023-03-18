package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Autonomous")
public class Auto extends LinearOpMode {
    /** Usage External Class */
    Robot robot = new Robot();

    /** Hardware */
    IMU imu;
    Servo LA, RA;
    DcMotor FL, FR, BL, BR;

    /** Variables */

    private void Init(){
        // HardwareMap
        imu = hardwareMap.get(IMU.class, "imu");
        LA = hardwareMap.get(Servo.class, "Left_Arm");
        RA = hardwareMap.get(Servo.class, "Right_Arm");
        FL = hardwareMap.get(DcMotor.class, "Front_Left");
        FR = hardwareMap.get(DcMotor.class, "Front_Right");
        BL = hardwareMap.get(DcMotor.class, "Back_Left");
        BR = hardwareMap.get(DcMotor.class, "Back_Right");

        // Initialize Robot
        robot.Initialize(imu, DcMotor.RunMode.RUN_WITHOUT_ENCODER, FL, FR, BL, BR, 0.35, LA, RA);
    }

    @Override
    public void runOpMode() {
        Init();
        waitForStart();
        if (opModeIsActive()) {

        }
    }
}
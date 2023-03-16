package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "servotest")
public class servotest extends LinearOpMode {
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;

    private Servo LA = null;
    private Servo RA = null;
    private double up;
    private IMU imu;
    private void Init() {
        //detect hardware
        FL = hardwareMap.get(DcMotor.class, "Front_Left");
        FR = hardwareMap.get(DcMotor.class, "Front_Right");
        BL = hardwareMap.get(DcMotor.class, "Back_Left");
        BR = hardwareMap.get(DcMotor.class, "Back_Right");
        LA = hardwareMap.get(Servo.class, "Left_Arm");
        RA = hardwareMap.get(Servo.class, "Right_Arm");
        imu = hardwareMap.get(IMU.class, "imu");
        //reverse motor and servo
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        RA.setDirection(Servo.Direction.REVERSE);
        //set stop and reset encoder
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //set motor run with out encoder
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Break after power = 0
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //set IMU
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(new Orientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, 0, 0, 0, 0))));
    }


    @Override
    public void runOpMode() {
        Init();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                Servo();
                telemetry.update();
            }
        }
    }
    private void Servo(){
        up = gamepad1.dpad_left ? up-0.005 : (gamepad1.dpad_right ? up+0.005 : up);
        up = Range.clip(up, 0, 0.35);

        RA.setPosition(up);
        LA.setPosition(up);

        telemetry.addData("up", up);
        sleep(20);
    }
    private boolean Plus_Minus(float input, int check, double range) {
        return check - range < input && input < check + range;
    }
    private double anglewrap(double w_angles) {
        while (w_angles >= 180)  w_angles -= 360;
        while (w_angles <= -180) w_angles += 360;
        return w_angles;
    }
}
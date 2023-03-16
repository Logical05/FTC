package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "SpeedTest")
public class SpeedTest extends LinearOpMode {
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private IMU imu;
    double speedX = 0.5;
    double speedY = 0.5;
    double speedRX = 0.5;
    double lastspeedX = 0;
    double lastspeedY = 0;
    double lastspeedRX = 0;
    double plusminusspeed = 0.01;
    double lastplusminusspeed = 0;
    private void Init() {
        //detect hardware
        FL = hardwareMap.get(DcMotor.class, "Front_Left");
        FR = hardwareMap.get(DcMotor.class, "Front_Right");
        BL = hardwareMap.get(DcMotor.class, "Back_Left");
        BR = hardwareMap.get(DcMotor.class, "Back_Right");
        imu = hardwareMap.get(IMU.class, "imu");
        //reverse motor
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
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
                Move();
                Motorspeed();
                IMUcheck();
                telemetry.update();
            }
        }
    }
    private void Move() {
        double x = Range.clip(gamepad1.left_stick_x, -speedX, speedX);
        double y = Range.clip(-gamepad1.left_stick_y, -speedY, speedY);
        double rx = Range.clip(gamepad1.right_stick_x, -speedRX, speedRX);
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        FL.setPower((y + x + rx)/ denominator);
        FR.setPower((y - x - rx)/ denominator);
        BL.setPower((y - x + rx)/ denominator);
        BR.setPower((y + x - rx)/ denominator);
    }
    private void Motorspeed(){
        plusminusspeed = gamepad1.a ? plusminusspeed-0.01 : (gamepad1.y ? plusminusspeed+0.01 : plusminusspeed);
        speedX = gamepad1.dpad_left ? speedX-plusminusspeed : (gamepad1.dpad_right ? speedX+plusminusspeed : speedX);
        speedY = gamepad1.x ? speedY-plusminusspeed : (gamepad1.b ? speedY+plusminusspeed : speedY);
        speedRX = gamepad1.left_bumper ? speedRX-plusminusspeed : (gamepad1.right_bumper ? speedRX+plusminusspeed : speedRX);

        plusminusspeed = Range.clip(plusminusspeed, -1, 1);
        speedX = Range.clip(speedX, -1, 1);
        speedY = Range.clip(speedY, -1, 1);
        speedRX = Range.clip(speedRX, -1, 1);

        if(speedX != lastspeedX || speedY != lastspeedY || speedRX != lastspeedRX || plusminusspeed != lastplusminusspeed) {
            sleep(100);
        }

        lastplusminusspeed = plusminusspeed;
        lastspeedX = speedX;
        lastspeedY = speedY;
        lastspeedRX = speedRX;

        telemetry.addData("speedX", speedX);
        telemetry.addData("speedY", speedY);
        telemetry.addData("speedRX", speedRX);
        telemetry.addData("plusminusspeed", plusminusspeed);
//        telemetry.update();
    }
    private void IMUcheck(){
        telemetry.addData("yaw", -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("pitch", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll", imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
//        telemetry.update();
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
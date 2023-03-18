package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="TeleOp")
public class Tele extends LinearOpMode {
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

    private void Movement(){
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double PID = robot.PIDControl(0);
        // Rotate
        double r = Math.abs(gamepad1.right_stick_x) > 0 ? gamepad1.right_stick_x : (robot.Plus_Minus(Math.toDegrees(robot.error), 0, 0.45) ? 0 : PID);
        // Denominator for division to get no more than 1
        double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);

        robot.FL.setPower((y + x + r)/ d);
        robot.FR.setPower((y - x - r)/ d);
        robot.BL.setPower((y - x + r)/ d);
        robot.BR.setPower((y + x - r)/ d);

        telemetry.addData("yaw", Math.toDegrees(robot.yaw));
        telemetry.addData("Encoder", robot.FL.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        Init();
        waitForStart();
        if (opModeIsActive()) {
            robot.PID_timer.reset();
            while (opModeIsActive()) {
                while(gamepad1.touchpad) imu.resetYaw();
                Movement();
            }
        }
    }
}

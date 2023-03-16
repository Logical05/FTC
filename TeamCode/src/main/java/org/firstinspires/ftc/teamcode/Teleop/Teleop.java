package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="TeleOp")
public class Teleop extends LinearOpMode {
    DcMotor FL, FR, BL, BR;
    Servo LA, RA;
    IMU imu;

    // Variables
    ElapsedTime timer = new ElapsedTime();
    double angle, setpoint, output, error, lasterror = 0, intergralsum = 0;

    private void Init(){
        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        // Initialize Motors
        FL = hardwareMap.get(DcMotor.class, "Front_Left");
        FR = hardwareMap.get(DcMotor.class, "Front_Right");
        BL = hardwareMap.get(DcMotor.class, "Back_Left");
        BR = hardwareMap.get(DcMotor.class, "Back_Right");
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

        // Initialize Servos
        LA = hardwareMap.get(Servo.class, "Left_Arm");
        RA = hardwareMap.get(Servo.class, "Right_Arm");
        // Reverse Servo
        RA.setDirection(Servo.Direction.REVERSE);
        //Set Servo
        RA.setPosition(0.35);

        // Ready to Go!
        telemetry.addLine("Robot Ready to Go!  Press Play.");
        telemetry.update();
    }

    private boolean Plus_Minus(double input, int check, double range) {
        return check - range < input && input < check + range;
    }

    private double AngleWrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    private double PIDControl(double setpoint, double state){
        double[] K_PID = {0.8, 0.3, 0.2};
        error = AngleWrap(setpoint - state);
        intergralsum = Plus_Minus(Math.toDegrees(error), 0, 0.3) ? 0 : intergralsum + (error * timer.seconds());
        double derivative = (error - lasterror) / timer.seconds();
        lasterror = error;

        timer.reset();

        output = (error * K_PID[0]) + (intergralsum * K_PID[1]) + (derivative * K_PID[2]);
        while (0 < output && output < 0.05) output = 0.05;
        while (-0.05 < output && output < 0) output = -0.05;
        return output;
    }

    private void Movement(){
        angle = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double PID = PIDControl(Math.toRadians(setpoint), angle);
        // Rotate
        double r = Math.abs(gamepad1.right_stick_x) > 0 ? gamepad1.right_stick_x : (Plus_Minus(Math.toDegrees(error), 0, 0.3) ? 0 : PID);
        // Denominator for division to get no more than 1
        double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);

        FL.setPower((y + x + r)/ d);
        FR.setPower((y - x - r)/ d);
        BL.setPower((y - x + r)/ d);
        BR.setPower((y + x - r)/ d);
    }

    @Override
    public void runOpMode() {
        Init();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                Movement();
                telemetry.addData("yaw", Math.toDegrees(angle));

                telemetry.update();
            }
        }
    }
}

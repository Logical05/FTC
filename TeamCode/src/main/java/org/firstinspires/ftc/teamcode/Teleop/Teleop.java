package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Initialize;

@TeleOp(name="TeleOp")
public class Teleop extends LinearOpMode {
    // Hardware
    public DcMotor FL, FR, BL, BR;
    public Servo LA, RA;
    public IMU imu;
    // Import Initialize
    Initialize init = new Initialize();
    // Variables
    ElapsedTime timer = new ElapsedTime();
    double angle, setpoint, output, error, lasterror = 0, intergralsum = 0;

    private void Init(){
        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // Initialize Motors
        FL = hardwareMap.get(DcMotor.class, "Front_Left");
        FR = hardwareMap.get(DcMotor.class, "Front_Right");
        BL = hardwareMap.get(DcMotor.class, "Back_Left");
        BR = hardwareMap.get(DcMotor.class, "Back_Right");

        // Initialize Servos
        LA = hardwareMap.get(Servo.class, "Left_Arm");
        RA = hardwareMap.get(Servo.class, "Right_Arm");

        init.Init(0.35, imu, FL, FR, BL, BR, RA, LA);
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
        angle = -init.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double PID = PIDControl(Math.toRadians(setpoint), angle);
        // Rotate
        double r = Math.abs(gamepad1.right_stick_x) > 0 ? gamepad1.right_stick_x : (Plus_Minus(Math.toDegrees(error), 0, 0.3) ? 0 : PID);
        // Denominator for division to get no more than 1
        double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);

        init.FL.setPower((y + x + r)/ d);
        init.FR.setPower((y - x - r)/ d);
        init.BL.setPower((y - x + r)/ d);
        init.BR.setPower((y + x - r)/ d);
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

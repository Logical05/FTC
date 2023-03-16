package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "AutoTest")
public class AutoTest extends LinearOpMode {

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private IMU imu;
    double angle;
    //PID
    double lasterror = 0;
    double intergralsum = 0;
    double Kp = 0.03;
    double Ki = 0.003;
    double Kd = 0.003;
    double Kf = 3;

    ElapsedTime timer = new ElapsedTime();
    private void Init() {
        //detect hardware
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        imu = hardwareMap.get(IMU.class, "imu");
        //reverse motor
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        //set stop and reset encoder
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //set run with encoder
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Break after power = 0
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //set imu
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(new Orientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, 0, 0, 0, 0))));
        imu.resetYaw();
    }
    @Override
    public void runOpMode() {
        Init();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            Move(0, 0.8, 0, 1500, 1);
            Move(-0.8, 0, 90, 900, 1);
            Move(0.8, 0, 90, 900, 1);
            stop();
        }
    }
    private void Move(double x, double y, double setpoint, int degree, double Break) {
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while ((Math.abs(FL.getCurrentPosition()) + Math.abs(FR.getCurrentPosition()) + Math.abs(BL.getCurrentPosition()) + Math.abs(BR.getCurrentPosition())) /4 <= degree) {
            angle = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double rx = PIDControl(setpoint, angle);
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double x2 = x * Math.cos(angle / 180 * Math.PI) - y * Math.sin(angle / 180 * Math.PI);
            double y2 = x * Math.sin(angle / 180 * Math.PI) + y * Math.cos(angle / 180 * Math.PI);
            FL.setPower((y2 + x2 + rx)/ denominator);
            FR.setPower((y2 - x2 - rx)/ denominator);
            BL.setPower((y2 - x2 + rx)/ denominator);
            BR.setPower((y2 + x2 - rx)/ denominator);
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        sleep((long)(Break * 1000));
    }

//        while ((Math.abs(FLAsDcMotor.getCurrentPosition()) + Math.abs(FRAsDcMotor.getCurrentPosition())) / 2 <= degree) {
//            angles = -imuAsIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//            rx = anglewrap(Setpoint - angles) / 30;
//            x2 = x * Math.cos(angles / 180 * Math.PI) - y * Math.sin(angles / 180 * Math.PI);
//            y2 = x * Math.sin(angles / 180 * Math.PI) + y * Math.cos(angles / 180 * Math.PI);
//            FLAsDcMotor.setPower((y2 + x2 + rx) / denominator);
//            FRAsDcMotor.setPower(((y2 - x2) - rx) / denominator);
//            BLAsDcMotor.setPower(((y2 - x2) + rx) / denominator);
//            BRAsDcMotor.setPower(((y2 + x2) - rx) / denominator);

    private double PIDControl(double setpoint, double state){
        double error = anglewrap(setpoint - state);
        intergralsum = Plus_Minus(error, 0, 0.3) ? 0 : intergralsum + (error * timer.seconds());
        double derivative = (error - lasterror) / timer.seconds();
        lasterror = error;

        timer.reset();

        double output = (error * Kp) + (intergralsum * Ki) + (derivative * Kd);
        while (0 < output && output < 0.1) output = 0.1;
        while (-0.1 > output && output > 0) output = -0.1;
        return output;
    }
    private double anglewrap(double w_angles) {
        while (w_angles >= 180)  w_angles -= 360;
        while (w_angles <= -180) w_angles += 360;
        return w_angles;
    }
    private boolean Plus_Minus(double input, int check, double range) {
        return check - range < input && input < check + range;
    }
}
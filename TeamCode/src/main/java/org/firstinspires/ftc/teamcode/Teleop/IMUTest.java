package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "IMUTest")
public class IMUTest extends LinearOpMode {
    private DcMotor FL, FR, BL, BR;
    private IMU imu;
    double angle;
    double setpoint;
    double lastsetpoint;
    //PID
    //Degree
//    double Kp = 0.03;
//    double Ki = 0.003;
//    double Kd = 0.003;
    //Radian
    double Kp = 0.8;
    double Ki = 0.3;
    double Kd = 0.2;
    double Kf = 0;
    double error = 0;
    double lastKp = 0;
    double lastKi = 0;
    double lastKd = 0;
    double lastKf = 0;
    double intergralsum = 0;
    double lasterror = 0;
    double output;
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
//        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(new Orientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, 0, 0, 0, 0))));
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(new Orientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS, 0, 0, 0, 0))));
        imu.resetYaw();
    }

    @Override
    public void runOpMode() {
        Init();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                IMUcheck();
                while(gamepad1.back) stop();
            }
        }
    }
    private void IMUcheck(){
        double plusminus = 0.001;
        Kp = gamepad1.a ? Kp - plusminus : (gamepad1.y ? Kp + plusminus : Kp);
        Ki = gamepad1.dpad_left ? Ki - plusminus : (gamepad1.dpad_right ? Ki + plusminus : Ki);
        Kd = gamepad1.x ? Kd - plusminus : (gamepad1.b ? Kd + plusminus : Kd);
        Kf = gamepad1.left_bumper ? Kf - plusminus : (gamepad1.right_bumper ? Kf+plusminus : Kf);
        setpoint = anglewrap(gamepad1.left_trigger > 0 ? setpoint - gamepad1.left_trigger : (gamepad1.right_trigger > 0 ? setpoint + gamepad1.right_trigger : setpoint), 1);
        while(gamepad1.start) intergralsum = 0;

        if(Kp != lastKp || Ki != lastKi || Kd != lastKd || Kf != lastKf || setpoint != lastsetpoint) {
            sleep(250);
        }
        if(setpoint == lastsetpoint) {
            Move();
        }
        lastKp = Kp;
        lastKi = Ki;
        lastKd = Kd;
        lastKf = Kf;
        lastsetpoint = setpoint;

        telemetry.addData("yaw", Math.toDegrees(angle));
//        telemetry.addData("pitch", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
//        telemetry.addData("Roll", imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
        telemetry.addData("Kp", Kp);
        telemetry.addData("Ki", Ki);
        telemetry.addData("Kd", Kd);
        telemetry.addData("Kf", Kf);
        telemetry.addData("setpoint", setpoint);
        telemetry.addData("error", Math.toDegrees(error));
        telemetry.addData("intergralsum",intergralsum);
        telemetry.addData("output",output);
        telemetry.addLine("Kp : -a +y");
        telemetry.addLine("Ki : -Dpad L +Dpad R");
        telemetry.addLine("Kd : -x +b");
        telemetry.addLine("Kf : -L Bum +R Bum");
        telemetry.addLine("Start : reset intergralsum");
        telemetry.addLine("Back : Stop Robot");
        telemetry.update();
    }
    private void Move() {
//        angle = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        angle = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
//        setpoint = Math.toRadians(setpoint);
        double PID = PIDControl(Math.toRadians(setpoint), angle);
        double rx = Math.abs(gamepad1.right_stick_x) > 0 ? gamepad1.right_stick_x : (Plus_Minus(Math.toDegrees(error), 0, 0.3) ? 0 : PID);
//        double rx = Math.abs(gamepad1.right_stick_x) > 0 ? gamepad1.right_stick_x :PID;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        FL.setPower((y + x + rx)/ denominator);
        FR.setPower((y - x - rx)/ denominator);
        BL.setPower((y - x + rx)/ denominator);
        BR.setPower((y + x - rx)/ denominator);
    }
    private double PIDControl(double setpoint, double state){
        error = anglewrap(setpoint - state, 0);
        intergralsum = Plus_Minus(Math.toDegrees(error), 0, 0.3) ? 0 : intergralsum + (error * timer.seconds());
        double derivative = (error - lasterror) / timer.seconds();
        lasterror = error;

        timer.reset();

        output = (error * Kp) + (intergralsum * Ki) + (derivative * Kd);
        while (0 < output && output < 0.2) output = 0.2;
        while (-0.2 < output && output < 0) output = -0.2;
        return output;
    }

    private boolean Plus_Minus(double input, int check, double range) {
        return check - range < input && input < check + range;
    }
    private double anglewrap(double w_angles, int set) {
        if(set == 0) {
            while (w_angles > Math.PI) w_angles -= 2 * Math.PI;
            while (w_angles < -Math.PI) w_angles += 2 * Math.PI;
        }
        if(set == 1) {
            while (w_angles >= 180) w_angles -= 360;
            while (w_angles <= -180) w_angles += 360;
        }
        return w_angles;
    }
}
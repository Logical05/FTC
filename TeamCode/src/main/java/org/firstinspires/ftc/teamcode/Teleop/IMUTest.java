package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "IMUTest")
public class IMUTest extends LinearOpMode {
    Robot robot = new Robot();
    IMU imu;
    Servo LA, RA, K;
    DcMotor FL, FR, BL, BR, B, LL, RL;
    double setpoint;
    double lastsetpoint;
    //PID
    //Degree
//    double Kp = 0.03;
//    double Ki = 0.003;
//    double Kd = 0.003;
    //Radian
    double yaw, Kp = 0.8;
    double Ki = 0.3;
    double Kd = 0.1;
    double Kf = 0;
    double lastKp = 0;
    double lastKi = 0;
    double lastKd = 0;
    double lastKf = 0;
    private void Init(){
        // HardwareMap
        imu = hardwareMap.get(IMU.class,     "imu");
        FL  = hardwareMap.get(DcMotor.class, "Front_Left");
        FR  = hardwareMap.get(DcMotor.class, "Front_Right");
        BL  = hardwareMap.get(DcMotor.class, "Back_Left");
        BR  = hardwareMap.get(DcMotor.class, "Back_Right");
        B   = hardwareMap.get(DcMotor.class, "Base");
        LL  = hardwareMap.get(DcMotor.class, "Left_Lift");
        RL  = hardwareMap.get(DcMotor.class, "Right_Lift");
        LA  = hardwareMap.get(Servo.class,   "Left_Arm");
        RA  = hardwareMap.get(Servo.class,   "Right_Arm");
        K   = hardwareMap.get(Servo.class,   "Keeper");

        // Initialize Robot
        robot.Initialize(imu, DcMotor.RunMode.RUN_WITHOUT_ENCODER, FL, FR, BL, BR, B, LL, RL,
                0, LA, RA, 0, K);
    }

    @Override
    public void runOpMode() {
        Init();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                yaw = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                IMUcheck();
                if (gamepad1.touchpad) imu.resetYaw();
            }
        }
    }
    private void IMUcheck(){
        double plusminus = 0.001;
        Kp = gamepad1.a ? Kp - plusminus : (gamepad1.y ? Kp + plusminus : Kp);
        Ki = gamepad1.dpad_left ? Ki - plusminus : (gamepad1.dpad_right ? Ki + plusminus : Ki);
        Kd = gamepad1.x ? Kd - plusminus : (gamepad1.b ? Kd + plusminus : Kd);
        Kf = gamepad1.left_bumper ? Kf - plusminus : (gamepad1.right_bumper ? Kf + plusminus : Kf);
        setpoint = robot.AngleWrap(gamepad1.left_trigger > 0 ? setpoint - 1E-2 : (gamepad1.right_trigger > 0 ? setpoint + 1E-2 : setpoint));

        if(Kp != lastKp || Ki != lastKi || Kd != lastKd || Kf != lastKf || setpoint != lastsetpoint) sleep(250);

        if(setpoint == lastsetpoint) Move();

        lastKp = Kp;
        lastKi = Ki;
        lastKd = Kd;
        lastKf = Kf;
        lastsetpoint = setpoint;

        telemetry.addData("yaw", Math.toDegrees(yaw));
        telemetry.addData("Kp", Kp);
        telemetry.addData("Ki", Ki);
        telemetry.addData("Kd", Kd);
        telemetry.addData("Kf", Kf);
        telemetry.addData("setpoint", Math.toDegrees(setpoint));
        telemetry.addData("error", Math.toDegrees(robot.error));
        telemetry.addLine("Kp : -a +y");
        telemetry.addLine("Ki : -Dpad L +Dpad R");
        telemetry.addLine("Kd : -x +b");
        telemetry.addLine("Kf : -L Bum +R Bum");
        telemetry.update();
    }
    private void Move() {
        double Lx =  gamepad1.left_stick_x;
        double Ly = -gamepad1.left_stick_y;
        double Rx =  gamepad1.right_stick_x;
        double[] K_PID = {Kp, Ki, Kd};
        double PID = robot.PIDControl(0, yaw, K_PID);
        // Rotate Condition
        double R = Math.abs(Rx) > 0 ? Rx :
                robot.Plus_Minus(Math.toDegrees(robot.error), 0, 0.45) ? 0 : PID;
        // Denominator for division to get no more than 1
        double D = Math.max(Math.abs(Ly) + Math.abs(Lx) + Math.abs(R), 1);
        robot.MovePower((Ly + Lx + R)/ D, (Ly - Lx - R)/ D,
                (Ly - Lx + R)/ D,  (Ly + Lx - R)/ D);
        robot.PID_timer.reset();
    }

}
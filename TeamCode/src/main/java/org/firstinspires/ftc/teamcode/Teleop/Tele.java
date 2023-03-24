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
    Servo LA, RA, K;
    DcMotor FL, FR, BL, BR, B;

    /** Variables */
    double K_pos = 0;

    private void Init(){
        // HardwareMap
        imu = hardwareMap.get(IMU.class,     "imu");
        FL  = hardwareMap.get(DcMotor.class, "Front_Left");
        FR  = hardwareMap.get(DcMotor.class, "Front_Right");
        BL  = hardwareMap.get(DcMotor.class, "Back_Left");
        BR  = hardwareMap.get(DcMotor.class, "Back_Right");
        B   = hardwareMap.get(DcMotor.class, "Base");
        LA  = hardwareMap.get(Servo.class,   "Left_Arm");
        RA  = hardwareMap.get(Servo.class,   "Right_Arm");
        K   = hardwareMap.get(Servo.class,   "Keeper");

        // Initialize Robot
        robot.Initialize(imu, DcMotor.RunMode.RUN_WITHOUT_ENCODER, FL, FR, BL, BR, B, 0.35, LA, RA, 0, K);
    }

    private void Movement(){
        double Lx =  gamepad1.left_stick_x;
        double Ly = -gamepad1.left_stick_y;
        double Rx =  gamepad1.right_stick_x;
        double[] K_PID = {0.8, 0.3, 0.2};
        double PID = robot.PIDControl(0, K_PID);
        // Rotate Condition
        double r = Math.abs(Rx) > 0 ? Rx :
                (robot.Plus_Minus(Math.toDegrees(robot.error), 0, 0.45) ? 0 : PID);
        // Denominator for division to get no more than 1
        double d = Math.max(Math.abs(Ly) + Math.abs(Lx) + Math.abs(r), 1);
        robot.MovePower((Ly + Lx + r)/ d, (Ly - Lx - r)/ d,
                        (Ly - Lx + r)/ d, (Ly + Lx - r)/ d);
        robot.PID_timer.reset();
    }

    private void Keep(){
        K_pos = gamepad1.left_bumper ? 0 : (gamepad1.right_bumper ? 0.25 : K_pos);
        K.setPosition(K_pos);
    }

    @Override
    public void runOpMode() {
        Init();
        waitForStart();
        if (opModeIsActive()) {
            robot.PID_timer.reset();
            while (opModeIsActive()) {
                if(gamepad1.touchpad) imu.resetYaw();
                Movement();
                Keep();
                telemetry.addData("yaw", Math.toDegrees(robot.yaw));
                telemetry.addData("Encoder", B.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}
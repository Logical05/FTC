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
    DcMotor FL, FR, BL, BR, B, LL, RL;

    /** Variables */
    double K_pos = 0;
    double setpoint = 0;

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
                0.35, LA, RA, 0, K);
    }

    private void Movement(){
        double Lx =  gamepad1.left_stick_x;
        double Ly = -gamepad1.left_stick_y;
        double Rx =  gamepad1.right_stick_x;
        double Ry =  gamepad1.right_stick_y;
        setpoint = Rx > 0.5 && Math.abs(Ry) < 0.5 ? 90 :
                   Rx < 0.5 && Math.abs(Ry) < 0.5 ? -90 :
                   Ry > 0.5 && Math.abs(Rx) < 0.5 ? 0 :
                   Ry < 0.5 && Math.abs(Rx) < 0.5 ? 180 : setpoint;
        double[] K_PID = {0.83, 0.35, 0.1};
        double PID = robot.PIDControl(setpoint, K_PID);
        // Rotate Condition
//        double R = Math.abs(Rx) > 0 ? Rx :
//                   robot.Plus_Minus(Math.toDegrees(robot.error), 0, 0.45) ? 0 : PID;
        double R = robot.Plus_Minus(Math.toDegrees(robot.error), 0, 0.45) ? 0 : PID;
        // Denominator for division to get no more than 1
        double D = Math.max(Math.abs(Ly) + Math.abs(Lx) + Math.abs(R), 1);
        robot.MovePower((Ly + Lx + R)/ D, (Ly - Lx - R)/ D,
                        (Ly - Lx + R)/ D,  (Ly + Lx - R)/ D);
        robot.PID_timer.reset();
    }


    private void Keep() {
        K_pos = gamepad1.left_bumper  ? 0 :
                gamepad1.right_bumper ? 0.25 : K_pos;
        K.setPosition(K_pos);
    }

    private void Lift() {
        int LL_pos = LL.getCurrentPosition();
        int RL_pos = RL.getCurrentPosition();
        double LT = gamepad1.left_trigger;
        double RT = gamepad1.right_trigger;
        double High = LL_pos >= robot.Max_Lift || RL_pos >= robot.Max_Lift ? 0 :  LT;
        double Low  = LL_pos <= 0              || RL_pos <= 0              ? 0 : -RT;
        double Power = LT >= 0.5 ? High :
                       RT >= 0.5 ? Low  : 0;
        LL.setPower(Power);
        RL.setPower(Power);
    }

    @Override
    public void runOpMode() {
        Init();
        waitForStart();
        if (opModeIsActive()) {
            robot.PID_timer.reset();
            while (opModeIsActive()) {
                if (gamepad1.touchpad) imu.resetYaw();
                Movement();
                Keep();
                Lift();
                telemetry.addData("yaw", Math.toDegrees(robot.yaw));
                telemetry.addData("Encoder", B.getCurrentPosition());
                telemetry.addData("setpoint", setpoint);
                telemetry.update();
            }
        }
    }
}
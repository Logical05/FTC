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
    DcMotor FL, FR, BL, BR, B;

    /** Variables */


    private void Init(){
        // HardwareMap
        imu  = hardwareMap.get(IMU.class,     "imu");
        FL   = hardwareMap.get(DcMotor.class, "Front_Left");
        FR   = hardwareMap.get(DcMotor.class, "Front_Right");
        BL   = hardwareMap.get(DcMotor.class, "Back_Left");
        BR   = hardwareMap.get(DcMotor.class, "Back_Right");
//        B    = hardwareMap.get(DcMotor.class, "Base");
        LA   = hardwareMap.get(Servo.class,   "Left_Arm");
        RA   = hardwareMap.get(Servo.class,   "Right_Arm");

        // Initialize Robot
        robot.Initialize(imu, DcMotor.RunMode.RUN_WITHOUT_ENCODER, FL, FR, BL, BR, B, 0.35, LA, RA);
    }

    private void Movement(){
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double PID = robot.PIDControl(0);
        // Rotate Condition
        double r = Math.abs(gamepad1.right_stick_x) > 0 ? gamepad1.right_stick_x :
                (robot.Plus_Minus(Math.toDegrees(robot.error), 0, 0.45) ? 0 : PID);
        // Denominator for division to get no more than 1
        double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);
        robot.MovePower((y + x + r)/ d, (y - x - r)/ d,
                        (y - x + r)/ d, (y + x - r)/ d);
        robot.PID_timer.reset();

        telemetry.addData("yaw", Math.toDegrees(robot.yaw));
//        telemetry.addData("Encoder", B.getCurrentPosition());
        telemetry.update();
    }

    private void Turn_Base(int angle) {
        int cpr = 1440;              // Encoder counts per revolution
        int c = (angle * cpr)/ 360;  // Encoder counts
        B.setTargetPosition(c);
        B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        B.setPower(0.5);
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
            }
        }
    }
}

package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
    int Base_angle = 0;
    double yaw, K_pos=0, setpoint=0;

    double Parabola;
    int CurrentPosition;

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
        double Lx   =  gamepad1.left_stick_x;
        double Ly   = -gamepad1.left_stick_y;
        double Rx   =  gamepad1.right_stick_x;
        double Ry   = -gamepad1.right_stick_y;
        setpoint = Rx >  0.75 && Math.abs(Ry) < 0.75 ?  Math.toRadians( 90) :
                   Rx < -0.75 && Math.abs(Ry) < 0.75 ?  Math.toRadians(-90) :
                   Ry >  0.75 && Math.abs(Rx) < 0.75 ?  Math.toRadians(  0) :
                   Ry < -0.75 && Math.abs(Rx) < 0.75 ?  Math.toRadians(180) : setpoint;
        double[] K_PID_move    = {0.8, 0.2, 0.05};
        double[] K_PID_notmove = {1.7, 0.3, 0.05};
        double[] K_PID = Lx < 0.25 && Ly < 0.25 ? K_PID_move : K_PID_notmove;
        double PID = robot.PIDControl(setpoint, yaw, K_PID);
        double X1 = Math.abs(Lx) > 0.25 && Math.abs(Ly) < 0.25 ? Lx : 0;
        double Y1 = Math.abs(Ly) > 0.25 && Math.abs(Lx) < 0.25 ? Ly : 0;
        double X2 = (Math.cos(yaw) * X1) - (Math.sin(yaw) * Y1);
        double Y2 = (Math.sin(yaw) * X1) + (Math.cos(yaw) * Y1);
        // Rotate Condition
        double R = robot.Plus_Minus(Math.toDegrees(robot.error), 0, 0.45) ? 0 : PID;
        // Denominator for division to get no more than 1
        double D = Math.max(Math.abs(X2) + Math.abs(Y2) + Math.abs(R), 1);
        robot.MovePower((Y2 + X2 + R) / D, (Y2 - X2 - R) / D,
                        (Y2 - X2 + R) / D,  (Y2 + X2 - R) / D);
        robot.PID_timer.reset();
    }

    private void Keep() {
        K_pos = gamepad1.left_bumper  ? 0 :
                gamepad1.right_bumper ? 0.25 : K_pos;
        K.setPosition(K_pos);
    }

    private void Lift() {
        CurrentPosition = Math.max(Math.abs(LL.getCurrentPosition()), Math.abs(RL.getCurrentPosition()));
        double Min_Power = 0.5;
        double Max_Lift = robot.Max_Lift;
        double k = 20;
        double h = Max_Lift / 2;
        double Latus_Rectum = -k / (h * h);
        Parabola = Parabola <= 0 ? Min_Power : (Latus_Rectum * ((CurrentPosition - h) * (CurrentPosition - h))) + k + Min_Power;
        double High = CurrentPosition >= Max_Lift ?  0 :
                      Parabola        >= 1        ?  1 : Parabola;
        double Low  = CurrentPosition <= 0        ?  0 :
                      Parabola        >= 1        ? -1 : -Parabola;
        double LT = gamepad1.left_trigger;
        double RT = gamepad1.right_trigger;
        double Power = LT >= 0.25 ? High :
                       RT >= 0.25 ? Low  : 0;
        LL.setPower(Power);
        RL.setPower(Power);
    }

    private void Turn_Base (){
        Base_angle = gamepad1.dpad_up    ?   0 :
                     gamepad1.dpad_left  ? -90 :
                     gamepad1.dpad_right ?  90 : Base_angle;
        robot.Turn_Base(Base_angle);
    }

    @Override
    public void runOpMode() {
        Init();
        waitForStart();
        if (opModeIsActive()) {
            imu.resetYaw();
            robot.PID_timer.reset();
            while (opModeIsActive()) {
                if (gamepad1.touchpad) imu.resetYaw();
                yaw = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                Movement();
                Keep();
                Lift();
                Turn_Base();
                telemetry.addData("yaw", Math.toDegrees(yaw));
                telemetry.addData("Encoder", B.getCurrentPosition());
                telemetry.addData("LL", LL.getCurrentPosition());
                telemetry.addData("RL", RL.getCurrentPosition());
                telemetry.addData("Parabola", Parabola);
                telemetry.update();
            }
        }
    }
}
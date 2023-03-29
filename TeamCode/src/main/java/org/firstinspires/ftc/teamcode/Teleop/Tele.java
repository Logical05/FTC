package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name="TeleOp")
public class Tele extends LinearOpMode {
    /** Usage External Class */
    Robot robot = new Robot();

    /** Hardware */
    IMU imu;
    Servo LA, RA, K;
    DcMotor FL, FR, BL, BR, B, LL, RL;

    /** Parabola Variables */
    final int    h               = robot.Max_Lift / 2;
    final double k               = 20;
    final double Latus_Rectum    = -k / (h * h);

    /** Variables */
    double yaw;
    public static int Base_angle = 0;
    public static double K_pos=0, setpoint=0;
    public static double p=0, i=0, d=0;

    double Parabola;

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
                0.345, LA, RA, 0, K);

        // Usage FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    private void Movement(){
        double Lx =  gamepad1.left_stick_x;
        double Ly = -gamepad1.left_stick_y;
        double Rx =  gamepad1.right_stick_x;
        double Ry = -gamepad1.right_stick_y;
        setpoint = Rx >  0.75 && Math.abs(Ry) < 0.75 ?  Math.toRadians( 90) :
                   Rx < -0.75 && Math.abs(Ry) < 0.75 ?  Math.toRadians(-90) :
                   Ry >  0.75 && Math.abs(Rx) < 0.75 ?  Math.toRadians(  0) :
                   Ry < -0.75 && Math.abs(Rx) < 0.75 ?  Math.toRadians(180) : setpoint;
        double JoyStickLeft_angle = Math.toDegrees(robot.AngleWrap(Math.atan2(Ly,Lx)));
        double   X1     = (-45 <= JoyStickLeft_angle && JoyStickLeft_angle <= 45) || (-135 >= JoyStickLeft_angle || JoyStickLeft_angle >= 135) ? Lx : 0;
        double   Y1     = (45  <  JoyStickLeft_angle && JoyStickLeft_angle < 135) || (-45  >  JoyStickLeft_angle && JoyStickLeft_angle > -135) ? Ly : 0;
        double   X2     = (Math.cos(yaw) * X1) - (Math.sin(yaw) * Y1);
        double   Y2     = (Math.sin(yaw) * X1) + (Math.cos(yaw) * Y1);
        double[] K_PID  = {p, i, d};
        double   PID    = robot.PIDControl(K_PID, Math.toRadians(setpoint), yaw);
        // Rotate Condition
        double R = robot.Plus_Minus(Math.toDegrees(robot.Error), 0, 0.45) ? 0 : PID;
        // Denominator for division to get no more than 1
        double D = Math.max(Math.abs(X2) + Math.abs(Y2) + Math.abs(R), 1);
        robot.MovePower((Y2 + X2 + R) / D, (Y2 - X2 - R) / D,
                        (Y2 - X2 + R) / D,  (Y2 + X2 - R) / D);
    }

    private void Keep() {
        K_pos = gamepad1.left_bumper  ? 0 :
                gamepad1.right_bumper ? 0.25 : K_pos;
        K.setPosition(K_pos);
    }

    private void Lift() {
        double Min_Power = 0.5;
        int CurrentPosition = Math.max(LL.getCurrentPosition(), RL.getCurrentPosition());
        double ParabolaPosition = CurrentPosition <= 0 ? 0 : Math.min(CurrentPosition, robot.Max_Lift);
        Parabola = (Latus_Rectum * ((ParabolaPosition - h) * (ParabolaPosition - h))) + k + Min_Power;
        double High = CurrentPosition >= robot.Max_Lift ?  0 :
                      Parabola        >= 1              ?  1 : Parabola;
        double Low  = CurrentPosition <= 0              ?  0 :
                      Parabola        >= 1              ? -1 : -Parabola;
        double LT = gamepad1.left_trigger;
        double RT = gamepad1.right_trigger;
        double Power = LT >= 0.25 ? High :
                       RT >= 0.25 ? Low  : 0.1;
        LL.setPower(Power);
        RL.setPower(Power);
    }

    private void Turn_Base() {
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
            robot.PID_timer.reset();
            while (opModeIsActive()) {
                if (gamepad1.touchpad) imu.resetYaw();
                yaw = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                Movement();
                Keep();
                Lift();
                Turn_Base();
                telemetry.addData("yaw", Math.toDegrees(yaw));
                telemetry.addData("Base", B.getCurrentPosition());
                telemetry.addData("LL", LL.getCurrentPosition());
                telemetry.addData("RL", RL.getCurrentPosition());
                telemetry.addData("Parabola", Parabola);
                telemetry.addData("setpoint",setpoint);
                telemetry.update();
            }
        }
    }
}
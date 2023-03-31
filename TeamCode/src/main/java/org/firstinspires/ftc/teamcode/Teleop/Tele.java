package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.Utilize.AngleWrap;
import static org.firstinspires.ftc.teamcode.Utilize.atTargetRange;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name="TeleOp")
public class Tele extends LinearOpMode {
//    public static double pX=3, iX=1, dX=0.1;
    public static double p=0, i=0, d=0;
    /** Usage External Class */
    Robot      robot = new Robot();
    Controller pid_R = new Controller(1.3, 0.085, 0.09);

    /** Hardware */
    IMU imu;
    Servo LA, RA, K;
    DcMotor FL, FR, BL, BR, B, LL, RL;

    /** Variables */
    int Levels = 0;
    double yaw, K_pos=0, setpoint=0;
    public static int Base_angle = 0;
    boolean Base_isBusy=false, Lift_isAuto =false;

    private void Init() {
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

    private void Movement() {
        double Lx =  gamepad1.left_stick_x;
        double Ly = -gamepad1.left_stick_y;
        double Rx =  gamepad1.right_stick_x;
        double Ry = -gamepad1.right_stick_y;
        setpoint  = Rx >  0.75 && Math.abs(Ry) < 0.75 ? Math.toRadians( 90) :
                    Rx < -0.75 && Math.abs(Ry) < 0.75 ? Math.toRadians(-90) :
                    Ry >  0.75 && Math.abs(Rx) < 0.75 ? Math.toRadians(  0) :
                    Ry < -0.75 && Math.abs(Rx) < 0.75 ? Math.toRadians(180) : setpoint;
        double JoyStickLeft_angle = Math.toDegrees(AngleWrap(Math.atan2(Ly,Lx)));
        boolean X_axis = (-45 <= JoyStickLeft_angle && JoyStickLeft_angle <= 45) || Math.abs(JoyStickLeft_angle) >= 135;
        boolean Y_axis = (45  <  JoyStickLeft_angle && JoyStickLeft_angle < 135) || (-45  >  JoyStickLeft_angle && JoyStickLeft_angle > -135);
        double  X1     = X_axis ? Lx : 0;
        double  Y1     = Y_axis ? Ly : 0;
        double  X2     = (Math.cos(yaw) * X1) - (Math.sin(yaw) * Y1);
        double  Y2     = (Math.sin(yaw) * X1) + (Math.cos(yaw) * Y1);
        // Rotate
        double R = pid_R.Calculate(AngleWrap(setpoint - yaw));
        // Denominator for division to get no more than 1
        double D = Math.max(Math.abs(X2) + Math.abs(Y2) + Math.abs(R), 1);
        robot.MovePower((Y2 + X2 + R) / D, (Y2 - X2 - R) / D,
                        (Y2 - X2 + R) / D,  (Y2 + X2 - R) / D);
    }

    private void Keep() {
        K_pos = gamepad1.right_bumper ? 0    :
                gamepad1.left_bumper  ? 0.25 : K_pos;
        K.setPosition(K_pos);
    }

    private void Lift() {
        int CurrentPosition = Math.max(LL.getCurrentPosition(), RL.getCurrentPosition());
        double  LT = gamepad1.right_trigger;
        double  RT = gamepad1.left_trigger;
        boolean Cr = gamepad1.cross;
        boolean Sq = gamepad1.square;
        boolean Tr = gamepad1.triangle;
        boolean Ci = gamepad1.circle;
        Levels = Cr ? robot.Ground_Junction :
                 Sq ? robot.Low_Junction    :
                 Tr ? robot.Medium_Junction :
                 Ci ? robot.High_Junction   : Levels;
        boolean Auto_Condition = (LT < 0.25) && (RT < 0.25) && (Lift_isAuto || Cr || Sq || Tr || Ci);
        double Min_Power  = 0.1;
        double Auto       = (atTargetRange(CurrentPosition, Levels, 10) ? Min_Power : (CurrentPosition < Levels ? 1 : -1));
        double Control    = (LT >= 0.25 ? (CurrentPosition >= robot.Max_Lift ? 0 : 1) : (RT >= 0.25 ? (CurrentPosition <= 0 ? 0 : -1) : Min_Power));
        double Lift_Power = Auto_Condition ? Auto : Control;
        Lift_isAuto = Auto_Condition;
        LL.setPower(Lift_Power);
        RL.setPower(Lift_Power);
    }

    private void Turn_Base() {
        boolean D_Up    = gamepad1.dpad_up;
        boolean D_Left  = gamepad1.dpad_left;
        boolean D_Right = gamepad1.dpad_right;
        Base_angle = D_Up    ?   0 :
                     D_Left  ? -90 :
                     D_Right ?  90 : Base_angle;
        if (Base_isBusy || (D_Up || D_Left || D_Right)) {
            Base_isBusy = robot.Turn_Base(Base_angle, p, i, d);
            return;
        }
        B.setPower(0);
        B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void runOpMode() {
        Init();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (gamepad1.ps) imu.resetYaw();
                yaw = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                Movement();
                Keep();
                Lift();
                Turn_Base();
                telemetry.addData("yaw", Math.toDegrees(yaw));
                telemetry.addData("setpoint",Math.toDegrees(setpoint));
                telemetry.addData("Integral",pid_R.Integral);
                telemetry.addData("Derivative",pid_R.Derivative);
                telemetry.addData("Base", B.getCurrentPosition());
                telemetry.addData("Base_angle", Base_angle*4);
                telemetry.addData("LL", LL.getCurrentPosition());
                telemetry.addData("RL", RL.getCurrentPosition());
                telemetry.addData("Levels", Levels);
                telemetry.update();
            }
        }
    }
}
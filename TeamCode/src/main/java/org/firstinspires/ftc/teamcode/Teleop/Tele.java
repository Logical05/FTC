package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.Utilize.AngleWrap;
import static org.firstinspires.ftc.teamcode.Utilize.atTargetRange;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

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
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Config
@TeleOp(name="Tele")
public class Tele extends LinearOpMode {
    /** PID Variables */
    public static double R_Kp=1.6, R_Ki=0.01, R_Kd=0.09;

    /** Usage External Class */
    Robot      robot = new Robot();
    Controller pid_R = new Controller(R_Kp, R_Ki, R_Kd, 0);

    /** Hardware */
    IMU imu;
    Servo LA, RA, K;
    DcMotorEx FL, FR, BL, BR, B, LL, ML, RL;

    /** Variables */
    int Level, CurrentPosition=0;
    double yaw, K_pos=0, Arm_pos=0, setpoint=Math.toRadians(180);
    boolean Base_atSetpoint=true, Lift_isAuto=false, LB_press=false, RB_press=false, ArmUp=true, isKeep=true, LT_press=false, RT_press=false;
    public static int Base_angle = 0;

    private void Init() {
        // HardwareMap
        imu = hardwareMap.get(IMU.class,     "imu");
        FL  = hardwareMap.get(DcMotorEx.class, "Front_Left");
        FR  = hardwareMap.get(DcMotorEx.class, "Front_Right");
        BL  = hardwareMap.get(DcMotorEx.class, "Back_Left");
        BR  = hardwareMap.get(DcMotorEx.class, "Back_Right");
        B   = hardwareMap.get(DcMotorEx.class, "Base");
        LL  = hardwareMap.get(DcMotorEx.class, "Left_Lift");
        ML  = hardwareMap.get(DcMotorEx.class, "Middle_Lift");
        RL  = hardwareMap.get(DcMotorEx.class, "Right_Lift");
        LA  = hardwareMap.get(Servo.class,   "Left_Arm");
        RA  = hardwareMap.get(Servo.class,   "Right_Arm");
        K   = hardwareMap.get(Servo.class,   "Keeper");

        // Initialize Robot
        robot.Initialize(imu, DcMotor.RunMode.RUN_WITHOUT_ENCODER, FL, FR, BL, BR, B, LL, ML, RL,
                Arm_pos, LA, RA, K_pos, K);

        // Show FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    private void WaitForStart() {
        boolean Base_atSetpoint = false;
        while (!isStarted() && !isStopRequested()) {
            if(gamepad1.share){
                LL .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ML .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RL .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LL .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ML .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RL .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    private double SpeedLimit(double joystick, double speed){
        if(CurrentPosition >= robot.Medium_Junction) Range.clip(joystick, -speed, speed);
        return joystick;
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
        double  X1     = X_axis ? SpeedLimit(Lx, 0.8) : 0;
        double  Y1     = Y_axis ? SpeedLimit(Ly, 0.8) : 0;
        double  X2     = (Math.cos(yaw) * X1) - (Math.sin(yaw) * Y1);
        double  Y2     = (Math.sin(yaw) * X1) + (Math.cos(yaw) * Y1);
        // Rotate
        pid_R.setPIDF(R_Kp, R_Ki, R_Kd, 0);
        double R = pid_R.Calculate(AngleWrap(setpoint - yaw));
        // Denominator for division to get no more than 1
        double D = Math.max(Math.abs(X2) + Math.abs(Y2) + Math.abs(R), 1);
        robot.MovePower((Y2 + X2 + R) / D, (Y2 - X2 - R) / D,
                        (Y2 - X2 + R) / D,  (Y2 + X2 - R) / D);
    }

    private void Keep() {
        boolean RB = gamepad1.right_bumper;
        boolean LB = gamepad1.left_bumper;
        LB_press = LB || LB_press;
        RB_press = RB || RB_press;
        if (LB_press && !LB) {
            LB_press = false;
            if (isKeep && ArmUp) {
                ArmUp = false;
                robot.setArmPosition(0.34);
                return;
            }
            if (isKeep && !ArmUp) {
                isKeep = false;
                K.setPosition(0.2);
                return;
            }
        }
        if (RB_press && !RB) {
            RB_press = false;
            if (!isKeep && !ArmUp) {
                isKeep = true;
                K.setPosition(0);
                return;
            }
            if (isKeep && !ArmUp) {
                ArmUp = true;
                robot.setArmPosition(0);
            }
        }
    }

    private void Lift() {
        CurrentPosition = Math.max(LL.getCurrentPosition(), Math.max(ML.getCurrentPosition(), RL.getCurrentPosition()));
        double  LT     = gamepad1.left_trigger;
        double  RT     = gamepad1.right_trigger;
        boolean D_Up   = gamepad1.dpad_up;
        boolean D_Down = gamepad1.dpad_down;
        int[]   Levels = {-50, robot.Ground_Junction, robot.Low_Junction, robot.Medium_Junction, robot.High_Junction};
        LT_press = LT >= 0.25 || LT_press;
        RT_press = RT >= 0.25 || RT_press;
        if (LT_press && (LT < 0.25)) {
            LT_press = false;
            Level--;
        }
        if (RT_press && (RT < 0.25)) {
            RT_press = false;
            Level++;
        }
        Level = D_Down || D_Up ? (CurrentPosition >= robot.High_Junction   ? 4 :
                            CurrentPosition >= robot.Medium_Junction ? 3 :
                            CurrentPosition >= robot.Low_Junction    ? 2 :
                            CurrentPosition >= robot.Ground_Junction ? 1 :
                            CurrentPosition >=                     0 ? 0 : Level) : Level;

        Level = Range.clip(Level,0, 4);
        boolean Auto_Condition = !(D_Up) && !(D_Down) && (Lift_isAuto || LT >= 0.25 || RT >= 0.25);
        double Min_Power  = 0.05;
        double Auto       = (atTargetRange(CurrentPosition, Levels[Level], 10) ? Min_Power : (CurrentPosition < Levels[Level] ? 1 : -0.75));
//        double Control    = (D_Up ? (CurrentPosition >= robot.Max_Lift ? 0 : 1) : (D_Down ? (CurrentPosition <= 0 ? 0 : -1) : Min_Power));
        double Control    = (D_Up ? 1 : (D_Down ? (CurrentPosition <= -50 ? 0 : -1) : Min_Power));
        double Lift_Power = Auto_Condition ? Auto : Control;
        Lift_isAuto = Auto_Condition;
        robot.LiftPower(Lift_Power);
    }

    private void Turn_Base() {
        boolean CR      = gamepad1.cross;
        boolean SQ      = gamepad1.square;
        boolean CC      = gamepad1.circle;
        boolean D_Left  = gamepad1.dpad_left;
        boolean D_Right = gamepad1.dpad_right;
        Base_angle = CR    ?   0 :
                     SQ    ?  90 :
                     CC    ? -90 : Base_angle;
        if(gamepad1.share) {
            B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if(D_Left || D_Right){
            Base_atSetpoint = true;
            B.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double B_Power = D_Left ? 0.5 : (D_Right ? -0.5 : 0);
            B.setPower(B_Power);
        }

        if (!Base_atSetpoint || CR || SQ || CC) {
            Base_atSetpoint = robot.Turn_Base(Base_angle, 0.65, 1);
            return;
        }
        if (!D_Left && !D_Right) {
            B.setPower(0);
            B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    @Override
    public void runOpMode() {
        Init();
        WaitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (gamepad1.ps){
                    imu.resetYaw();
                    setpoint = 0;
                }
                yaw = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                Movement();
                Keep();
                Lift();
                Turn_Base();
                telemetry.addData("yaw", Math.toDegrees(yaw));
                telemetry.addData("setpoint",Math.toDegrees(setpoint));
//                telemetry.addData("Base", B.getCurrentPosition());
//                telemetry.addData("Base_angle", (int) (Base_angle * robot.Gear_60_HD_HEX) / 360);
//                telemetry.addData("Base_atSetpoint", Base_atSetpoint);
                telemetry.addData("LL", LL.getCurrentPosition());
                telemetry.addData("ML", ML.getCurrentPosition());
                telemetry.addData("RL", RL.getCurrentPosition());
                telemetry.addData("FL", FL.getCurrentPosition());
                telemetry.addData("FR", FR.getCurrentPosition());
                telemetry.addData("BL", BL.getCurrentPosition());
                telemetry.addData("BR", BR.getCurrentPosition());
//                telemetry.addData("Levels", Levels);
                telemetry.update();
            }
        }
    }
}
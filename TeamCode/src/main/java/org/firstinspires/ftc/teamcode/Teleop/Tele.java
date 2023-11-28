package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static org.firstinspires.ftc.teamcode.Utilize.*;
import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;

@Config
@TeleOp(name="Tele")
public class Tele extends LinearOpMode {
    /** Usage External Class */
    Controller pid_R = new Controller(1.6, 0.01, 0.09, 0);

    /** Hardware */
    IMU imu;
    Servo LLL, LRL, LA, RA, LH, RH, K;
    DcMotorEx FL, FR, BL, BR, LL, RL, PU, V;

    /** Variables */
    public static double setpoint = Math.toDegrees(0), liftAng = 0, hoistAng = 0, armAng = 0;
    boolean SqPressed = false, VisBusy = false;

    private void Init() {
        // HardwareMap
        imu = hardwareMap.get(IMU.class,       "imu");
        FL  = hardwareMap.get(DcMotorEx.class, "Front_Left");   FR  = hardwareMap.get(DcMotorEx.class, "Front_Right");
        BL  = hardwareMap.get(DcMotorEx.class, "Back_Left");    BR  = hardwareMap.get(DcMotorEx.class, "Back_Right");
        LL  = hardwareMap.get(DcMotorEx.class, "Left_Lift");    RL  = hardwareMap.get(DcMotorEx.class, "Right_Lift");
        PU  = hardwareMap.get(DcMotorEx.class, "Pull_Up");      V   = hardwareMap.get(DcMotorEx.class, "Vacuum");
        LLL = hardwareMap.get(Servo.class,     "Lift_LL");      LRL = hardwareMap.get(Servo.class,     "Lift_RL");
        LA  = hardwareMap.get(Servo.class,     "Left_Arm");     RA  = hardwareMap.get(Servo.class,     "Right_Arm");
        LH  = hardwareMap.get(Servo.class,     "Left_Hoist");   RH  = hardwareMap.get(Servo.class,     "Right_Hoist");
        K   = hardwareMap.get(Servo.class,     "Keep");

        // Initialize Robot
        Robot.Initialize(imu, DcMotor.RunMode.RUN_WITHOUT_ENCODER, FL, FR, BL, BR, LL, RL, PU, V,
                         LLL, LRL, LA, RA, LH, RH, K, 0);

        // Show FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    private void Movement() {
        double X1  =  gamepad1.left_stick_x;
        double Y1  = -gamepad1.left_stick_y;
        double yaw =  imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double X2  =  (Math.cos(yaw) * X1) - (Math.sin(yaw) * Y1);
        double Y2  =  (Math.sin(yaw) * X1) + (Math.cos(yaw) * Y1);
        // Rotate
        double R =  pid_R.Calculate(WrapRads(setpoint - yaw));
        double X = -gamepad1.right_stick_x;
        if (X != 0) {
            R = X;
            setpoint = yaw;
        }
        // Denominator for division to get no more than 1
        double D = Math.max(Math.abs(X2) + Math.abs(Y2) + Math.abs(R), 1);
        Robot.MovePower((Y2 + X2 + R) / D, (Y2 - X2 - R) / D,
                        (Y2 - X2 + R) / D,  (Y2 + X2 - R) / D);
        telemetry.addData("yaw", Math.toDegrees(yaw));
    }

    private void Vacuum() {
        if (!gamepad1.square) {
            SqPressed = false;
            return;
        }
        if (SqPressed) { return; }
        SqPressed = true;
        if (!VisBusy) {
            V.setPower(1);
            VisBusy = true;
            return;
        }
        V.setPower(0);
        VisBusy = false;
    }

    private void PullUp() {
        double PUPow = 0;
        if (gamepad1.triangle) {
            PUPow = 0.1;
            PU.setPower(1);
            return;}
        if (gamepad1.cross) {
            PU.setPower(-0.1);
            return;}
        PU.setPower(PUPow);   //0.1
    }

//    private void DuoServo() {
//        double[]    pos    = {0, 0, 0};
//        double[]    speed  = {1e-2, 1e-2, 1e-2};
//        boolean[][] button = {{gamepad1.left_bumper, gamepad1.right_bumper}, {gamepad1.dpad_left, gamepad1.dpad_right},
//                              {gamepad1.dpad_up, gamepad1.dpad_down}};
//        Servo[][]   servos = {{LLL, LRL}, {LA, RA}, {LH, RH}};
//        for (int i = 0; i < 3; i++) {
//            pos[i] = button[i][0] ? + speed[i] :
//                     button[i][1] ? - speed[i] : pos[i];
//            Robot.SetDuoServoPos(pos[i], servos[i][0], servos[i][1]);
//        }
//    }

    private void RaiseLift() {
        double raiseSp = 0.01;
        liftAng = gamepad1.left_bumper  ? + raiseSp :
                  gamepad1.right_bumper ? - raiseSp : liftAng;
        Robot.SetDuoServoPos(liftAng, LLL, LRL);
    }

    private void Arm() {
        double armSp = 0.01;
        armAng = gamepad1.dpad_left  ? + armSp :
                 gamepad1.dpad_right ? - armSp : armAng;
        Robot.SetDuoServoPos(armAng, LA, RA);
    }

    private void Hoist() {
        double hoistSp = 0.01;
        hoistAng = gamepad1.dpad_up   ? + hoistSp :
                   gamepad1.dpad_down ? - hoistSp : hoistAng;
        Robot.SetDuoServoPos(hoistAng, LH, RH);
    }

//    private void Lift() {
//        CurrentPosition = Math.max(LL.getCurrentPosition(), Math.max(ML.getCurrentPosition(), RL.getCurrentPosition()));
//        double  LT     = gamepad1.left_trigger;
//        double  RT     = gamepad1.right_trigger;
//        boolean D_Up   = gamepad1.dpad_up;
//        boolean D_Down = gamepad1.dpad_down;
//        int[]   Levels = {-50, Robot.Ground_Junction, Robot.Low_Junction, Robot.Medium_Junction, Robot.High_Junction};
//        LT_press = LT >= 0.25 || LT_press;
//        RT_press = RT >= 0.25 || RT_press;
//        if (LT_press && (LT < 0.25)) {
//            LT_press = false;
//            Level--;
//        }
//        if (RT_press && (RT < 0.25)) {
//            RT_press = false;
//            Level++;
//        }
//        Level = D_Down || D_Up ? (CurrentPosition >= Robot.High_Junction   ? 4 :
//                            CurrentPosition >= Robot.Medium_Junction ? 3 :
//                            CurrentPosition >= Robot.Low_Junction    ? 2 :
//                            CurrentPosition >= Robot.Ground_Junction ? 1 :
//                            CurrentPosition >=                     0 ? 0 : Level) : Level;
//
//        Level = Range.clip(Level,0, 4);
//        boolean Auto_Condition = !(D_Up) && !(D_Down) && (Lift_isAuto || LT >= 0.25 || RT >= 0.25);
//        double Min_Power  = 0.05;
//        double Auto       = (AtTargetRange(CurrentPosition, Levels[Level], 10) ? Min_Power : (CurrentPosition < Levels[Level] ? 1 : -0.75));
////        double Control    = (D_Up ? (CurrentPosition >= Robot.Max_Lift ? 0 : 1)     : (D_Down ? (CurrentPosition <= 0 ? 0 : -1) : Min_Power));
//        double Control    = (D_Up ? 1 : (D_Down ? (CurrentPosition <= -50 ? 0 : -1) : Min_Power));
//        double Lift_Power = Auto_Condition ? Auto : Control;
//        Lift_isAuto = Auto_Condition;
//        Robot.LiftPower(Lift_Power);
//    }

    @Override
    public void runOpMode() {
        Init();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (gamepad1.ps){
                    imu.resetYaw();
                    setpoint = 0;
                }
                Movement();
                Vacuum();
                RaiseLift();
                Arm();
                Hoist();
                PullUp();
                telemetry.addData("setpoint",Math.toDegrees(setpoint));
                telemetry.addData("LL", LL.getCurrentPosition());
                telemetry.addData("RL", RL.getCurrentPosition());
                telemetry.addData("FL", FL.getCurrentPosition());
                telemetry.addData("FR", FR.getCurrentPosition());
                telemetry.addData("BL", BL.getCurrentPosition());
                telemetry.addData("BR", BR.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}

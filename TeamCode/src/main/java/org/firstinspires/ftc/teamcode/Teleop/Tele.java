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
    double setpoint = Math.toDegrees(0), PUPow = 0, liftAng = 0, armAng = 0, hoistAng = 0;
    boolean VPressed = false, VisBusy = false;

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
                         LLL, LRL, LA, RA, LH, RH, K, new double[]{liftAng, armAng, hoistAng});

        // Show FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    private void Movement() {
        double X1  =  gamepad1.left_stick_x;
        double Y1  = -gamepad1.left_stick_y;
        double yaw = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double X2  =  (Math.cos(yaw) * X1) - (Math.sin(yaw) * Y1);
        double Y2  =  (Math.sin(yaw) * X1) + (Math.cos(yaw) * Y1);
        // Rotate
        double R =  pid_R.Calculate(WrapRads(setpoint + yaw));
        double X = -gamepad1.right_stick_x;
        if (X != 0) {
            R = X;
            setpoint = -yaw;
        }
        // Denominator for division to get no more than 1
        double D = Math.max(Math.abs(X2) + Math.abs(Y2) + Math.abs(R), 1);
        Robot.MovePower((Y2 + X2 + R) / D, (Y2 - X2 - R) / D,
                        (Y2 - X2 + R) / D,  (Y2 + X2 - R) / D);
        telemetry.addData("yaw", Math.toDegrees(yaw));
    }

    private void Vacuum() {
        boolean sq = gamepad1.square;
        boolean ci = gamepad1.circle;
        if (!(sq || ci)) {
            VPressed = false;
            return;
        }
        if (VPressed) { return; }
        VPressed = true;
        if (!VisBusy) {
            double pow = sq ? 1 : -1;
            V.setPower(pow);
            VisBusy = true;
            return;
        }
        V.setPower(0);
        VisBusy = false;
    }

    private void PullUp() {
        if (gamepad1.triangle) {
            PUPow = 0.1;
            PU.setPower(1);
            return;}
        if (gamepad1.cross) {
            PU.setPower(-0.1);
            return;}
        PU.setPower(PUPow);   //0.1
    }

    private void RaiseLift() {
        double raiseSp = 0.02;
        liftAng = gamepad1.left_bumper  ? liftAng + raiseSp :
                  gamepad1.right_bumper ? liftAng - raiseSp : liftAng;
        liftAng = Robot.SetDuoServoPos(liftAng, new float[]{0, 0.96f}, LLL, LRL);
    }

    private void Arm() {
        double armSp = 0.02;
        armAng = gamepad1.dpad_left  ? armAng + armSp :
                 gamepad1.dpad_right ? armAng - armSp : armAng;
        armAng = Robot.SetDuoServoPos(armAng, null, LA, RA);
    }

    private void Hoist() {
        double hoistSp = 0.01;
        hoistAng = gamepad1.dpad_up   ? hoistAng + hoistSp :
                   gamepad1.dpad_down ? hoistAng - hoistSp : hoistAng;
        hoistAng = Robot.SetDuoServoPos(hoistAng, null, LH, RH);
    }

    private void Lift() {
        double LT = gamepad1.left_trigger;
        double RT = gamepad1.right_trigger;
        double Lift_Power = LT >= 0.25 ? LT : RT >= 0.25 ? -RT : 0.05;
        Robot.LiftPower(Lift_Power);
    }

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
                Lift();
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

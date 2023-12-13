package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utilize.AtTargetRange;
import static org.firstinspires.ftc.teamcode.Utilize.WrapRads;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name="Tele")
public class Tele extends Robot {
    // Variables
    int targetLift = 0;
    double setpoint = 0, H_Ang = 0, AL_Ang = 0, AD_Ang = 0;
    boolean autoLift = false, V_Pressed = false, VisBusy = false;

    private void Init() {
        // Initialize Robot
        Initialize(DcMotor.RunMode.RUN_WITHOUT_ENCODER, new double[]{0, 0, 0},
                                                        new double[]{0, 0, 0, 0});

        // Show FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    private void Movement() {
        double speed =  0.35;
        double lx    = -gamepad1.left_stick_x;
        double ly    =  gamepad1.left_stick_y;
        double x1    =  gamepad1.dpad_left ?  speed : gamepad1.dpad_right ? -speed : lx;
        double y1    =  gamepad1.dpad_up   ? -speed : gamepad1.dpad_down  ? speed : ly;
        double yaw   = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double x2    =  (Math.cos(yaw) * x1) - (Math.sin(yaw) * y1);
        double y2    =  (Math.sin(yaw) * x1) + (Math.cos(yaw) * y1);
        // Rotate
        double r = new Controller(1.6, 0.01, 0.01, 0).Calculate(WrapRads(setpoint - yaw));
        double x = gamepad1.right_stick_x;
        if (x != 0) {
            r = x;
            setpoint = yaw;
        }
        // Denominator for division to get no more than 1
        double d = Math.max(Math.abs(x2) + Math.abs(y2) + Math.abs(r), 1);
        MovePower((y2 + x2 + r) / d, (y2 - x2 - r) / d,
                  (y2 - x2 + r) / d, (y2 + x2 - r) / d);
        telemetry.addData("yaw", Math.toDegrees(yaw));
    }

    private void Rocket() { if (gamepad1.touchpad) SetServoPos(0.2, R); }

    private void Vacuum() {
        boolean lt = gamepad1.left_trigger  >= 0.25;
        boolean rt = gamepad1.right_trigger >= 0.25;
        if (!(lt || rt)) {
            V_Pressed = false;
            return;
        }
        if (V_Pressed) return;
        V_Pressed = true;
        if (!VisBusy) {
            double pow = rt ? 0.7 : -0.7;
            V.setPower(pow);
            VisBusy = true;
            return;
        }
        V.setPower(0);
        VisBusy = false;
    }

    private void Dropper() {
        if (gamepad2.right_stick_button) {
            SetServoPos(0, IT);
            SetServoPos(0.2, DP);
        }
    }

    private void PullUp() {
        double pullPow = gamepad2.dpad_up   ? -1   :
                         gamepad2.dpad_down ?  0.5 : 0;
        PU.setPower(pullPow);
    }

    private void Hoist() {
        double rs = gamepad2.right_stick_x;
        H_Ang     = rs >=  0.1 || rs <= -0.1 ? H_Ang + (rs / 100) : H_Ang;
        H_Ang     = SetServoPos(H_Ang, new float[]{0f, 0.75f}, LH, RH);
    }

    private void AdjustDrop() {
        double ls = gamepad2.left_stick_x;
        AD_Ang    = ls >=  0.1 || ls <= -0.1 ? AD_Ang + (ls / 100) : AD_Ang;
        AD_Ang    = SetServoPos(AD_Ang, ADP);
    }

    private void AdjustLift() {
        double AL_Sp = 0.02;
        AL_Ang = gamepad2.right_bumper ? AL_Ang + AL_Sp :
                 gamepad2.left_bumper  ? AL_Ang - AL_Sp : AL_Ang;
        AL_Ang = SetServoPos(AL_Ang, new float[]{0f, 0.34f}, ALL, ARL);
    }

    private void Lift() {
        double  curPos     = Math.max(LL.getCurrentPosition(), RL.getCurrentPosition());
        double  lt         = gamepad2.left_trigger;
        double  rl         = gamepad2.right_trigger;
        boolean lt_Pressed = lt >= 0.25;
        boolean rl_Pressed = rl >= 0.25;
        boolean x = gamepad2.x;
        boolean y = gamepad2.y;
        boolean b = gamepad2.b;
        boolean a = gamepad2.a;
        if (x || y || b || a) {
            if (!a) SetServoPos(0.3, IT);
            targetLift      = x      ? 700  :
                              y      ? 1600 :
                              b      ? 2030 : 0;
            double LARA_pos = a      ? 0.2  : 0.92;
            AD_Ang          = a      ? 0    : 0.38;
            AL_Ang          = x || a ? 0    : 0.2;
            SetServoPos(AL_Ang, ALL, ARL);
            SetServoPos(LARA_pos, LA, RA);
            SetServoPos(AD_Ang, ADP);
            if (a) SetServoPos(0, DP, IT);
            autoLift = true;
        }
        if (autoLift && targetLift == 0 && curPos <= 10){
            SetServoPos(0, LA, RA);
        }
        if ((autoLift && AtTargetRange(curPos, targetLift, 10)) || (lt_Pressed || rl_Pressed)) {
            autoLift = false;
        }
        double Lift_Power = lt_Pressed ? (curPos < 0          ?  0   : -lt) :
                            rl_Pressed ? (curPos > 3000       ?  0   :  rl) :
                            autoLift   ? (curPos > targetLift ? -0.2 :  1)  : 0;
        LiftPower(Lift_Power);
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
                PullUp();
                Rocket();
                Hoist();
                AdjustDrop();
                AdjustLift();
                Dropper();
                Lift();
                Vacuum();
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

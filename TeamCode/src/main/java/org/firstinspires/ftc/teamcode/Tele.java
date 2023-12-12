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
    int hoistLevel = 0, targetLift = 0;
    double setpoint = 0, PU_Pow = 0, armAng = 0, AD_Ang = 0, AL_Ang = 0;
    boolean V_Pressed = false, VisBusy = false, rb_Pressed = false, dl_Pressed = false, DisOn = false,
            dr_Pressed = false, ITisOn = false, autoLift = false;

    private void Init() {
        // Initialize Robot
        Initialize(DcMotor.RunMode.RUN_WITHOUT_ENCODER, new double[]{armAng, 0, AL_Ang},
                                                        new double[]{0, 0, AD_Ang, 0});

        // Show FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    private void Movement() {
        double speed =  0.5;
        double lx    = -gamepad1.left_stick_x;
        double ly    =  gamepad1.left_stick_y;

        double x1    =  gamepad1.dpad_up   ? -speed : gamepad1.dpad_down  ?  speed : lx;
        double y1    =  gamepad1.dpad_left ?  speed : gamepad1.dpad_right ? -speed : ly;
        double yaw   = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double x2    =  (Math.cos(yaw) * x1) - (Math.sin(yaw) * y1);
        double y2    =  (Math.sin(yaw) * x1) + (Math.cos(yaw) * y1);
        // Rotate
        double r = new Controller(1.6, 0.01, 0.09, 0).Calculate(WrapRads(setpoint - yaw));
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

    private void PullUp() {
        double LL = gamepad1.left_trigger;
        double RL = gamepad1.right_trigger;
        double pullPow = LL >= 0.25 ? -LL :
                         RL >= 0.25 ?  RL : PU_Pow;
        PU.setPower(pullPow);
    }

    private void Rocket() { if (gamepad1.triangle) SetServoPos(0.2, R); }

    private void Hoist() {
        switch (hoistLevel) {
            case 1:
                SetServoPos(0.5, LH, RH);
                break;
            case 2:
                SetServoPos(0.75, LH, RH);
                break;
            case 3:
                SetServoPos(0, LH, RH);
                PU_Pow = 0.1;
                break;
            default:
                break;
        }
        boolean rb = gamepad1.right_bumper;
        if (!rb) {
            rb_Pressed = false;
            return;
        }
        if (rb_Pressed) return;
        rb_Pressed = true;
        hoistLevel++;
    }

    private void Vacuum() {
        boolean sq = gamepad1.square;
        boolean ci = gamepad1.circle;
        if (!(sq || ci)) {
            V_Pressed = false;
            return;
        }
        if (V_Pressed) return;
        V_Pressed = true;
        if (!VisBusy) {
            double pow = sq ? 0.8 : -0.8;
            V.setPower(pow);
            VisBusy = true;
            return;
        }
        V.setPower(0);
        VisBusy = false;
    }

    private void Arm() {
        double armSp = 0.02;
        armAng = gamepad2.y ? armAng + armSp :
                 gamepad2.a ? armAng - armSp : armAng;
        armAng = SetServoPos(armAng, LA, RA);
    }

    private void AdjustLift() {
        double AL_Sp = 0.02;
        AL_Ang = gamepad2.right_bumper ? AL_Ang + AL_Sp :
                 gamepad2.left_bumper  ? AL_Ang - AL_Sp : AL_Ang;
        AL_Ang = SetServoPos(AL_Ang, new float[]{0f, 0.34f}, ALL, ARL);
    }

    private void AdjustDrop() {
        double AD_Sp = 0.05;
        AD_Ang = gamepad2.dpad_down ? AD_Ang + AD_Sp :
                 gamepad2.dpad_up   ? AD_Ang - AD_Sp : AD_Ang;
        AD_Ang = SetServoPos(AD_Ang, ADP);
    }

    private void Dropper() {
        boolean dl = gamepad2.b;
        if (!(dl)) {
            dl_Pressed = false;
            return;
        }
        if (dl_Pressed) return;
        dl_Pressed = true;
        if (!DisOn) {
            SetServoPos(0.2, DP);
            DisOn = true;
            return;
        }
        SetServoPos(0, DP);
        DisOn = false;
    }

    private void Intake() {
        boolean dr = gamepad2.x;
        if (!(dr)) {
            dr_Pressed = false;
            return;
        }
        if (dr_Pressed) return;
        dr_Pressed = true;
        if (!ITisOn) {
            SetServoPos(0.45, IT);
            ITisOn = true;
            return;
        }
        SetServoPos(0, IT);
        ITisOn = false;
    }

    private void Lift() {
        double  curPos     = Math.max(LL.getCurrentPosition(), RL.getCurrentPosition());
        double  LL         = gamepad2.left_trigger;
        double  RL         = gamepad2.right_trigger;
        boolean LL_Pressed = LL >= 0.25;
        boolean RL_Pressed = RL >= 0.25;
        if (curPos <= 100 && RL_Pressed) {
            SetServoPos(0.2, LA, RA);
        }
        if (gamepad2.start) {
            SetServoPos(0, ADP);
            SetServoPos(0.2, LA, RA);
            SetServoPos(0, IT, DP);
            autoLift = true;
            targetLift = 0;
        }
        if (autoLift && targetLift == 0 && curPos <= 100){
            SetServoPos(0, LA, RA);
        }
        if ((autoLift && AtTargetRange(curPos, targetLift, 10)) || (LL_Pressed || RL_Pressed)) {
            autoLift = false;
        }
        double Lift_Power = LL_Pressed ? (curPos < 0          ?  0   : -LL) :
                            RL_Pressed ? (curPos > 2550       ?  0   :  RL) :
                            autoLift   ? (curPos > targetLift ? -0.5 : 0.5) : 0;
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
                Arm();
                AdjustLift();
                AdjustDrop();
                Dropper();
                Intake();
                Lift();
                Vacuum();
                telemetry.addData("setpoint",Math.toDegrees(setpoint));
                telemetry.addData("LL", LL.getCurrentPosition());
                telemetry.addData("RL", RL.getCurrentPosition());
                telemetry.addData("armAng", armAng);
                telemetry.addData("AL_Ang", AL_Ang);
                telemetry.addData("AD_Ang", AD_Ang);
                telemetry.update();
            }
        }
    }
}

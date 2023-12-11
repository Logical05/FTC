package org.firstinspires.ftc.teamcode;

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
    /** Variables */
    double setpoint = Math.toDegrees(0.1), PUPow = 0.1, armAng = 0.1, hoistAng = 0.1, keepAng = 0.1, keepArmAng = 0.1, rocketAng = 0.1;
    boolean VPressed = false, VisBusy = false;

    private void Init() {
        // Initialize Robot
        Initialize(DcMotor.RunMode.RUN_WITHOUT_ENCODER, new double[]{0, 0},
                                                        new double[]{0, 0, 0, 0});

        // Show FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    private void Movement() {
        double x1  =  gamepad1.left_stick_x;
        double y1  = -gamepad1.left_stick_y;
        double yaw = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double x2  =  (Math.cos(yaw) * x1) - (Math.sin(yaw) * y1);
        double y2  =  (Math.sin(yaw) * x1) + (Math.cos(yaw) * y1);
        // Rotate
        double r =  new Controller(1.6, 0.01, 0.09, 0).Calculate(WrapRads(setpoint - yaw));
        double x = -gamepad1.right_stick_x;
        if (x != 0) {
            r = x;
            setpoint = yaw;
        }
        // Denominator for division to get no more than 1
        double d = Math.max(Math.abs(x2) + Math.abs(y2) + Math.abs(r), 1);
        MovePower((y2 + x2 + r) / d, (y2 - x2 - r) / d,
                        (y2 - x2 + r) / d,  (y2 + x2 - r) / d);
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
            double pow = sq ? 0.8: -1;
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
        PU.setPower(PUPow);
    }

    private void Arm() {
        double armSp = 0.02;
        armAng = gamepad1.dpad_left  ? armAng + armSp :
                 gamepad1.dpad_right ? armAng - armSp : armAng;
        armAng = SetDuoServoPos(armAng, LA, RA);
    }

    private void Hoist() {
        double hoistSp = 0.01;
        hoistAng = gamepad1.dpad_up   ? hoistAng + hoistSp :
                   gamepad1.dpad_down ? hoistAng - hoistSp : hoistAng;
        hoistAng = SetDuoServoPos(hoistAng, LH, RH);
    }

    private void Rocket(){
        double rocketSp = 0.01;
        rocketAng = gamepad2.square  ? rocketAng + rocketSp :
                  gamepad2.circle  ? rocketAng - rocketSp : rocketAng;
        rocketAng = SetServoPos(rocketAng, R);
    }

    private void Lift() {
        double LL = gamepad1.left_trigger;
        double RL = gamepad1.right_trigger;
        double Lift_Power = LL >= 0.25 ? LL : RL >= 0.25 ? -RL : 0.05;
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
                Vacuum();     
                Lift();
                Arm();
                Hoist();
                PullUp();
                Rocket();
                telemetry.addData("setpoint",Math.toDegrees(setpoint));
                telemetry.addData("LL", LL.getCurrentPosition());
                telemetry.addData("RL", RL.getCurrentPosition());
                telemetry.addData("FL", FL.getCurrentPosition());
                telemetry.addData("FR", FR.getCurrentPosition());
                telemetry.addData("BL", BL.getCurrentPosition());
                telemetry.addData("BR", BR.getCurrentPosition());
                telemetry.addData("keep", keepAng);
                telemetry.addData("keep_Arm", keepArmAng);
                telemetry.addData("Rocket", rocketAng);
                telemetry.update();
            }
        }
    }
}

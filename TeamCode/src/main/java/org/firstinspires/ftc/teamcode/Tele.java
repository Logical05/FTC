package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static org.firstinspires.ftc.teamcode.Utilize.*;

@Config
@TeleOp(name="Tele")
public class Tele extends LinearOpMode {
    /** Usage External Class */
    Controller pid_R = new Controller(1.6, 0.01, 0.09, 0);

    /** Hardware */
    IMU imu;
    Servo LLL, LRL, LA, RA, LH, RH, K, KA, R;
    DcMotorEx FL, FR, BL, BR, LL, RL, PU, V;

    /** Variables */
    double setpoint = Math.toDegrees(0.1), PUPow = 0.1, liftAng = 1, armAng = 0.1, hoistAng = 0.1, keepAng = 0.1, keepArmAng = 0.1, rocketAng = 0.1;
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
        K   = hardwareMap.get(Servo.class,     "Keep");         KA  = hardwareMap.get(Servo.class,     "Keep_Arm");
        R   = hardwareMap.get(Servo.class,     "Rocket");

        // Initialize Robot
        Robot.Initialize(imu, DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                         FL, FR, BL, BR, LL, RL, PU, V,
                         LLL, LRL, LA, RA, LH, RH, K, KA, R,
                         new double[]{liftAng, armAng, hoistAng},
                         new double[]{keepAng, keepArmAng, rocketAng});

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
        double r =  pid_R.Calculate(WrapRads(setpoint - yaw));
        double x = -gamepad1.right_stick_x;
        if (x != 0) {
            r = x;
            setpoint = yaw;
        }
        // Denominator for division to get no more than 1
        double d = Math.max(Math.abs(x2) + Math.abs(y2) + Math.abs(r), 1);
        Robot.MovePower((y2 + x2 + r) / d, (y2 - x2 - r) / d,
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

    private void RaiseLift() {
        double raiseSp = 0.02;
        liftAng = gamepad1.left_bumper  ? liftAng + raiseSp :
                  gamepad1.right_bumper ? liftAng - raiseSp : liftAng;
        liftAng = Robot.SetDuoServoPos(liftAng, new float[]{1f, 0.96f}, LLL, LRL);
    }

    private void Arm() {
        double armSp = 0.02;
        armAng = gamepad1.dpad_left  ? armAng + armSp :
                 gamepad1.dpad_right ? armAng - armSp : armAng;
        armAng = Robot.SetDuoServoPos(armAng, LA, RA);
    }

    private void Hoist() {
        double hoistSp = 0.01;
        hoistAng = gamepad1.dpad_up   ? hoistAng + hoistSp :
                   gamepad1.dpad_down ? hoistAng - hoistSp : hoistAng;
        hoistAng = Robot.SetDuoServoPos(hoistAng, LH, RH);
    }

    private void Keep(){
        double keepSp = 0.01;
        keepAng = gamepad2.dpad_up   ? keepAng + keepSp :
                  gamepad2.dpad_down ? keepAng - keepSp : keepAng;
        keepAng = Robot.SetServoPos(keepAng, K);
    }

    private void Keep_Arm(){
        double keepArmSp = 0.01;
        keepArmAng = gamepad2.dpad_left   ? keepArmAng + keepArmSp :
                  gamepad2.dpad_right  ? keepArmAng - keepArmSp : keepArmAng;
        keepArmAng = Robot.SetServoPos(keepArmAng, KA);

    }

    private void Rocket(){
        double rocketSp = 0.01;
        rocketAng = gamepad2.square  ? rocketAng + rocketSp :
                  gamepad2.circle  ? rocketAng - rocketSp : rocketAng;
        rocketAng = Robot.SetServoPos(rocketAng, R);
    }

    private void Lift() {
        double LL = gamepad1.left_trigger;
        double RL = gamepad1.right_trigger;
        double Lift_Power = LL >= 0.25 ? LL : RL >= 0.25 ? -RL : 0.05;
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
                Keep();
                Keep_Arm();
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

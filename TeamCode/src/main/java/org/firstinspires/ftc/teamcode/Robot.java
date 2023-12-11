package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utilize.AtTargetRange;
import static org.firstinspires.ftc.teamcode.Utilize.SigNum;
import static org.firstinspires.ftc.teamcode.Utilize.WrapRads;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public abstract class Robot extends LinearOpMode {
    IMU Imu;
    Servo LLL, LRL, LA, RA, LH, RH, K, KA, R;
    DcMotorEx FL, FR, BL, BR, LL, RL, PU, V;
    public int FL_Target, FR_Target, BL_Target, BR_Target;
    public final double[] tileSize            = {23.5, 23.5};  // Width * Length
    /** TETRIX Motor Encoder per revolution */
    public final int      Counts_per_TETRIX   = 24;
    /** HD HEX Motor Encoder per revolution */
    public final int      Counts_per_HD_HEX   = 28;
    /** 20:1 HD HEX Motor Encoder per revolution */
    public final int      Gear_20_HD_HEX      = Counts_per_HD_HEX * 20;
    /** (3 * 4 * 5):1 UltraPlanetary HD HEX Motor Encoder per revolution */
    public final double   Gear_60_HD_HEX      = Counts_per_HD_HEX * 54.8;
    public final double   Wheel_Diameter_Inch = 3;
    public final double   Counts_per_Inch     = Gear_20_HD_HEX /
                                                     (Wheel_Diameter_Inch * Math.PI);
    public double         heading             = 0;
    public double[]       currentXY           = {0, 0};

    public void LiftPower(double liftPower) {
        LL.setPower(liftPower);
        RL.setPower(liftPower);
    }

    public void MovePower(double Front_Left, double Front_Right,
                                 double Back_Left,  double Back_Right) {
        FL.setPower(Front_Left);
        FR.setPower(Front_Right);
        BL.setPower(Back_Left);
        BR.setPower(Back_Right);
    }

    public void MoveMode(DcMotor.RunMode moveMode) {
        FL.setMode(moveMode);
        FR.setMode(moveMode);
        BL.setMode(moveMode);
        BR.setMode(moveMode);
    }

    public boolean MoveisBusy() {
        return FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy();
    }

    public void MoveTargetPosition(double FL_Inches, double FR_Inches,
                                          double BL_Inches, double BR_Inches) {
        FL_Target = FL.getCurrentPosition() + ((int) (FL_Inches * Counts_per_Inch));
        FR_Target = FR.getCurrentPosition() + ((int) (FR_Inches * Counts_per_Inch));
        BL_Target = BL.getCurrentPosition() + ((int) (BL_Inches * Counts_per_Inch));
        BR_Target = BR.getCurrentPosition() + ((int) (BR_Inches * Counts_per_Inch));
//        FL.setTargetPositionTolerance(2);
//        FR.setTargetPositionTolerance(2);
//        BL.setTargetPositionTolerance(2);
//        BR.setTargetPositionTolerance(2);
        FL.setTargetPosition(FL_Target);
        FR.setTargetPosition(FR_Target);
        BL.setTargetPosition(BL_Target);
        BR.setTargetPosition(BR_Target);
    }

    public double SetDuoServoPos(double pos, float[] minMax, Servo L_servo, Servo R_servo) {
        pos = Range.clip(pos, minMax[0], minMax[1]);
        L_servo.setPosition(pos);
        R_servo.setPosition(pos);
        return pos;
    }

    public double SetDuoServoPos(double pos, Servo L_servo, Servo R_servo) {
        pos = Range.clip(pos, 0, 1);
        L_servo.setPosition(pos);
        R_servo.setPosition(pos);
        return pos;
    }

    public double SetServoPos(double pos, float[] minMax, Servo servo){
        pos = Range.clip(pos, minMax[0], minMax[1]);
        servo.setPosition(pos);
        return pos;
    }

    public double SetServoPos(double pos, Servo servo){
        pos = Range.clip(pos, 0, 1);
        servo.setPosition(pos);
        return pos;
    }

    public void Break(double stopSecond) {
        if (stopSecond == 0) return;
        MovePower(0, 0, 0, 0);
        MoveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep((long) (stopSecond * 1000));
    }

    public void Move(double power, double kp, double ki, double targetX, double targetY,
                      double stopSecond, double timeOut) {
        double x        = targetX - currentXY[0];
        double y        = targetY - currentXY[1];
        double absX     = Math.abs(x);
        double absY     = Math.abs(y);
        double powerX   = absX > absY ? power : (absX / absY) * power;
        double powerY   = absX < absY ? power : (absY / absX) * power;
        double x1       = powerX * SigNum(x);
        double y1       = powerY * SigNum(y);
        double inchesX1 = tileSize[0] * x;
        double inchesY1 = tileSize[1] * y;
        double inchesX2 = (Math.cos(heading) * inchesX1) - (Math.sin(heading) * inchesY1);
        double inchesY2 = (Math.sin(heading) * inchesX1) + (Math.cos(heading) * inchesY1);
        double FL_BR_In = inchesY2 + inchesX2;
        double FR_BL_In = inchesY2 - inchesX2;

        PIDCoefficients pid = new PIDCoefficients(kp, ki, 0);
        FL.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);
        FR.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);
        BL.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);
        BR.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);
        MoveTargetPosition(FL_BR_In, FR_BL_In, FR_BL_In, FL_BR_In);
        MoveMode(DcMotor.RunMode.RUN_TO_POSITION);

        Controller  pid_R   = new Controller(1.6, 0.01, 0.09, 0);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (opModeIsActive()) {
            double yaw = -Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double x2 = (Math.cos(heading) * x1) - (Math.sin(heading) * y1);
            double y2 = (Math.sin(heading) * x1) + (Math.cos(heading) * y1);
            // Rotate
            double r = pid_R.Calculate(WrapRads(heading - yaw));
            // Denominator for division to get no more than 1
            double d = Math.max(Math.abs(x2) + Math.abs(y2) + Math.abs(r), 1);
            MovePower((y2 + x2 + r) / d, (y2 - x2 - r) / d,
                             (y2 - x2 + r) / d,  (y2 + x2 - r) / d);

            if (((runtime.seconds() >= timeOut) && (timeOut != 0)) || !MoveisBusy()) break;
        }
        Break(stopSecond);
        currentXY = new double[] {targetX, targetY};
    }

    public void Turn(double degs, double stopSecond) {
        double rads = Math.toRadians(degs);
        while (opModeIsActive()) {
            double yaw   = -Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double error = WrapRads(rads - yaw);
            double r     = AtTargetRange(error, 0, Math.toRadians(30)) ? 0.2 : 0.6;
            if (error < 0) r = -r;
            MovePower(r, -r, r,-r);

            if (AtTargetRange(error, 0, 0.05)) break;
        }
        Break(stopSecond);
        heading = rads;
    }

    public void Initialize(IMU imu, DcMotor.RunMode moveMode,
                           DcMotorEx motor1, DcMotorEx motor2, DcMotorEx motor3, DcMotorEx motor4,
                           DcMotorEx motor5, DcMotorEx motor6, DcMotorEx motor7, DcMotorEx motor8,
                           Servo servo1, Servo servo2, Servo servo3, Servo servo4, Servo servo5,
                           Servo servo6, Servo servo7, Servo servo8, Servo servo9,
                           double[] DuoServoAng, double[] ServoAng) {
        // Add Variable
        Imu = imu;
        FL  = motor1; FR  = motor2; BL  = motor3; BR  = motor4;
        LL  = motor5; RL  = motor6; PU  = motor7; V   = motor8;
        LLL = servo1; LRL = servo2; LA  = servo3; RA  = servo4;
        LH  = servo5; RH  = servo6; K   = servo7; KA  = servo8;
        R   = servo9;

        // Initialize IMU
        Imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

        // Reverse Servo
        LRL.setDirection(Servo.Direction.REVERSE);
        LA .setDirection(Servo.Direction.REVERSE);
        LH .setDirection(Servo.Direction.REVERSE);
        K  .setDirection(Servo.Direction.REVERSE);
        KA .setDirection(Servo.Direction.REVERSE);
        R  .setDirection(Servo.Direction.REVERSE);
        // Set Servo Position
        SetDuoServoPos(DuoServoAng[0], LLL, LRL);
        SetDuoServoPos(DuoServoAng[1], LA,  RA);
        SetDuoServoPos(DuoServoAng[2], LH,  RH);
        SetServoPos(ServoAng[0], K);
        SetServoPos(ServoAng[1], KA);
        SetServoPos(ServoAng[2], R);
        // Reverse Motors
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        LL.setDirection(DcMotorSimple.Direction.REVERSE);
        PU.setDirection(DcMotorSimple.Direction.REVERSE);
        // setMode Motors
        MoveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MoveMode(moveMode);
        LL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PU.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        V .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // SetBehavior Motors
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor5.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor6.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor7.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor8.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // SetPower Motors
        MovePower(0, 0, 0, 0);
        motor5.setPower(0);
        motor6.setPower(0);
        motor7.setPower(0);
        motor8.setPower(0);
    }
}
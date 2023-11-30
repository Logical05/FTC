package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    static Servo LLL, LRL, LA, RA, LH, RH, K;
    static DcMotorEx FL, FR, BL, BR, LL, RL, PU, V;
    public static int FL_Target, FR_Target, BL_Target, BR_Target;
    /** TETRIX Motor Encoder per revolution */
    public static final int    Counts_per_TETRIX   = 24;
    /** HD HEX Motor Encoder per revolution */
    public static final int    Counts_per_HD_HEX   = 28;
    /** 20:1 HD HEX Motor Encoder per revolution */
    public static final int    Gear_20_HD_HEX      = Counts_per_HD_HEX * 20;
    /** (3 * 4 * 5):1 UltraPlanetary HD HEX Motor Encoder per revolution */
    public static final double Gear_60_HD_HEX      = Counts_per_HD_HEX * 54.8;
    public static final double Wheel_Diameter_Inch = 3;
    public static final double Counts_per_Inch     = Gear_20_HD_HEX /
                                                     (Wheel_Diameter_Inch * Math.PI);

    public static void LiftPower(double liftPower) {
        LL.setPower(liftPower);
        RL.setPower(liftPower);
    }

    public static void MovePower(double Front_Left, double Front_Right,
                          double Back_Left,  double Back_Right) {
        FL.setPower(Front_Left);
        FR.setPower(Front_Right);
        BL.setPower(Back_Left);
        BR.setPower(Back_Right);
    }

    public static void MoveMode(DcMotor.RunMode moveMode) {
        FL.setMode(moveMode);
        FR.setMode(moveMode);
        BL.setMode(moveMode);
        BR.setMode(moveMode);
    }

    public static boolean MoveisBusy() {
        return FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy();
    }

    public static void MoveTargetPosition(double FL_Inches, double FR_Inches,
                                   double BL_Inches,  double BR_Inches) {
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

    /**
     * Default Range : float[] minMax = null
     * <p>
     * Custom Range : float[] minMax = Your Range
     */
    public static double SetDuoServoPos(double pos, float[] minMax, Servo L_servo, Servo R_servo) {
        pos = minMax == null ? Range.clip(pos, 0, 1) :
                               Range.clip(pos, minMax[0], minMax[1]);
        L_servo.setPosition(pos);
        R_servo.setPosition(pos);
        return pos;
    }

    public static void Initialize(IMU imu, DcMotor.RunMode moveMode,
                           DcMotorEx motor1, DcMotorEx motor2, DcMotorEx motor3, DcMotorEx motor4,
                           DcMotorEx motor5, DcMotorEx motor6, DcMotorEx motor7, DcMotorEx motor8,
                           Servo servo1, Servo servo2, Servo servo3, Servo servo4, Servo servo5,
                           Servo servo6, Servo servo7, double[] DuoServoAng) {
        // Add Variable
        FL  = motor1; FR  = motor2; BL  = motor3; BR  = motor4;
        LL  = motor5; RL  = motor6; PU  = motor7; V   = motor8;
        LLL = servo1; LRL = servo2; LA  = servo3; RA  = servo4;
        LH  = servo5; RH  = servo6; K   = servo7;

        // Initialize IMU
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

        // Reverse Servo
        LLL.setDirection(Servo.Direction.REVERSE);
        LA .setDirection(Servo.Direction.REVERSE);
        LH .setDirection(Servo.Direction.REVERSE);
        // Set Servo Position
        SetDuoServoPos(DuoServoAng[0], null, LLL, LRL);
        SetDuoServoPos(DuoServoAng[1], null, LA,  RA);
        SetDuoServoPos(DuoServoAng[2], null, LH,  RH);

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
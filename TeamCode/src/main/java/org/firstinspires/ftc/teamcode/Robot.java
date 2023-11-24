package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    public static IMU IMU;
    public static Servo LA, RA, K;
    public static DcMotorEx FL, FR, BL, BR, B, LL, ML, RL;
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

    public static void setArmPosition(double armPos) {
        LA.setPosition(armPos);
        RA.setPosition(armPos);
    }

    public static void Initialize(IMU imu, DcMotor.RunMode moveMode,
                           DcMotorEx Front_Left, DcMotorEx Front_Right,
                           DcMotorEx Back_Left,  DcMotorEx Back_Right,  DcMotorEx Base,
                           DcMotorEx Left_Lift,  DcMotorEx Middle_Lift, DcMotorEx Right_Lift,
                           double armPos, Servo Left_Arm, Servo Right_Arm,
                           double keeperPos, Servo Keeper) {
        // Add Variable
        IMU = imu;
        FL  = Front_Left;
        FR  = Front_Right;
        BL  = Back_Left;
        BR  = Back_Right;
        B   = Base;
        LL  = Left_Lift;
        ML  = Middle_Lift;
        RL  = Right_Lift;
        LA  = Left_Arm;
        RA  = Right_Arm;
        K   = Keeper;

        // Initialize IMU
        IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

        // Reverse Servo
        RA.setDirection(Servo.Direction.REVERSE);
        // Set Servo Position
        setArmPosition(armPos);
        K .setPosition(keeperPos);

        // Reverse Motors
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        B .setDirection(DcMotorSimple.Direction.REVERSE);
        RL.setDirection(DcMotorSimple.Direction.REVERSE);
        // setMode Motors
        MoveMode  (DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MoveMode  (moveMode);
        B .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ML.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // SetBehavior Motors
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        B .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ML.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // SetPower Motors
        MovePower(0, 0, 0, 0);
        B .setPower(0);
        LL.setPower(0);
        ML.setPower(0);
        RL.setPower(0);
    }
}
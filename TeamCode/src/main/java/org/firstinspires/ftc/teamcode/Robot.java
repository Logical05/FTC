package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utilize.atTargetRange;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    /** Hardware */
    public IMU IMU;
    public Servo LA, RA, K;
    public DcMotorEx FL, FR, BL, BR, B, LL, ML, RL;

    /** Motor Variables */
    public final int    Counts_per_TETRIX     = 24;  // TETRIX Motor Encoder per revolution
    public final int    Counts_per_HD_HEX     = 28;  // HD HEX Motor Encoder per revolution
    public final double Gear_60_HD_HEX        = Counts_per_HD_HEX * 54.8;  // (3 * 4 * 5):1 UltraPlanetary HD HEX Motor Encoder per revolution
    public final int    Gear_20_HD_HEX        = Counts_per_HD_HEX * 20;  // 20:1 HD HEX Motor Encoder per revolution
    public final double Wheel_Diameter_Inches = 4;
    public final double Counts_per_Inch       = Gear_20_HD_HEX / (Wheel_Diameter_Inches * Math.PI);

    /** Variables */
    public final int    High_Junction           = 615;
    public final int    Medium_Junction         = 445;
    public final int    Low_Junction            = 240;
    public final int    Ground_Junction         = 35;
    public final int    Max_Lift                = 640;
    public int FL_Target, FR_Target, BL_Target, BR_Target;
    public int Error_FL=0, Error_FR=0, Error_BL=0, Error_BR=0;

    public void LiftPower(double Lift_Power) {
        LL.setPower(Lift_Power);
        ML.setPower(Lift_Power);
        RL.setPower(Lift_Power);
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
                                   double BL_Inches,  double BR_Inches) {
        FL_Target = FL.getCurrentPosition() + ((int) (FL_Inches * Counts_per_Inch));
        FR_Target = FR.getCurrentPosition() + ((int) (FR_Inches * Counts_per_Inch));
        BL_Target = BL.getCurrentPosition() + ((int) (BL_Inches * Counts_per_Inch));
        BR_Target = BR.getCurrentPosition() + ((int) (BR_Inches * Counts_per_Inch));
//        FL.setTargetPositionTolerance(2);
//        FR.setTargetPositionTolerance(2);
//        BL.setTargetPositionTolerance(2);
//        BR.setTargetPositionTolerance(2);
        FL.setTargetPosition(FL_Target + Error_FL);
        FR.setTargetPosition(FR_Target + Error_FR);
        BL.setTargetPosition(BL_Target + Error_BL);
        BR.setTargetPosition(BR_Target + Error_BR);
    }

    public void setArmPosition(double armPos) {
        LA.setPosition(armPos);
        RA.setPosition(armPos);
    }

    public boolean Turn_Base(int Base_angle, double power, int range) {
        int Counts = (int) (Base_angle * Gear_60_HD_HEX) / 360;
        B.setTargetPosition(Counts);
        B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        B.setPower(power);
        return atTargetRange(B.getCurrentPosition(), Counts, range);
    }

    public void Initialize(IMU imu, DcMotor.RunMode moveMode,
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
        IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                                                                       RevHubOrientationOnRobot.UsbFacingDirection.UP)));

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

//        PIDCoefficients Basepid = new PIDCoefficients(5, 0.5, 0);
//        B.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, Basepid);

        // Reverse Servo
        RA.setDirection(Servo.Direction.REVERSE);
        // Set Servo Position
        setArmPosition(armPos);
        K .setPosition(keeperPos);
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    /** Usage External Class */
    Controller Base_pid = new Controller(0, 0, 0);

    /** Hardware */
    public IMU IMU;
    public Servo LA, RA, K;
    public DcMotor FL, FR, BL, BR, B, LL, RL;

    /** Motor Variables */
    public final int    Counts_per_TETRIX       = 24 * 60;  // 60:1 TETRIX Motor Encoder per revolution
    public final int    Counts_per_HD_HEX       = 28 * 20;  // 20:1 HD HEX Motor Encoder per revolution
    public final double Wheel_Diameter_Inches   = 4;
    public final double Counts_per_Inch         = Counts_per_HD_HEX / (Wheel_Diameter_Inches * Math.PI);

    /** Variables */
    public final int    Max_Lift                = 875;
    public final int    High_Junction           = 855;
    public final int    Medium_Junction         = 615;
    public final int    Low_Junction            = 370;
    public final int    Ground_Junction         = 75;

    public void MovePower(double Front_Left, double Front_Right,
                          double Back_Left,  double Back_Right) {
        FL.setPower(Front_Left);
        FR.setPower(Front_Right);
        BL.setPower(Back_Left);
        BR.setPower(Back_Right);
    }

    public void MoveMode(DcMotor.RunMode MoveMode) {
        FL.setMode(MoveMode);
        FR.setMode(MoveMode);
        BL.setMode(MoveMode);
        BR.setMode(MoveMode);
    }

    public void MoveTargetPosition(double Inches) {
        int Counts = (int)(Inches * Counts_per_Inch);
        FL.setTargetPosition(FL.getCurrentPosition( ) + Counts);
        FR.setTargetPosition(FR.getCurrentPosition() + Counts);
        BL.setTargetPosition(BL.getCurrentPosition() + Counts);
        BR.setTargetPosition(BR.getCurrentPosition() + Counts);
    }

    public boolean Turn_Base(int angle, double kP, double kI, double kD) {
        Base_pid.setPID(kP, kI, kD);
        int Counts = (angle * Counts_per_TETRIX) / 360;
        double Power = Base_pid.Calculate(Counts - B.getCurrentPosition());
        B.setPower(Power);
        return !Base_pid.atSetpoint();
    }

    public void Initialize(IMU imu, DcMotor.RunMode MoveMode,
                           DcMotor Front_Left, DcMotor Front_Right,
                           DcMotor Back_Left,  DcMotor Back_Right, DcMotor Base,
                           DcMotor Left_Lift,  DcMotor Right_Lift,
                           double Arm_pos, Servo Left_Arm, Servo Right_Arm,
                           double Keeper_pos, Servo Keeper) {
        // Add Variable
        IMU = imu;
        FL  = Front_Left;
        FR  = Front_Right;
        BL  = Back_Left;
        BR  = Back_Right;
        B   = Base;
        LL  = Left_Lift;
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
        RL.setDirection(DcMotorSimple.Direction.REVERSE);
        // setMode Motors
        MoveMode  (DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        B .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MoveMode  (MoveMode);
        B .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // SetBehavior Motors
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        B .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // SetPower Motors
        MovePower(0, 0, 0, 0);
        B .setPower(0);
        LL.setPower(0);
        RL.setPower(0);

        // Reverse Servo
        RA.setDirection(Servo.Direction.REVERSE);
        // Set Servo Position
        LA.setPosition(Arm_pos);
        RA.setPosition(Arm_pos);
        K .setPosition(Keeper_pos);
    }
}

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utilize.AtTargetRange;
import static org.firstinspires.ftc.teamcode.Utilize.WrapRads;
import static org.firstinspires.ftc.teamcode.Utilize.XY2Base;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

public abstract class Robot extends LinearOpMode {
    public IMU imu;
    public TfodProcessor tfod;
    public VisionPortal visionPortal;
    public ColorSensor colors;
    public Servo LA, RA, LH, RH, ALL, ARL, IT, DP, ADP, R;
    public DcMotorEx FL, FR, BL, BR, LL, RL, PU, V;
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
    public final double   Counts_per_Inch     = Gear_20_HD_HEX / (Wheel_Diameter_Inch * Math.PI);
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

    public double SetServoPos(double pos, float[] minMax, Servo L_servo, Servo R_servo) {
        pos = Range.clip(pos, minMax[0], minMax[1]);
        L_servo.setPosition(pos);
        R_servo.setPosition(pos);
        return pos;
    }

    public double SetServoPos(double pos, Servo L_servo, Servo R_servo) {
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

    private double[] CalInchesDirection(double x, double y) {
        double inchesX =  tileSize[0] * x;
        double inchesY =  tileSize[1] * y;
        double yaw     = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return new double[]{(Math.cos(yaw) * inchesX) - (Math.sin(yaw) * inchesY),
                            (Math.sin(yaw) * inchesX) + (Math.cos(yaw) * inchesY)};
    }

    public void Move(double power, double kp, double ki, double tileX, double tileY,
                     double stopSecond, double timeOut) {
        double   x        = tileX - currentXY[0];
        double   y        = tileY - currentXY[1];
        double[] xy1      = XY2Base(power, x, y);
        double[] inchesXY = CalInchesDirection(x, y);
        double   FL_BR_In = inchesXY[1] + inchesXY[0];
        double   FR_BL_In = inchesXY[1] - inchesXY[0];

        PIDCoefficients pid = new PIDCoefficients(kp, ki, 0);
        FL.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);
        FR.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);
        BL.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);
        BR.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);
        MoveTargetPosition(FL_BR_In, FR_BL_In, FR_BL_In, FL_BR_In);
        MoveMode(DcMotor.RunMode.RUN_TO_POSITION);

        Controller  pidR    = new Controller(1.6, 0.01, 0.09, 0);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (opModeIsActive()) {
            double yaw = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double x2  = (Math.cos(yaw) * xy1[0]) - (Math.sin(yaw) * xy1[1]);
            double y2  = (Math.sin(yaw) * xy1[0]) + (Math.cos(yaw) * xy1[1]);
            // Rotate
            double r = pidR.Calculate(WrapRads(heading - yaw));
            // Denominator for division to get no more than 1
            double d = Math.max(Math.abs(x2) + Math.abs(y2) + Math.abs(r), 1);
            MovePower((y2 + x2 + r) / d, (y2 - x2 - r) / d,
                      (y2 - x2 + r) / d, (y2 + x2 - r) / d);

            if (((runtime.seconds() >= timeOut) && (timeOut != 0)) || !MoveisBusy()) break;
        }
        Break(stopSecond);
        currentXY = new double[]{tileX, tileY};
    }

    public void MoveColor(double powerX, double powerY, double stopSecond, double timeOut) {
        Controller  pidR    = new Controller(1.6, 0.01, 0.09, 0);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (opModeIsActive()) {
            double yaw = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double x2  = (Math.cos(yaw) * powerX) - (Math.sin(yaw) * powerY);
            double y2  = (Math.sin(yaw) * powerX) + (Math.cos(yaw) * powerY);
            // Rotate
            double r = pidR.Calculate(WrapRads(heading - yaw));
            // Denominator for division to get no more than 1
            double d = Math.max(Math.abs(x2) + Math.abs(y2) + Math.abs(r), 1);
            MovePower((y2 + x2 + r) / d, (y2 - x2 - r) / d,
                      (y2 - x2 + r) / d, (y2 + x2 - r) / d);

            if (((runtime.seconds() >= timeOut) && (timeOut != 0)) || !MoveisBusy() ||
                ((colors.blue() >= 6000 || colors.red() >= 5500) && colors.green() <= 3500)) break;
        }
        Break(stopSecond);
        currentXY = new double[]{0, 0};
    }

    public void Turn(double degs, double stopSecond) {
        double rads = Math.toRadians(degs);
        while (opModeIsActive()) {
            double yaw   = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double error =  WrapRads(rads - yaw);
            double r     =  AtTargetRange(error, 0, Math.toRadians(30)) ? 0.2 : 0.6;
            if (error < 0) r = -r;
            MovePower(r, -r, r,-r);

            if (AtTargetRange(error, 0, 0.05)) break;
        }
        Break(stopSecond);
        heading = rads;
    }

    public void TfodInit(String deviceName, String TFOD_MODEL_FILE, String[] LABELS, float minResultConfidence) {
        // Initialize TensorFlow Object Detection (TFOD)
        tfod = new TfodProcessor.Builder()
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                .build();
        tfod.setMinResultConfidence(minResultConfidence);
        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, deviceName));
        builder.addProcessor(tfod);
        visionPortal = builder.build();
    }

    public void Initialize(DcMotor.RunMode moveMode, double[] DuoServoAng, double[] ServoAng) {
        imu = hardwareMap.get(IMU.class,       "imu");
        FL  = hardwareMap.get(DcMotorEx.class, "Front_Left");    FR  = hardwareMap.get(DcMotorEx.class, "Front_Right");
        BL  = hardwareMap.get(DcMotorEx.class, "Back_Left");     BR  = hardwareMap.get(DcMotorEx.class, "Back_Right");
        LL  = hardwareMap.get(DcMotorEx.class, "Left_Lift");     RL  = hardwareMap.get(DcMotorEx.class, "Right_Lift");
        PU  = hardwareMap.get(DcMotorEx.class, "Pull_Up");       V   = hardwareMap.get(DcMotorEx.class, "Vacuum");
        LA  = hardwareMap.get(Servo.class,     "Left_Arm");      RA  = hardwareMap.get(Servo.class,     "Right_Arm");
        LH  = hardwareMap.get(Servo.class,     "Left_Hoist");    RH  = hardwareMap.get(Servo.class,     "Right_Hoist");
        ALL = hardwareMap.get(Servo.class,     "Adjust_LL");     ARL = hardwareMap.get(Servo.class,     "Adjust_RL");
        IT  = hardwareMap.get(Servo.class,     "InTake");        DP  = hardwareMap.get(Servo.class,     "Dropper");
        ADP = hardwareMap.get(Servo.class,     "AdjustDropper"); R   = hardwareMap.get(Servo.class,     "Rocket");

        // Initialize IMU
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection .RIGHT)));

        // Initialize Color Sensor
        colors = hardwareMap.get(ColorSensor.class, "sensor_color");

        // Reverse Servo
        RA .setDirection(Servo.Direction.REVERSE);
        RH .setDirection(Servo.Direction.REVERSE);
        ALL.setDirection(Servo.Direction.REVERSE);
        ADP.setDirection(Servo.Direction.REVERSE);
        // Set Servo Position
        SetServoPos(DuoServoAng[0], LA, RA);
        SetServoPos(DuoServoAng[1], LH, RH);
        SetServoPos(DuoServoAng[2], ALL, ARL);
        SetServoPos(ServoAng[0], IT);
        SetServoPos(ServoAng[1], DP);
        SetServoPos(ServoAng[2], ADP);
        SetServoPos(ServoAng[3], R);
        // Reverse Motors
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        LL.setDirection(DcMotorSimple.Direction.REVERSE);
        RL.setDirection(DcMotorSimple.Direction.REVERSE);
        PU.setDirection(DcMotorSimple.Direction.REVERSE);
        V .setDirection(DcMotorSimple.Direction.REVERSE);
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
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PU.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        V .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // SetPower Motors
        MovePower(0, 0, 0, 0);
        LL.setPower(0);
        RL.setPower(0);
        PU.setPower(0);
        V .setPower(0);
    }
}
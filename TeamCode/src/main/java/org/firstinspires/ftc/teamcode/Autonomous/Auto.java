package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Utilize.AngleWrap;
import static org.firstinspires.ftc.teamcode.Utilize.atTargetRange;
import static org.firstinspires.ftc.teamcode.Utilize.signNum;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AprilTag.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Robot;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Objects;

@Config
@Autonomous(name = "Autonomous")
public class Auto extends LinearOpMode {
    /** PID Variables */
    public static double R_Kp=1.6, R_Ki=0.085, R_Kd=0.09;

    /** Usage External Class */
    Robot      robot = new Robot();
    Controller pid_R = new Controller(R_Kp, R_Ki, R_Kd, 0);

    /** Hardware */
    IMU imu;
    Servo LA, RA, K;
    DcMotorEx FL, FR, BL, BR, B, LL, ML, RL;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    /** Variables */
    double yaw;
    double Heading=0;
    double[]       CurrentXY = {1.46305914893617, 0.3660998492209};
    final double[] Tile_Size = {23.5, 23.5};  // Width * Length
    public static double K_pos=0, Arm_pos=0;
    int[] ConeLift_Level = {150, 105, 80, 30, 0, 0};
    int Base_ErrorTolerance = 3;

    private void Init(){
        // HardwareMap
        imu = hardwareMap.get(IMU.class,     "imu");
        FL  = hardwareMap.get(DcMotorEx.class, "Front_Left");
        FR  = hardwareMap.get(DcMotorEx.class, "Front_Right");
        BL  = hardwareMap.get(DcMotorEx.class, "Back_Left");
        BR  = hardwareMap.get(DcMotorEx.class, "Back_Right");
        B   = hardwareMap.get(DcMotorEx.class, "Base");
        LL  = hardwareMap.get(DcMotorEx.class, "Left_Lift");
        ML  = hardwareMap.get(DcMotorEx.class, "Middle_Lift");
        RL  = hardwareMap.get(DcMotorEx.class, "Right_Lift");
        LA  = hardwareMap.get(Servo.class,   "Left_Arm");
        RA  = hardwareMap.get(Servo.class,   "Right_Arm");
        K   = hardwareMap.get(Servo.class,   "Keeper");

        // Initialize Robot
        robot.Initialize(imu, DcMotor.RunMode.RUN_WITHOUT_ENCODER, FL, FR, BL, BR, B, LL, ML, RL,
                Arm_pos, LA, RA, K_pos, K);
        imu.resetYaw();

        // Initialize Camera
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                 hardwareMap.get(WebcamName.class, "Webcam 1"),cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(0.166, 578.272, 578.272, 402.145, 221.506);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() { camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT); }

            @Override
            public void onError(int errorCode) {}
        });

        // Show FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    private void WaitForStart() {
        boolean Base_atSetpoint = false;
        while (!isStarted() && !isStopRequested()) {
            if (!Base_atSetpoint) {
                Base_atSetpoint = robot.Turn_Base(45, 0.5, Base_ErrorTolerance);
                continue;
            }
            B.setPower(0);
            B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.update();
            int[] ID_TAG_OF_INTEREST = {8, 10, 15};
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() == 0) {
                telemetry.addLine("Not Found");
                continue;
            }
            String[] pos = {"Left", "Middle", "Right"};
            AprilTagDetection tag = currentDetections.get(0);
            for (int i = 0; i <= 2; i++) {
                if (tag.id == ID_TAG_OF_INTEREST[i]) {
                    telemetry.addLine(String.format("Found %s", pos[i]));
                    break;
                }
            }
        }
    }

    private void Break(double stopSecond) {
        if (stopSecond == 0) return;
        robot.MovePower(0, 0, 0, 0);
        robot.MoveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.LiftPower(0);
        sleep((long) (stopSecond * 1000));
    }

    private void Lift(int height) {
        int CurrentPosition = Math.max(LL.getCurrentPosition(), Math.max(ML.getCurrentPosition(), RL.getCurrentPosition()));
        double Lift_Power = (atTargetRange(CurrentPosition, height, 8) ? 0 : (CurrentPosition < height ? 1 : -0.75));
        robot.LiftPower(Lift_Power);
    }

    private void Move(double power, double Kp, double Ki, double Kd, double targetX, double targetY, double stopSecond,
                      int Base_angle, double K_pos, double Arm_pos, int Height) {
        yaw = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double X        = targetX - CurrentXY[0];
        double Y        = targetY - CurrentXY[1];
        double absX     = Math.abs(X);
        double absY     = Math.abs(Y);
        double X_Power  = absX > absY ? power : (absX / absY) * power;
        double Y_Power  = absX > absY ? (absY / absX) * power : power;
        double X1       = X_Power * signNum(X);
        double Y1       = Y_Power * signNum(Y);
        double X_Inches = Tile_Size[0] * Math.abs(X);
        double Y_Inches = Tile_Size[1] * Math.abs(Y);
        double X1_In    = X_Inches * signNum(X);
        double Y1_In    = Y_Inches * signNum(Y);
        double X2_In    = (Math.cos(Heading) * X1_In) - (Math.sin(Heading) * Y1_In);
        double Y2_In    = (Math.sin(Heading) * X1_In) + (Math.cos(Heading) * Y1_In);
        double FL_BR_In = Y2_In + X2_In;
        double FR_BL_In = Y2_In - X2_In;
        boolean Base_atSetpoint = false;

        PIDCoefficients pid = new PIDCoefficients(Kp, Ki, Kd);
        FL.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);
        FR.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);
        BL.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);
        BR.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);
        CurrentXY = new double[] {targetX, targetY};
        robot.MoveTargetPosition(FL_BR_In, FR_BL_In, FR_BL_In, FL_BR_In);
        robot.MoveMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && robot.MoveisBusy()) {
            yaw = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double X2  = (Math.cos(Heading) * X1) - (Math.sin(Heading) * Y1);
            double Y2  = (Math.sin(Heading) * X1) + (Math.cos(Heading) * Y1);
            // Rotate
            double R = pid_R.Calculate(AngleWrap(Heading - yaw));
//            double R =0;
            // Denominator for division to get no more than 1
            double D = Math.max(Math.abs(X2) + Math.abs(Y2) + Math.abs(R), 1);
            robot.MovePower((Y2 + X2 + R) / D, (Y2 - X2 - R) / D,
                            (Y2 - X2 + R) / D,  (Y2 + X2 - R) / D);

//            telemetry.addData("X",X);
//            telemetry.addData("Y",Y);
//            telemetry.addData("X_Power",X_Power);
//            telemetry.addData("Y_Power",Y_Power);
//            telemetry.addData("X1",X1);
//            telemetry.addData("Y1",Y1);
//            telemetry.addData("X1_In",X1_In);
//            telemetry.addData("Y1_In",Y1_In);
//            telemetry.addData("X2_In",X2_In);
//            telemetry.addData("Y2_In",Y2_In);
//            telemetry.addData("X_Inches",X_Inches);
//            telemetry.addData("Y_Inches",Y_Inches);
//            telemetry.addData("FL_BR_In",FL_BR_In);
//            telemetry.addData("FR_BL_In",FR_BL_In);
//            telemetry.addData("FL", ((int) (FL_BR_In * robot.Counts_per_Inch)));
//            telemetry.addData("FLC", FL.getCurrentPosition());
//            telemetry.addData("FR", ((int) (FR_BL_In * robot.Counts_per_Inch)));
//            telemetry.addData("FRC", FR.getCurrentPosition());
//            telemetry.addData("BL", ((int) (FR_BL_In * robot.Counts_per_Inch)));
//            telemetry.addData("BLC", BL.getCurrentPosition());
//            telemetry.addData("BR", ((int) (FL_BR_In * robot.Counts_per_Inch)));
//            telemetry.addData("BRC", BR.getCurrentPosition());
//            telemetry.update();

            Lift(Height);
            K.setPosition(K_pos);
            robot.setArmPosition(Arm_pos);
            if (!Base_atSetpoint) {
                Base_atSetpoint = robot.Turn_Base(Base_angle, 0.5, Base_ErrorTolerance);
                continue;
            }
            B.setPower(0);
            B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        Break(stopSecond);
    }

    private void Turn(double Turn_angle, int Height, int Base_angle, double stopSecond) {
        Turn_angle = Math.toRadians(Turn_angle);
        Heading = Turn_angle;
        boolean Base_atSetpoint = false;
        while (opModeIsActive()) {
            yaw = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double power = 0.4;
            double error = AngleWrap(Turn_angle - yaw);
            double R = error == 0 ? 0 : (error < 0 ? -power : power);
            if (Math.abs(error) == (Math.toRadians(0))) break;
            robot.MovePower(R, -R, R,-R);

            Lift(Height);
            if (!Base_atSetpoint) {
                Base_atSetpoint = robot.Turn_Base(Base_angle, 0.5, Base_ErrorTolerance);
                continue;
            }
            B.setPower(0);
            B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        Break(stopSecond);
    }

    @Override
    public void runOpMode() {
        Init();
        WaitForStart();
        if (opModeIsActive()) {
            Move(1, 1.6, 0.01, 0, 1.9, 2.59, 0.25, 45, 0, 0, 740);
            robot.setArmPosition(0.35);
            sleep(100);
            K.setPosition(0.25);
            Lift(130);
            sleep(100);
            Move(1, 2, 0.085, 0, 1.5, 2.5, 0.25, 45, 0.25, 0.35, 130);
            Turn(-90, 130, 0, 0.25);
            Move(1, 2, 0.07, 0, 0.65, 2.5, 0.25, 0, 0.2, 0.35, ConeLift_Level[0]);
//            for (int i=0; i<=4; i++) {
                if (0 != 0) Move(1, 1.6, 0.01, 0, 0.65, 2.5, 0.25, 0, 0.2, 0.35, ConeLift_Level[0]);
                K.setPosition(0);
                sleep(400);
                Move(1, 2, 0.07, 0, 2.125, 2.5, 0.25, 90, 0, 0, robot.High_Junction);
                robot.setArmPosition(0.55);
                sleep(300);
                K.setPosition(0.15);
                sleep(150);
                robot.setArmPosition(0);
                sleep(300);
//            }
//            Move(0.75, 0.7, 2.475, 0.3, 0, 0.2, 0.35, 120);
//            K.setPosition(0);
//            sleep(500);

//            for (int i=0; i<4; i++) {
//                Move(0.6, 1.5, 5.5, 0.5, 0, 0, 0, 0);
//                sleep(1000);
//                Move(0.6, 1.5, 1.5, 0.5, 0, 0, 0, 0);
//            }

//            while (opModeIsActive()) {
//                if (Start) {
//                    Move(Power, Kp, Ki, Kd, TargetX, TargetY, 0.3, Base_angle, K_pos, Arm_pos, Height);
//                    Turn(Turn_angle, Height, Base_angle, 0.3);
//                    Start=false;
//                }
//            }
        }
    }
}
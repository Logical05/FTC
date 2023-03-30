package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AprilTag.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Calculate;
import org.firstinspires.ftc.teamcode.Robot;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Autonomous")
public class Auto extends LinearOpMode {
    /** Usage External Class */
    Robot     robot = new Robot();
    Calculate calc  = new Calculate();
    /** Hardware */
    IMU imu;
    Servo LA, RA, K;
    DcMotor FL, FR, BL, BR, B, LL, RL;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    /** Variables */
    double[]    CurrentXY   = {0, 0};
    final int[] Tile_Size   = {24, 24};  // Width * Length

    private void Init(){
        // HardwareMap
        imu = hardwareMap.get(IMU.class,     "imu");
        FL  = hardwareMap.get(DcMotor.class, "Front_Left");
        FR  = hardwareMap.get(DcMotor.class, "Front_Right");
        BL  = hardwareMap.get(DcMotor.class, "Back_Left");
        BR  = hardwareMap.get(DcMotor.class, "Back_Right");
        B   = hardwareMap.get(DcMotor.class, "Base");
        LL  = hardwareMap.get(DcMotor.class, "Left_Lift");
        RL  = hardwareMap.get(DcMotor.class, "Right_Lift");
        LA  = hardwareMap.get(Servo.class,   "Left_Arm");
        RA  = hardwareMap.get(Servo.class,   "Right_Arm");
        K   = hardwareMap.get(Servo.class,   "Keeper");

        // Initialize Robot
        robot.Initialize(imu, DcMotor.RunMode.RUN_USING_ENCODER, FL, FR, BL, BR,B, LL, RL,
                0, LA, RA, 0, K);

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
    }

    private void WaitForStart() {
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("FL", FL.getCurrentPosition());
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

    private void Move(double Power, double Kp, double Ki, double Kd, double TargetX, double TargetY) {
        double[] K_PID  = {Kp, Ki, Kd};
        double X        = TargetX - CurrentXY[0];
        double Y        = TargetY - CurrentXY[1];
        double X1       = Power * (X / Math.abs(X));
        double Y1       = Power * (Y / Math.abs(Y));
        double X_Inches = Tile_Size[0] * Math.abs(X);
        double Y_Inches = Tile_Size[1] * Math.abs(Y);
        double Inches   = Math.sqrt((X_Inches * X_Inches) + (Y_Inches * Y_Inches));

        robot.MoveTargetPosition(Inches);
        robot.MoveMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())) {
            double yaw = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double X2  = (Math.cos(yaw) * X1) - (Math.sin(yaw) * Y1);
            double Y2  = (Math.sin(yaw) * X1) + (Math.cos(yaw) * Y1);
            double PID = calc.PIDControl(K_PID, 0, yaw);
            // Rotate Condition
            double R = calc.Plus_Minus(Math.toDegrees(calc.Error), 0, 0.45) ? 0 : PID;
            // Denominator for division to get no more than 1
            double D = Math.max(Math.abs(X2) + Math.abs(Y2) + Math.abs(R), 1);
            robot.MovePower((Y2 + X2 + R) / D, (Y2 - X2 - R) / D,
                            (Y2 - X2 + R) / D, (Y2 + X2 - R) / D);
        }

        robot.MovePower(0, 0, 0, 0);
        robot.MoveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        CurrentXY = new double[]{TargetX, TargetY};
        sleep(250);
    }

    @Override
    public void runOpMode() {
        Init();
        WaitForStart();
        if (opModeIsActive()) {
//            Move(0.25, 0, 0, 0, 0, 1);
        }
    }
}
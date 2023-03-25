package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Robot;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Autonomous")
public class Auto extends LinearOpMode {
    /** Usage External Class */
    Robot robot = new Robot();

    /** Hardware */
    IMU imu;
    Servo LA, RA, K;
    DcMotor FL, FR, BL, BR, B, LL, RL;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    /** Variables */


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
            public void onError(int errorCode) { }
        });
    }

    private void WaitForStart() {
        while (!isStarted() && !isStopRequested()) {
            telemetry.update();
            int[] ID_TAG_OF_INTEREST = {8, 10, 15};
            String[] pos = {"Left", "Middle", "Right"};
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() == 0) {
                telemetry.addLine("Not Found");
                continue;
            }
            for (AprilTagDetection tag : currentDetections) {
                for (int i = 0; i <= 2; i++) {
                    if (tag.id == ID_TAG_OF_INTEREST[i]) {
                        telemetry.addLine(String.format("Found %s", pos[i]));
                        break;
                    }
                }
                break;
            }
        }
    }

    private void Move(double Setpoint, double Pwr_X, double Pwr_Y){
        double[] K_PID = {0.8, 0.3, 0.1};
        double Beta   = robot.yaw;
        double Pwr_X2 = (Math.cos(Beta) * Pwr_X) - (Math.sin(Beta) * Pwr_Y);
        double Pwr_Y2 = (Math.sin(Beta) * Pwr_X) - (Math.cos(Beta) * Pwr_Y);
        double PID = robot.PIDControl(Setpoint, K_PID);
        // Rotate Condition
        double R = robot.Plus_Minus(Math.toDegrees(robot.error), 0, 0.45) ? 0 : PID;
        // Denominator for division to get no more than 1
        double D = Math.max(Math.abs(Pwr_X2) + Math.abs(Pwr_Y2) + Math.abs(R), 1);
        robot.MovePower((Pwr_Y2 + Pwr_X2 + R)/ D, (Pwr_Y2 - Pwr_X2 - R)/ D,
                        (Pwr_Y2 - Pwr_X2 + R)/ D,  (Pwr_Y2 + Pwr_X2 - R)/ D);
        robot.PID_timer.reset();
    }

    @Override
    public void runOpMode() {
        Init();
        WaitForStart();
        if (opModeIsActive()) {

        }
    }
}
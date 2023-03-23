package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

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
    DcMotor FL, FR, BL, BR, B;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    /** Variables */


    private void Init(){
        // HardwareMap
        imu  = hardwareMap.get(IMU.class,     "imu");
        FL   = hardwareMap.get(DcMotor.class, "Front_Left");
        FR   = hardwareMap.get(DcMotor.class, "Front_Right");
        BL   = hardwareMap.get(DcMotor.class, "Back_Left");
        BR   = hardwareMap.get(DcMotor.class, "Back_Right");
//        B    = hardwareMap.get(DcMotor.class, "Base");
        LA   = hardwareMap.get(Servo.class,   "Left_Arm");
        RA   = hardwareMap.get(Servo.class,   "Right_Arm");
        K    = hardwareMap.get(Servo.class,   "Keeper");

        // Initialize Robot
        robot.Initialize(imu, DcMotor.RunMode.RUN_WITHOUT_ENCODER, FL, FR, BL, BR, B, 0, LA, RA, 0, K);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                                                                                      "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(0.166, 578.272, 578.272, 402.145, 221.506);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    @Override
    public void runOpMode() {
        Init();
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
                for (int i=0; i<=2; i++) {
                    if (tag.id == ID_TAG_OF_INTEREST[i]) {
                        telemetry.addLine(String.format("Found %s", pos[i]));
                        break;
                    }
                }
                break;
            }
        }
        if (opModeIsActive()) {

        }
    }
}
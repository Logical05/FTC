package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;


@Config
@Autonomous(name="Auto")
public class Auto extends Robot {
    TfodProcessor tfod;
    VisionPortal visionPortal;
    IMU imu;
    Servo LLL, LRL, LA, RA, LH, RH, K, KA, R;
    DcMotorEx FL, FR, BL, BR, LL, RL, PU, V;

    final String   TFOD_MODEL_ASSET = "Signal.tflite";
    final String[] LABELS           = {"Signal"};
    String signalPos;

    private void Init() {
        // HardwareMap
        imu = hardwareMap.get(IMU.class,       "imu");
        FL  = hardwareMap.get(DcMotorEx.class, "Front_Left");   FR  = hardwareMap.get(DcMotorEx.class, "Front_Right");
        BL  = hardwareMap.get(DcMotorEx.class, "Back_Left");    BR  = hardwareMap.get(DcMotorEx.class, "Back_Right");
        LL  = hardwareMap.get(DcMotorEx.class, "Left_Lift");    RL  = hardwareMap.get(DcMotorEx.class, "Right_Lift");
        PU  = hardwareMap.get(DcMotorEx.class, "Pull_Up");      V   = hardwareMap.get(DcMotorEx.class, "Vacuum");
        LLL = hardwareMap.get(Servo.class,     "Lift_LL");      LRL = hardwareMap.get(Servo.class,     "Lift_RL");
        LA  = hardwareMap.get(Servo.class,     "Left_Arm");     RA  = hardwareMap.get(Servo.class,     "Right_Arm");
        LH  = hardwareMap.get(Servo.class,     "Left_Hoist");   RH  = hardwareMap.get(Servo.class,     "Right_Hoist");
        K   = hardwareMap.get(Servo.class,     "Keep");         KA  = hardwareMap.get(Servo.class,     "Keep_Arm");
        R   = hardwareMap.get(Servo.class,     "Rocket");

        // Initialize Robot
        Initialize(imu, DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                         FL, FR, BL, BR, LL, RL, PU, V,
                         LLL, LRL, LA, RA, LH, RH, K, KA, R,
                         new double[]{1, 0.1, 0.1},
                         new double[]{0.1, 0.1, 0.1});
        imu.resetYaw();

        // Initialize TensorFlow Object Detection (TFOD)
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera.
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
//        tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
//        visionPortal.setProcessorEnabled(tfod, true);

        // Show FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    private void WaitForStart() {
        while (!isStarted() && !isStopRequested()) {
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                signalPos = x < 200 ? "Left"  :
                            x > 600 ? "Right" : "Mid";
                telemetry.addData("- Position", "%.0f", x);
            }
            telemetry.update();
        }
    }

    @Override
    public void runOpMode() {
        Init();
        WaitForStart();
        visionPortal.stopStreaming();
        if (opModeIsActive()) {

        }
    }
}
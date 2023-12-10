package org.firstinspires.ftc.teamcode;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

import static org.firstinspires.ftc.teamcode.Utilize.*;

@Config
@Autonomous(name="Auto")
public class Auto extends LinearOpMode {
    Controller pid_R = new Controller(1.6, 0.01, 0.09, 0);
    TfodProcessor tfod;
    VisionPortal visionPortal;
    IMU imu;
    Servo LLL, LRL, LA, RA, LH, RH, K, KA, R;
    DcMotorEx FL, FR, BL, BR, LL, RL, PU, V;

    double         heading   = 0;
    double[]       currentXY = {0, 0};
    final double[] tileSize  = {23.5, 23.5};  // Width * Length
    private static final String   TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    private static final String[] LABELS           = {"Pixel"};

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
        Robot.Initialize(imu, DcMotor.RunMode.RUN_WITHOUT_ENCODER,
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
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
        }

        visionPortal.stopStreaming();

        // Show FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    private void WaitForStart() {
        while (!isStarted() && !isStopRequested()) {

        }
    }

    private void Break(double stopSecond) {
        if (stopSecond == 0) return;
        Robot.MovePower(0, 0, 0, 0);
        Robot.MoveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep((long) (stopSecond * 1000));
    }

    private void Move(double power, double kp, double ki, double targetX, double targetY,
                      double stopSecond, double timeOut) {
        double x        = targetX - currentXY[0];
        double y        = targetY - currentXY[1];
        double absX     = Math.abs(x);
        double absY     = Math.abs(y);
        double powerX   = absX > absY ? power : (absX / absY) * power;
        double powerY   = absX < absY ? power : (absY / absX) * power;
        double x1       = powerX * SigNum(x);
        double y1       = powerY * SigNum(y);
        double inchesX1 = tileSize[0] * x;
        double inchesY1 = tileSize[1] * y;
        double inchesX2 = (Math.cos(heading) * inchesX1) - (Math.sin(heading) * inchesY1);
        double inchesY2 = (Math.sin(heading) * inchesX1) + (Math.cos(heading) * inchesY1);
        double FL_BR_In = inchesY2 + inchesX2;
        double FR_BL_In = inchesY2 - inchesX2;

        PIDCoefficients pid = new PIDCoefficients(kp, ki, 0);
        FL.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);
        FR.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);
        BL.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);
        BR.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);
        Robot.MoveTargetPosition(FL_BR_In, FR_BL_In, FR_BL_In, FL_BR_In);
        Robot.MoveMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (opModeIsActive()) {
            double yaw = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double x2 = (Math.cos(heading) * x1) - (Math.sin(heading) * y1);
            double y2 = (Math.sin(heading) * x1) + (Math.cos(heading) * y1);
            // Rotate
            double r =  pid_R.Calculate(WrapRads(heading - yaw));
            // Denominator for division to get no more than 1
            double d = Math.max(Math.abs(x2) + Math.abs(y2) + Math.abs(r), 1);
            Robot.MovePower((y2 + x2 + r) / d, (y2 - x2 - r) / d,
                            (y2 - x2 + r) / d,  (y2 + x2 - r) / d);

            if (((runtime.seconds() >= timeOut) && (timeOut != 0)) || !Robot.MoveisBusy()) break;
        }
        Break(stopSecond);
        currentXY = new double[] {targetX, targetY};
    }

    private void Turn(double degs, double stopSecond) {
        double rads = Math.toRadians(degs);
        while (opModeIsActive()) {
            double yaw      = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double error    = WrapRads(rads - yaw);
            double r        = AtTargetRange(error, 0, Math.toRadians(30)) ? 0.2 : 0.6;
            if (error < 0) r = -r;
            Robot.MovePower(r, -r, r,-r);

            if (AtTargetRange(error, 0, 0.05)) break;
        }
        Break(stopSecond);
        heading = rads;
    }

    @Override
    public void runOpMode() {
        Init();
        WaitForStart();
        if (opModeIsActive()) {

        }
    }
}

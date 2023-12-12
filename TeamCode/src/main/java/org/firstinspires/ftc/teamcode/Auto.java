package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Config
@Autonomous(name="Auto")
public class Auto extends Robot {
    private char signalPos;

    private void Init() {
        // Initialize Robot
        Initialize(DcMotor.RunMode.RUN_WITHOUT_ENCODER, new double[]{0, 0, 0},
                                                        new double[]{0, 0, 0.1, 0});
        imu.resetYaw();

        // Initialize TensorFlow Object Detection (TFOD)
        TfodInit("Webcam 1", "Signal.tflite", new String[]{"Signal"}, 0.75f);

        // Show FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    private void WaitForStart() {
        while (!isStarted() && !isStopRequested()) {
            signalPos = 'L';
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                signalPos = x < 320 ? 'M'  : 'R';
                telemetry.addData("Position", "%.0f", x);
            }
            telemetry.addData("SignalPos", signalPos);
            telemetry.addData("Red", colors.red());
            telemetry.addData("Green", colors.green());
            telemetry.addData("Blue", colors.blue());
            telemetry.update();
        }
    }

    @Override
    public void runOpMode() {
        Init();
        WaitForStart();
        visionPortal.close();
        if (opModeIsActive()) {
            Move(0.6, 4.75, 0.23, 0, 1, 0.25, 0);
            if (signalPos == 'M') {
                V.setPower(0.4);
                Break(0.1);
                V.setPower(0);
            }
            Turn(-90, 0.25);
        }
    }
}

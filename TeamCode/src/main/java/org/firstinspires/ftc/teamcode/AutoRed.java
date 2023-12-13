package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@Config
@Autonomous(name="AutoRed")
public class AutoRed extends Robot {
    private char signalPos;

    private void Init() {
        // Initialize Robot
        Initialize(DcMotor.RunMode.RUN_WITHOUT_ENCODER, new double[]{0, 0, 0},
                                                        new double[]{0, 0, 0.1, 0});
        imu.resetYaw();

        // Initialize TensorFlow Object Detection (TFOD)
        TfodInit("Webcam 1", "Signal.tflite", new String[]{"Signal"}, 0.7f);

        // Show FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    private void WaitForStart() {
        while (!isStarted() && !isStopRequested()) {
            signalPos = 'R';
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                signalPos = x < 320 ? 'L'  : 'M';
                telemetry.addData("Position", "%.0f", x);
            }
            telemetry.addData("SignalPos", signalPos);
            telemetry.addData("Red", colors.red());
            telemetry.addData("Green", colors.green());
            telemetry.addData("Blue", colors.blue());
            telemetry.update();
        }
    }

    private void Drop() {
        V.setPower(-0.4);
        Break(1);
        V.setPower(0);
    }

    @Override
    public void runOpMode() {
        Init();
        WaitForStart();
        visionPortal.close();
        if (opModeIsActive()) {
            Move(0.5, 4.5, 0.23, 0, 1, 0.25, 0);
            if (signalPos == 'M') {
                Drop();
            }
            Turn(-90, 0.25);
            Move(0.35, 4.5, 0.23, 0, 1.18, 0.25, 0);
            if (signalPos == 'L') {
                Drop();
            }
            Move(0.9, 4.5, 0.23, 3.1, 1.18, 0.25, 0);
//            Move(0.5, 4.75, 0.23, -1, 0, 0.25, 0);
        }
    }
}

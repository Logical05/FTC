package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utilize.AtTargetRange;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@Config
@Autonomous(name="AutoRedOut", group = "Out")
public class AutoRedOut extends Robot {
    private char signalPos;

    private void Init() {
        // Initialize Robot
        Initialize(DcMotor.RunMode.RUN_WITHOUT_ENCODER, new double[]{0, 0, 0},
                                                        new double[]{0.3, 0, 0.1, 0});
        imu.resetYaw();

        // Initialize TensorFlow Object Detection (TFOD)
        TfodInit("Webcam 1", "Signal.tflite", new String[]{"Signal"}, 0.6f);

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
            telemetry.update();
        }
    }

    private void Drop() {
        V.setPower(0.1);
        Break(1.2);
        V.setPower(0);
    }

    private void Lift(double height) {
        double LARA_pos = height == 0 ? 0.2 : 0.92;
        double AL_Ang   = height == 0 ? 0   : 0.043;
        double AD_Ang   = height == 0 ? 0   : 0.453;
        SetServoPos(0.3, IT);
        SetServoPos(AL_Ang, ALL, ARL);
        SetServoPos(LARA_pos, LA, RA);
        SetServoPos(AD_Ang, ADP);
        while (true) {
            double  curPos     = Math.max(LL.getCurrentPosition(), RL.getCurrentPosition());
            double  Lift_Power = AtTargetRange(curPos, height, 10) ?  0 :
                                              (curPos > height ? -0.2    :  1);
            LiftPower(Lift_Power);
            if (Lift_Power == 0) break;
        }
    }

    private void Dropper() {
        Break(1);
        SetServoPos(0, IT);
        SetServoPos(0.2, DP);
        Break(0.5);
    }

    @Override
    public void runOpMode() {
        Init();
        WaitForStart();
        visionPortal.close();
        if (opModeIsActive()) {
            if (signalPos == 'M') {
                Drop();
            }
            Move(0.7, 4.5, 0.23, 0, 1.1, 0.25, 0);
            Move(0.35, 4.5, 0.23, 0, 0.95, 0.25, 0);
            Turn(-90, 0.25);
            Move(0.8, 4.5, 0.23, 0, 1.175, 0.25, 0);
            if (signalPos == 'L') {
                Drop();
            }
            if (signalPos == 'R') {
                Move(0.7, 4.5, 0.23, 0.89, 1.175, 0.25, 0);
                Drop();
            }
            Move(0.9, 4.5, 0.23, 3.6, 1.175, 0.4, 0);
            Lift(800);
            if (signalPos == 'L') {
                Move(0.7, 4.5, 0.23, 3.6, 1.5, 0.25, 0);
            }
            if (signalPos == 'R') {
                Move(0.7, 4.5, 0.23, 3.6, 0.8, 0.25, 0);
            }
            Dropper();
            Lift(0);
            Move(0.7, 4.75, 0.23, 3.6, 2.35, 0.25, 0); // Mid
//            Move(0.7, 4.75, 0.23, 3.33, 0.25, 0.25, 0); // Corner
            Move(0.9, 4.75, 0.23, 3.65, 2.35, 0.25, 0);
        }
    }
}

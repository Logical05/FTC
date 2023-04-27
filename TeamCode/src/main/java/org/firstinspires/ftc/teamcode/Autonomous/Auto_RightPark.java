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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AprilTag.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.Robot;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(name = "Auto_RightPark")
public class Auto_RightPark extends LinearOpMode {
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
    int            Base_ErrorTolerance = 1;
    int            Tag_ID              = 0;
    int[]          ConeLift_Level      = {100, 85, 70, 30, 0, 0};
    double[]       CurrentXY           = {-1.47068085106383, 0.3660998492209};
    final double[] Tile_Size           = {23.5, 23.5};  // Width * Length
    double yaw, Heading=0, K_pos=0, Arm_pos=0;

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
        B  .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LL .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ML .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RL .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
                Base_atSetpoint = robot.Turn_Base(0, 0.5, Base_ErrorTolerance);
                continue;
            }
            B.setPower(0);
            B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.update();
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() == 0) {
                telemetry.addLine("Not Found");
                continue;
            }
            int[] ID_TAG_OF_INTEREST = {8, 10, 15};
            String[] pos = {"Left", "Middle", "Right"};
            AprilTagDetection tag = currentDetections.get(0);
            for (int i = 0; i <= 2; i++) {
                if (tag.id == ID_TAG_OF_INTEREST[i]) {
                    Tag_ID = ID_TAG_OF_INTEREST[i];
                    telemetry.addLine(String.format("Found %s", pos[i]));
                    telemetry.addData("Tag_ID", Tag_ID);
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

    private boolean Lift(int height) {
        int CurrentPosition = Math.max(LL.getCurrentPosition(), Math.max(ML.getCurrentPosition(), RL.getCurrentPosition()));
        double Lift_Power = atTargetRange(CurrentPosition, height, 8) ?    0 :
                                          CurrentPosition < height           ?    1 :
                                          CurrentPosition - 25 < height      ? -0.3 : -0.75;
        robot.LiftPower(Lift_Power);
        return Lift_Power == 0;
    }

    private void Move(double power, double Kp, double Ki, double targetX, double targetY, double stopSecond,
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
        boolean Lift_atTarget   = false;
        boolean MoveisBusy      = true;

        PIDCoefficients pid = new PIDCoefficients(Kp, Ki, 0);
        FL.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);
        FR.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);
        BL.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);
        BR.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);
        CurrentXY = new double[] {targetX, targetY};
        robot.MoveTargetPosition(FL_BR_In, FR_BL_In, FR_BL_In, FL_BR_In);
        robot.MoveMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && (MoveisBusy || !Lift_atTarget)) {
//            if (MoveisBusy) {
                yaw = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double X2 = (Math.cos(Heading) * X1) - (Math.sin(Heading) * Y1);
                double Y2 = (Math.sin(Heading) * X1) + (Math.cos(Heading) * Y1);
                // Rotate
                double R = pid_R.Calculate(AngleWrap(Heading - yaw));
                // Denominator for division to get no more than 1
                double D = Math.max(Math.abs(X2) + Math.abs(Y2) + Math.abs(R), 1);
                robot.MovePower((Y2 + X2 + R) / D, (Y2 - X2 - R) / D,
                        (Y2 - X2 + R) / D, (Y2 + X2 - R) / D);
                MoveisBusy = robot.MoveisBusy();
//            } else {
//                robot.MovePower(0, 0, 0, 0);
//                robot.MoveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            }

            Lift_atTarget = Lift(Height);
            K.setPosition(K_pos);
            robot.setArmPosition(Arm_pos);
            if (!Base_atSetpoint) {
                Base_atSetpoint = robot.Turn_Base(Base_angle, 0.75, Base_ErrorTolerance);
                continue;
            }
            B.setPower(0);
            B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        Break(stopSecond);
//        robot.Error_FL = robot.FL_Target - FL.getCurrentPosition();
//        robot.Error_FR = robot.FR_Target - FR.getCurrentPosition();
//        robot.Error_BL = robot.BL_Target - BL.getCurrentPosition();
//        robot.Error_BR = robot.BR_Target - BR.getCurrentPosition();
    }

    private void Turn(double Turn_angle, int Height, int Base_angle, double stopSecond) {
        Turn_angle = Math.toRadians(Turn_angle);
        robot.Error_FL = robot.Error_FR = robot.Error_BL = robot.Error_BR = 0;
        Heading = Turn_angle;
        boolean Base_atSetpoint = false;
        while (opModeIsActive()) {
            yaw = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double maxpower = 0.6;
            double minpower = 0.2;
            double error = AngleWrap(Turn_angle - yaw);
            double R = atTargetRange(error, 0, 0.05)        ? 0 :
                       atTargetRange(error, 0, Math.toRadians(30)) ?
                                    (error < 0 ? -minpower : minpower)   :
                                    (error < 0 ? -maxpower : maxpower);
            if (atTargetRange(error, 0, 0.05)) break;
            robot.MovePower(R, -R, R,-R);

            Lift(Height);
            if (!Base_atSetpoint) {
                Base_atSetpoint = robot.Turn_Base(Base_angle, 0.75, Base_ErrorTolerance);
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
            Move(0.6, 4.75, 0.23, CurrentXY[0], 1.5, 0.25, 0, 0, 0, 0);
            switch (Tag_ID) {
                case 8:
                    Turn(90, 0, 0, 0.25);
                    Move(1, 4.75, 0.23, -2.5, 1.5, 30, 0, 0, 0, 0);
                case 10:
                    Turn(90, 0, 0, 0.25);
                    Move(1, 4.75, 0.23, -1.5, 1.5, 30, 0, 0, 0, 0);
                case 15:
                    Turn(90, 0, 0, 0.25);
                    Move(1, 4.75, 0.23, -0.5, 1.5, 30, 0, 0, 0, 0);
            }
        }
    }
}
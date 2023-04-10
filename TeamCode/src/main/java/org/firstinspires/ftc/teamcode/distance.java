package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "distance")
public class distance extends LinearOpMode {

    @SuppressLint("DefaultLocale")
    private void distance(Rev2mDistanceSensor distance){
        telemetry.addData("deviceName", distance.getDeviceName() );
        telemetry.addData("range", String.format("%.01f mm", distance.getDistance(DistanceUnit.MM)));
        telemetry.addData("range", String.format("%.01f cm", distance.getDistance(DistanceUnit.CM)));
        telemetry.addData("range", String.format("%.01f m", distance.getDistance(DistanceUnit.METER)));
        telemetry.addData("range", String.format("%.01f in", distance.getDistance(DistanceUnit.INCH)));

        // Rev2mDistanceSensor specific methods.
        telemetry.addData("ID", String.format("%x", distance.getModelID()));
        telemetry.addData("did time out", Boolean.toString(distance.didTimeoutOccur()));
    }

    public void runOpMode(){
        // you can use this as a regular DistanceSensor.
        Rev2mDistanceSensor distance0 = hardwareMap.get(Rev2mDistanceSensor.class, "distance0");
        Rev2mDistanceSensor distance1 = hardwareMap.get(Rev2mDistanceSensor.class, "distance1");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            // generic DistanceSensor methods.
            distance(distance0);
            distance(distance1);
            telemetry.update();
        }
    }

}

package org.firstinspires.ftc.teamcode;

public class Utilize {
    public static boolean atTargetRange(double number, double target, double range) {
        return target - range < number && number < target + range;
    }

    public static double AngleWrap(double radians) {
        if (radians > Math.PI)  radians -= 2 * Math.PI;
        if (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    public static int signNum(double num) { return num == 0 ? 0 : (num < 0 ? -1 : 1); }
}

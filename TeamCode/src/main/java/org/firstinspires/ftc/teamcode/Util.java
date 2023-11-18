package org.firstinspires.ftc.teamcode;

public class Util {

    public static double normalizeDegrees(double degrees) {
        return (degrees % 360 + 360) % 360;
    }

    public static boolean atTargetTolerance(double input, double target, double tolerance) {
        return (input >= (target - tolerance)) && (input <= (target + tolerance));
    }

}

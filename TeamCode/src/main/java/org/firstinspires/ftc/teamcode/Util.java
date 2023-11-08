package org.firstinspires.ftc.teamcode;

public class Util {

    public static double normalizeDegrees(double degrees) {
        return (degrees % 360 + 360) % 360;
    }

    public static boolean atTargetTolerance(double input, double tolerance) {
        return (input >= (input - tolerance)) && (input <= (input + tolerance));
    }

}

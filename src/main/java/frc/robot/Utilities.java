package frc.robot;

public class Utilities {

    /** Maps a given set of input ranges to a given set of output ranges 
     * for math to calculate angles to encoder positions
     */
    public static double map(double x, double inputMin, double inputMax, double outputMin, double outputMax) {
        return ((outputMax - outputMin) / (inputMax - inputMin)) * (x - inputMin) + outputMin;
    }

    public static double applyDeadband(double input, double threshold) {
        if (Math.abs(input) < threshold) {
            return 0;
        } else {
            return input;
        }
    }
}

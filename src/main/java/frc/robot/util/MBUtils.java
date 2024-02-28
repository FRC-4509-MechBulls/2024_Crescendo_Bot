package frc.robot.util;

import java.util.Optional;

public class MBUtils {

    public static double lerp(double value1, double value2, double t) { //linear interpolation!! (for shrimp)
        return (1 - t) * value1 + t * value2;
    }

    public static double slerp(double a, double b, double t) { //spherical interpolation (yum!)
        // Normalize angles
        double delta = ((b - a) + Math.PI) % (2 * Math.PI) - Math.PI;

        // Make sure we interpolate in the short direction
        if (delta > Math.PI) {
            delta -= 2 * Math.PI;
        } else if (delta < -Math.PI) {
            delta += 2 * Math.PI;
        }

        return a + delta * t;
    }


    public static double clamp(double input, double absMax){
        absMax = Math.abs(absMax);

        if(input>absMax)
            input = absMax;
        if(input<-absMax)
            input = -absMax;

        return input;
    }

    public static double angleDiffDeg(double ang1, double ang2){
        if(Math.abs(ang1-ang2)>180){
            if(ang1>ang2)
                return -ang1+ang2+360;
            else
                return -ang1+ang2-360;
        }
        return ang2-ang1;
    }

    public static double angleDiffRad(double ang1, double ang2){
        double rawDiff = ang2 - ang1;
        // Normalize the difference to [-π, π]
        //  Adjust if the difference is greater than π
        rawDiff = (rawDiff + Math.PI) % (2 * Math.PI) - Math.PI;
        //  Handle the case where rawDiff becomes -π by adding 2π, ensuring it stays within the desired range
        if (rawDiff < -Math.PI)
            rawDiff += 2 * Math.PI;
        return rawDiff;
    }




    public static double clamp(double input, double min, double max){
        if(input>max) input = max;
        if(input<min) input = min;
        return input;
    }

  //  public static double easeInOut(double t) {
  //      return t < 0.5 ? 2 * t * t : -1 + (4 - 2 * t) * t;
  //  }

    public static double easeInOut(double t, double a) {
        if (a < 1) {
            a = 1; // Prevents division by zero and maintains at least the original curve's shape
        }

        if (t < 0.5) {
            return Math.pow(2 * t, a) / 2;
        } else {
            return (2 - Math.pow(2 * (1 - t), a)) / 2;
        }
    }



    public static double easeInOut(double value1, double value2, double t, double aggression){
        return lerp(value1,value2,easeInOut(t,aggression));
    }

    public static double easeInOutSlerp(double value1, double value2, double t, double aggression){ //sus!
        return slerp(value1, value2, easeInOut(t, aggression));
    }

    public static Optional<Double> interpolate(double[] xValues, double[] yValues, double x) {
        // Ensure the arrays have the same length
        if (xValues.length != yValues.length) {
            throw new IllegalArgumentException("xValues and yValues must have the same length.");
        }

        int indexBelow = -1;
        int indexAbove = -1;

        // Find the two indices where x lies in between
        for (int i = 0; i < xValues.length - 1; i++) {
            if (x >= xValues[i] && x <= xValues[i + 1]) {
                indexBelow = i;
                indexAbove = i + 1;
                break;
            }
        }

        //If indices found, perform interpolation. Otherwise, return an empty Optional.
        if (indexBelow != -1) {
            double x1 = xValues[indexBelow];
            double x2 = xValues[indexAbove];
            double y1 = yValues[indexBelow];
            double y2 = yValues[indexAbove];

            double interpolatedValue = y1 + ((x - x1) / (x2 - x1)) * (y2 - y1);
            return Optional.of(interpolatedValue);
        } else {
            return Optional.empty();
        }
    }



}

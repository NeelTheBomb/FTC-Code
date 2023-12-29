package org.firstinspires.ftc.teamcode.lib;
import java.util.*;


public class MathStuff {
    public static double remapRange(double old_min, double old_max, double new_min, double new_max, double value) {
        return new_min + (value - old_min) * ((new_max - new_min) / (old_max - old_min));
    }
    
    
    public static double clamp(double min, double max, double value) {
        if (value > max) { return max; }
        if (value < min) { return min; }
        return value;
    }
    
    
    public static int sqr(int x) {
        return x * x;
    }
    
    
    public static double sqr(double x) {
        return x * x;
    }
    
    
    public static double findMedian(int a[], int n)
    {
        // First we sort the array
        Arrays.sort(a);
 
        // check for even case
        if (n % 2 != 0)
            return (double)a[n / 2];
 
        return (double)(a[(n - 1) / 2] + a[n / 2]) / 2.0;
    }
    
    
    public static int smallestDiff(int nums[], int value) {
        int smallestDifference = 250;
        int smallestValue = nums[0];
        for (int i = 0; i < nums.length; i++) {
            int difference = Math.abs(nums[i] - value);
            if (difference < smallestDifference) {
                smallestDifference = difference;
                smallestValue = nums[i];
            }
        }
        return smallestValue;
    }
    

    public static double shortestAngleRemapped(double currentAngle, double targetAngle) {
        // convert from radians to degrees
        double target_angle = (targetAngle)*(Math.PI/180);
        
        double differenceOne = target_angle - currentAngle;
        double differenceTwo = target_angle + currentAngle;
        
        if (differenceTwo - differenceOne < 0) {
             return remapRange(-180.0, 180.0, -1.0, 1.0, differenceTwo);
        }
        else {
            return remapRange(-180.0, 180.0, -1.0, 1.0, differenceOne);
        }
    }
}

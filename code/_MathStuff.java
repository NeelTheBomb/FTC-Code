package org.firstinspires.ftc.teamcode.Event2023;


public class MathStuff {
    public static double remap_range(double old_min, double old_max, double new_min, 
            double new_max, double value) {
        
        return new_min + (value - old_min) * ((new_max - new_min) 
                / (old_max - old_min));
    }
    
    
    public static double clamp(double min, double max, double value) {
        if (value > max) {value = max;}
        if (value < min) {value = min;}
        return value;
    }
}

package code;

public class IK {
    private double[] getTargetAngles(double targetX, double targetY, double armLength1, double armLength2) {
        double num = (targetX * targetX) + (targetY * targetY) - (armLength1 * armLength1) - (armLength2 * armLength2);
        double den = 2 * armLength1 * armLength2;
        double arm2TargetAngle = Math.acos(num / den);
            
        num = armLength2 * Math.sin(arm2TargetAngle);
        den = armLength1 + (armLength2 * Math.cos(arm2TargetAngle));
        double arm1TargetAngle = Math.atan(targetY / targetX) - Math.atan(num / den);
        
        double[] angles = {arm1TargetAngle, arm2TargetAngle};
        return angles;
    }
}

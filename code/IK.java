public class IK {
    private static final double armLength1 = 40; // cm, roughly
    private static final double armLength2 = 35; // cm, roughly
    
    private double[] getTargetAngles(double targetX, double targetY) {
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

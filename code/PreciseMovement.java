package org.firstinspires.ftc.teamcode.lib;

// first
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// our stuff
import org.firstinspires.ftc.teamcode.lib.Vector;
import org.firstinspires.ftc.teamcode.lib.MathStuff;

// our static stuff
import static org.firstinspires.ftc.teamcode.lib.MathStuff.sqr;
import static org.firstinspires.ftc.teamcode.lib.MathStuff.shortestAngleRemapped;


public class PreciseMovement {
    private static DcMotorEx fr, fl, br, bl, opv, oph;
    private static BNO055IMU imu;

    // MovementPID
    private static double movementKp, movementKi, movementKd, movementIntegralPrior, movementErrorPrior;

    // AnglePID
    private static double angleKp, angleKi, angleKd, angleIntegralPrior, angleErrorPrior, movementTolerance, angleTolerance;

    // Positions and Motor Ticks
    private static double xPos, yPos, flPastTick, frPastTick, blPastTick, brPastTick, opvPastTick, ophPastTick;


    /******* PUBLIC FUNCTIONS ********/
    // all motors run forward
    public static void init(BNO055IMU imu, DcMotorEx fr, DcMotorEx fl, 
            DcMotorEx br, DcMotorEx bl, DcMotorEx deadVertical, DcMotorEx deadHorizontal) {

        _init(imu, deadVertical, deadHorizontal, fr, fl, br, bl, false, false, false, false);
    }
    
    // Specify which motors to reverse
    public static void init(BNO055IMU imu, DcMotorEx fr, DcMotorEx fl, 
            DcMotorEx br, DcMotorEx bl, boolean frRev, boolean flRev, 
            boolean brRev, boolean blRev) {
                
        _init(imu, opv, oph, fr, fl, br, bl, frRev, flRev, brRev, blRev);
    }


    /******* PUBLIC FUNCTIONS ********/
    public static boolean moveToPos(double targetPosX, double targetPosY, double iterationTime) {
        double[] moveConstants = desiredVector(xPos, yPos, targetPosX, targetPosY, iterationTime);

        robotMove(moveConstants[0], moveConstants[1], 0);

        return distanceFromPos(targetPosX, targetPosY) < 10;
    }


    public static boolean moveToAngle(double targetAngle, double iterationTime) {
        double[] angleValues = PID(angleKp, angleKd, angleKi, -imu.getAngularOrientation().firstAngle, targetAngle,
                angleIntegralPrior, angleErrorPrior, iterationTime);

        double angleDifference = angleValues[0];
        angleErrorPrior = angleValues[1];
        angleIntegralPrior = angleValues[2];
        robotMove(0, 0, angleDifference);

        return Math.abs(shortestAngleRemapped(currentAngle(), targetAngle)) < angleTolerance;
    }


    public static void updatePos() {
        double flTick = fl.getCurrentPosition() - flPastTick;
        double frTick = fr.getCurrentPosition() - frPastTick;
        double blTick = bl.getCurrentPosition() - blPastTick;
        double brTick = br.getCurrentPosition() - brPastTick;

        Vector currentVector = finalWheelVector(flTick, frTick, blTick, brTick, -imu.getAngularOrientation().firstAngle);
        double[] pos = newPosition(xPos, yPos, currentVector);
        xPos = pos[0];
        yPos = pos[1];

        // remember past encoder values
        flPastTick = fl.getCurrentPosition();
        frPastTick = fr.getCurrentPosition();
        blPastTick = bl.getCurrentPosition();
        brPastTick = br.getCurrentPosition();
    }
    
    
    
    public static double[] updateDeadWheelPos() {
        double ophTick = oph.getCurrentPosition() - ophPastTick;
        double opvTick = opv.getCurrentPosition() - opvPastTick;
        
        Vector currentVector = finalDeadWheelVector(ophTick, opvTick, -imu.getAngularOrientation().firstAngle);
        double[] pos = newPosition(xPos, yPos, currentVector);
        xPos = pos[0];
        yPos = pos[1];

        // remember past encoder values
        ophPastTick = oph.getCurrentPosition();
        opvPastTick = opv.getCurrentPosition();
        return new double[] {ophTick, opvTick, opv.getCurrentPosition(), oph.getCurrentPosition()};
    }
    
    
    private static Vector finalDeadWheelVector(double deltaOph, double deltaOpv, double robotAngle) {
        Vector ophVector = new Vector(deltaOph, Math.PI + robotAngle);
        Vector opvVector = new Vector(deltaOpv, Math.PI/2 + robotAngle);

        Vector[] deadWheelVectors = new Vector[] {ophVector, opvVector};
        // just using a random vector to use the add function in the vector class
        Vector finalVector = ophVector.add(deadWheelVectors);
        return new Vector(finalVector.getMagnitude(), finalVector.getAngle() + robotAngle);
    }


    public static double distanceFromPos(double targetPosX, double targetPosY) {
        return Math.sqrt(sqr(targetPosX-xPos) + sqr(targetPosY-yPos));
    }


    /******* GETTERS ********/
    public static double[] getPos() { return new double[] { xPos, yPos }; }


    /******* SETTERS ********/
    public static void setMovementTolerance(double t) { movementTolerance = t; }

    public static void setAngleTolerance(double t) { angleTolerance = t; }

    public static void setMovementPID(double kp, double kd, double ki) {
        movementKp = kp;
        movementKd = kd;
        movementKi = ki;
        movementIntegralPrior = 0;
        movementErrorPrior = 0;
    }

    public static void setAnglePID(double Kp, double Kd, double Ki) {
        angleKp = Kp;
        angleKd = Kd;
        angleKi = Ki;
        angleIntegralPrior = 0;
        angleErrorPrior = 0;
    }


    /******* PRIVATE FUNCTIONS ********/
    private static void _init(BNO055IMU imu, DcMotorEx fr, DcMotorEx fl, 
            DcMotorEx br, DcMotorEx bl, DcMotorEx opv, DcMotorEx oph, 
            boolean frRev, boolean flRev, boolean brRev, boolean blRev) {
                
        PreciseMovement.imu = imu;
        
        PreciseMovement.oph = oph;
        PreciseMovement.opv = opv;
        
        PreciseMovement.fr = fr;
        PreciseMovement.fl = fl;
        PreciseMovement.br = br;
        PreciseMovement.bl = bl;
        
        if (frRev) { fr.setDirection(DcMotor.Direction.REVERSE); }
        if (flRev) { fl.setDirection(DcMotor.Direction.REVERSE); }
        if (brRev) { br.setDirection(DcMotor.Direction.REVERSE); }
        if (blRev) { bl.setDirection(DcMotor.Direction.REVERSE); }

        PreciseMovement.movementKp = 0.5;
        PreciseMovement.movementKi = 0.0;
        PreciseMovement.movementKd = 0.0;
        PreciseMovement.movementIntegralPrior = 0.0;
        PreciseMovement.movementErrorPrior = 0.0;

        PreciseMovement.angleKp = 0.5;
        PreciseMovement.angleKi = 0.0;
        PreciseMovement.angleKd = 0.0;
        PreciseMovement.angleIntegralPrior = 0.0;
        PreciseMovement.angleErrorPrior = 0.0;

        PreciseMovement.movementTolerance = 10.0;
        PreciseMovement.angleTolerance = 0.05;

        PreciseMovement.xPos = 0.0;
        PreciseMovement.yPos = 0.0;
        PreciseMovement.flPastTick = 0.0;
        PreciseMovement.frPastTick = 0.0;
        PreciseMovement.blPastTick = 0.0;
        PreciseMovement.brPastTick = 0.0;
    }


    private static Vector finalWheelVector(double deltaFl, double deltaFr, double deltaBl, double deltaBr, double robotAngle) {
        Vector flvector = new Vector(deltaFl, Math.PI*0.25 + robotAngle);
        Vector frvector = new Vector(deltaFr, Math.PI*0.75 + robotAngle);
        Vector blvector = new Vector(deltaBl, Math.PI*0.75 + robotAngle);
        Vector brvector = new Vector(deltaBr, Math.PI*0.25 + robotAngle);

        Vector[] wheelVectors = new Vector[] { flvector, frvector, blvector, brvector };
        // just using a random vector to use the add function in the vector class
        Vector finalVector = blvector.add(wheelVectors);
        return new Vector(finalVector.getMagnitude(), finalVector.getAngle() + robotAngle);
    }
    


    private static double[] newPosition(double oldX, double oldY, Vector finalVector) {
        return new double[] { oldX + finalVector.getX(), oldY + finalVector.getY() };
    }


    // This will return an x and y value that describes how much the wheels should move
    // Think of this as the x and y of the joystick on last year's movement code
    // Divide constant adjusts how big the vector will be according to how faraway the desired position is
    // iterationTime should be in milliseconds
    private static double[] desiredVector(double currentX, double currentY, double desiredX, double desiredY, double iterationTime) {
        double targetDistance = Math.sqrt(sqr(desiredX-currentX) * sqr(desiredY-currentY));
        double angle = Math.atan2((desiredX-currentX), (desiredY-currentY));

        double[] wheelValues = PID(movementKp, movementKd, movementKi, 0, targetDistance,
                movementIntegralPrior, movementErrorPrior, iterationTime);

        movementErrorPrior = wheelValues[1];
        movementIntegralPrior = wheelValues[2];

        return new double[] { Math.cos(angle) * wheelValues[0], Math.sin(angle) * wheelValues[0] };
    }


    private static void robotMove(double rotX, double rotY, double rx) {
        final double MAX_SPEED = 0.75;

        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = -imu.getAngularOrientation().firstAngle;

        rotX = (rotX * Math.cos(botHeading) - rotY * Math.sin(botHeading)) / 2;
        rotY = (rotX * Math.sin(botHeading) + rotY * Math.cos(botHeading)) / 2;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + (rx), 1);
        double frontLeftPower = (rotY + rotX - rx) / denominator;
        double backLeftPower = (rotY - rotX - rx) / denominator;
        double frontRightPower = (rotY - rotX + rx) / denominator;
        double backRightPower = (rotY + rotX  + rx) / denominator;

        fl.setPower(frontLeftPower * MAX_SPEED);
        bl.setPower(backLeftPower * MAX_SPEED);
        fr.setPower(frontRightPower * MAX_SPEED);
        br.setPower(backRightPower * MAX_SPEED);
    }


    private static double[] PID(double kp, double kd, double ki, double actualValue, double desiredValue,
                         double integralPrior, double errorPrior, double iterationTime) {

        double error = desiredValue - actualValue;
        double integral = integralPrior + error * iterationTime;
        double derivative = (error - errorPrior) / iterationTime;
        double output = kp*error + ki*integral + kd*derivative;

        return new double[] { output, error, integral };
    }
    
    
    private static double currentAngle() {
        return -imu.getAngularOrientation().firstAngle;
    }
}

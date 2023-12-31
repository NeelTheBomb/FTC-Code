package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="2023TeleOp", group="Linear Opmode")
public class Teleop2023 extends LinearOpMode {
    private Servo gripper = null;
    private DcMotorEx fl = null;
    private DcMotorEx bl = null;
    private DcMotorEx fr = null;
    private DcMotorEx br = null;

    private double maxSpeed = 1;
    private double maxRotationSpeed = 1;
    private double turnSpeed = 1;
    
    private double newLeftStickx = 0;
    private double newLeftSticky = 0;
    private double newRightSticky = 0;
    private double newRightStickx = 0;
    
    private DcMotorEx uArm;
    private DcMotorEx lArm;
    
    private static final double armLength1 = 14.5; // in
    private static final double armLength2 = 12.5; // in
    
    private final double armVelocity = 5.0;
    
    private double armTargetX = 0;
    private double armTargetY = 27;
    
    private double lastValidArmTargetX;
    private double lastValidArmTargetY;
    
    double rx = 0;

    ElapsedTime t = new ElapsedTime();


    @Override
    public void runOpMode() {
        // Declare our motors
        // Make sure your ID's match your configuration
        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        br = hardwareMap.get(DcMotorEx.class, "backRight");
        
        lArm = hardwareMap.get(DcMotorEx.class, "lArm");
        uArm = hardwareMap.get(DcMotorEx.class, "uArm");
        
        lArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        uArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "gyro");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
        // armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        
        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            //these control speed of robot
            turnSpeed = 1;
            maxSpeed = 1;
            
            double y = gamepad1.left_stick_y * .9; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
 
            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle;
            telemetry.addData("botheading", Math.toDegrees(botHeading));

            double rotX = (x * Math.cos(botHeading) - y * Math.sin(botHeading)) / 2;
            double rotY = (x * Math.sin(botHeading) + y * Math.cos(botHeading)) / 2;
            double rx = 0;
            
            if (gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0) {
                telemetry.addData("works", "works");
                
                rx = -2 * sticksAndImuToTurn(gamepad1.right_stick_x, 
                        gamepad1.right_stick_y, 
                        -imu.getAngularOrientation().firstAngle); 
            }
                
            //rx = (newRightStickx * Math.cos(botHeading) - newRightSticky * Math.sin(botHeading));
            telemetry.addData("angular", rx);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + (rx), 1);
            double frontLeftPower = (rotY + rotX - rx) / denominator;
            double backLeftPower = (rotY - rotX - rx) / denominator;
            double frontRightPower = (rotY - rotX + rx) / denominator;
            double backRightPower = (rotY + rotX  + rx) / denominator;

            fl.setPower(frontLeftPower * maxSpeed);
            bl.setPower(backLeftPower * maxSpeed);
            fr.setPower(frontRightPower * maxSpeed);
            br.setPower(backRightPower * maxSpeed);
            
            // arm
            if (gamepad2.left_stick_y > 0.2 || gamepad2.left_stick_y < -0.2) {
                armTargetX -= gamepad2.left_stick_y*.04;
            }
            if (gamepad2.right_stick_y > 0.2 || gamepad2.right_stick_y < -0.2) {
                armTargetY -= gamepad2.right_stick_y*.04;
            }
            
            if (armTargetX < -(armLength1+armLength2)) {
                armTargetX = -(armLength1+armLength2);
            }
            else if (armTargetX > (armLength1+armLength2)) {
                armTargetX = (armLength1+armLength2);
            }
            if (armTargetY < -(armLength1+armLength2)) {
                armTargetY = -(armLength1+armLength2);
            }
            else if (armTargetY > (armLength1+armLength2)) {
                armTargetY = (armLength1+armLength2);
            }
            
            moveArm(armTargetX, armTargetY);
            telemetry.addData("realx", armTargetX);
            
            telemetry.update();
        }
    }
    
    
    public double stickToDegrees(float x,float y) {
        //just saving the values
        float xsave = x;
        float ysave = y;
            
        //swapping x and y
        x = ysave;
        y = xsave;
        //y = -y;
        x = -x;
        double angle = Math.atan2(y,x);
        return angle;
    }
        
            
    public double angleDifference(double actual, double desired) {  //TODO: rewrite, sucks ass, bad
        /* if (actual > 180) { 
            actual = actual-360;
        } */
        
        double angle = actual - desired;
        double primary = angle;
        double secondary = (2*Math.PI) - Math.abs(angle);
        
        if (Math.abs(angle) < secondary) {
            primary = angle; 
        }
        if (secondary < Math.abs(angle)) {
            primary = secondary;
        }
        
        return primary;
    }
        
        
    public double remap_range(double old_min, double old_max, double new_min, 
            double new_max, double value) {
        
        return new_min + (value - old_min) * ((new_max - new_min) 
                / (old_max - old_min));
    }
        
        
    public double sticksAndImuToTurn(float x,float y, double botHeading) {
        double angle = stickToDegrees(x, y);
        double angleFromRobot = angleDifference(botHeading, angle);
        
        return remap_range(-2 * Math.PI, 2 * Math.PI, -1, 1, angleFromRobot);
    }
        
        
    private double[] getTargetArmAngles(double targetX, double targetY) {
        double num = (targetX * targetX) + (targetY * targetY) 
                - (armLength1 * armLength1) - (armLength2 * armLength2);
                
        double den = 2 * armLength1 * armLength2;
        
        double arm2TargetAngle = Math.acos(num / den);
            
        num = armLength2 * Math.sin(arm2TargetAngle);
        den = armLength1 + (armLength2 * Math.cos(arm2TargetAngle));
        
        double arm1TargetAngle = Math.atan(targetY / targetX) 
                - Math.atan(num / den);
        
        double[] angles = { arm1TargetAngle, arm2TargetAngle };
        return angles;
    }
    
    
    private void moveArm(double targetX, double targetY) {
        double lArmCoefficient = 4.72;
        double uArmCoefficient = 4.72;
        
        double[] targetAngles = getTargetArmAngles(targetX, targetY);
        
        if (!Double.isNaN(targetAngles[0]) && !Double.isNaN(targetAngles[1])) {
            lastValidArmTargetX = targetX;
            lastValidArmTargetY = targetY;
        }
        else {
            targetX = lastValidArmTargetX;
            targetY = lastValidArmTargetY;
            armTargetX = targetX;
            armTargetY = targetY;
            targetAngles = getTargetArmAngles(targetX, targetY);
            telemetry.addData("NAN", "NAN");
        }
        
        telemetry.addData("targetX", targetX);
        telemetry.addData("targetY", targetY);
        telemetry.addData("larm angle", targetAngles[0]  * lArmCoefficient);
        telemetry.addData("uarm angle", targetAngles[1]  * lArmCoefficient);
        telemetry.addData("larm tick", lArm.getCurrentPosition());
        telemetry.addData("uarm tick", uArm.getCurrentPosition());
        telemetry.addData("StickX", gamepad2.right_stick_x);
        telemetry.addData("StickY", gamepad2.right_stick_y);
    }
}

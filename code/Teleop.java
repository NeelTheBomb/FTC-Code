package org.firstinspires.ftc.teamcode;

// first
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
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

// our classes
import org.firstinspires.ftc.teamcode.lib.PreciseMovement;

// our static functions
import static org.firstinspires.ftc.teamcode.lib.MathStuff.remapRange;
import static org.firstinspires.ftc.teamcode.lib.MathStuff.shortestAngleRemapped;
import static org.firstinspires.ftc.teamcode.lib.MathStuff.sqr;


@TeleOp(name="Teleop", group="Linear Opmode")
public class Teleop extends LinearOpMode {
    // initialize hardware variables
    private DcMotorEx fl = null;
    private DcMotorEx bl = null;
    private DcMotorEx fr = null;
    private DcMotorEx br = null;
    private DcMotorEx slide = null;
    private DcMotorEx tIntake = null;
    private DcMotorEx sIntake = null;
    private DcMotorEx arm = null;
    private Servo drone = null;
    private Servo lGripper = null;
    private Servo rGripper = null;
    // private Servo gripper = null;
    private BNO055IMU imu = null;
    
    private double rxMultiplier = 0.9;

    @Override
    public void runOpMode() {
        // Declare our motors
        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");      // front left
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");       // back left
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");     // front right
        br = hardwareMap.get(DcMotorEx.class, "backRight");      // back right
        // slide = hardwareMap.get(DcMotorEx.class, "slide");       // slide
        // tIntake = hardwareMap.get(DcMotorEx.class, "tIntake");   // torque intake
        // sIntake = hardwareMap.get(DcMotorEx.class, "sIntake");   // speed intake
        // arm = hardwareMap.get(DcMotorEx.class, "arm");           // arm
        // drone = hardwareMap.get(Servo.class, "drone");           // drone
        // lGripper = hardwareMap.get(Servo.class, "gripperLeft");  // left gripper    
        // rGripper = hardwareMap.get(Servo.class, "gripperRight"); // right gripper
        // gripper = hardwareMap.get(Servo.class, "gripper");
        
        // reset encoders
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // // set zeroPowerBehavior
        // slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // reverse stuff
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        // slide.setDirection(DcMotor.Direction.REVERSE);
        
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "gyro");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
        
        // wait until init is pressed
        waitForStart();
        
        while (opModeIsActive()) {
            move();
            // slide();
            // arm();
            // drone();
            // gripper();
            
            telemetry.update();
        }
    }
    
    
    private void move() {
        // Changing encoder mode
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double maxSpeed = 0.7;
        
                
        
        double botHeading = -(imu.getAngularOrientation().firstAngle);
        
        // old turning
        double rx = ((gamepad1.right_stick_y) * Math.sin(botHeading) + (gamepad1.right_stick_x) *Math.cos(botHeading));

        //  double rx = 0.0;
        //  if (Math.abs(gamepad1.right_stick_x) >= 0.1 || Math.abs(gamepad1.right_stick_y) >= Math.abs(0.1)) {
        //      rx = shortestAngleRemapped(botHeading, Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x));
        //      rx *= rxMultiplier;
        //  }
        
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double y = -gamepad1.left_stick_y * .9; // Remember, this is reversed!
        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        // double rotX = (x * Math.cos(botHeading) - y * Math.sin(botHeading)) / 2;
        // double rotY = (x * Math.sin(botHeading) + y * Math.cos(botHeading)) / 2;
        // telemetry.addData("angle", botHeading);

        //  telemetry.addData("maththing", Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x));
        //  telemetry.addData("rx", rx);
        
        // double denominator = Math.max(Math.abs(y) + Math.abs(x) + (rx), 1);
        // double frontLeftPower = (rotY + rotX - rx) / denominator;
        // double backLeftPower = (rotY - rotX - rx) / denominator;
        // double frontRightPower = (rotY - rotX + rx) / denominator;
        // double backRightPower = (rotY + rotX  + rx) / denominator;
        
        
        theta += botHeading;
        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));
        
        double frontLeftPower = power * cos/max + rx;
        double frontRightPower = power * sin/max - rx;
        double backLeftPower = power * sin/max + rx;
        double backRightPower = power * cos/max - rx;
        
        if ((power + Math.abs(rx)) > 1){
            frontLeftPower /= power + rx;
            frontRightPower /= power + rx;
            backLeftPower /= power + rx;
            backRightPower /= power + rx;
        }
        
        

        fl.setPower(frontLeftPower * maxSpeed);
        bl.setPower(backLeftPower * maxSpeed);
        fr.setPower(frontRightPower * maxSpeed);
        br.setPower(backRightPower * maxSpeed);
    }

    
    private void slide() {
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        final int LOWEST_POSITION = 0;
        final int HIGHEST_POSITION = 2500;
        
        // left trigger down, right trigger up
        double triggerInput = gamepad2.right_trigger - gamepad2.left_trigger;
        double currentPosition = slide.getCurrentPosition();
        double remapControl = 1.5;
        
        if (currentPosition < LOWEST_POSITION) {
            slide.setPower(.1);
        }
        else if (currentPosition > HIGHEST_POSITION) {
            slide.setPower(-.1);
        }
        else if (triggerInput > .2) {
        // if either trigger pressed
            double powerControl = remapRange(LOWEST_POSITION, HIGHEST_POSITION, 0, remapControl, currentPosition);
            powerControl = 1/Math.pow(Math.abs(powerControl),3);
            slide.setPower(powerControl*triggerInput);
        }
        else if (triggerInput < -.2) {
            double powerControl = remapRange(HIGHEST_POSITION, LOWEST_POSITION-20, 0, remapControl, currentPosition);
            powerControl = 1/Math.pow(Math.abs(powerControl),3);
            slide.setPower(powerControl*triggerInput);
        }
        else {
            slide.setPower(0);
        }
        
        telemetry.addData("currentPosition", currentPosition);
    }
    
    
    private void arm() {
        if (gamepad2.right_bumper && arm.getPower() != 1.0) {
            arm.setPower(1.0);
        }
        else if (gamepad2.left_bumper && arm.getPower() != -1.0) {
            arm.setPower(-1.0);
        }
        else if (arm.getPower() != 0.0) {
            arm.setPower(0.0);
        }
    }
    
    
    private void drone() {
        if (gamepad2.y) {
            drone.setPosition(.9);
        }
        else {
            drone.setPosition(.5);
        }
    }
    
    
    private void gripper() {
        if (gamepad2.x) {
            lGripper.setPosition(.2);
            rGripper.setPosition(.2);
        }
        else {
            lGripper.setPosition(.08);
            rGripper.setPosition(.3);
        }
        
        // if (gamepad2.x) {
        //     gripper.setPosition(.6);
        // }
        // else {
        //     gripper.setPosition(1.6);
        // }
    }
}

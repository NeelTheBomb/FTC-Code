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

@TeleOp(name="VIRAAJ IS BETTER THAN NEEL", group="Linear Opmode")
public class ViraajW extends LinearOpMode {
    Servo gripper = null;
    DcMotorEx fl = null;
    DcMotorEx bl = null;
    DcMotorEx fr = null;
    DcMotorEx br = null;
    DcMotorEx armMotor = null;
    double maxSpeed = 1;
    double maxRotationSpeed = 1;
    double turnSpeed = 1;
    double newLeftStickx = 0;
    double newLeftSticky = 0;
    double newRightSticky = 0;
    double newRightStickx = 0;
    // s
    double backLeftSave = 0;
    double backRightSave = 0;
    double frontRightSave = 0;
    double frontLeftSave = 0;
    double currentX = 0;
    double currentY = 0;
    // stuff
    double rx = 0;
    double limit = 0;
    double level = 0;
    static final double COUNTS_PER_MOTOR_REV = 288; 
    static final double GEAR_REDUCTION = 2.7778;   
    static final double COUNTS_PER_GEAR_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION;
    static final double COUNTS_PER_DEGREE = COUNTS_PER_GEAR_REV/360;
    static final int newarmpos = 4485;
    static final int oldarmmaxpos = 3820;
    ElapsedTime t = new ElapsedTime();
    int height = 0;
    private boolean prevY = false;
    private boolean prevA = false;
    @Override

    public void runOpMode(){
        // Declare our motors
        // Make sure your ID's match your configuration
        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        br = hardwareMap.get(DcMotorEx.class, "backRight");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gripper = hardwareMap.get(Servo.class, "gripperServo1");
        
        int minPosition = armMotor.getCurrentPosition();
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
        double maxPosition = armMotor.getCurrentPosition();
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            int NewPos = armMotor.getCurrentPosition();
            //these control speed of robot
            turnSpeed = 1;
            maxSpeed = 1;
            
            
            // double deltaFL = fl.getCurrentPosition() - frontLeftSave;
            // double deltaBL = bl.getCurrentPosition() - backLeftSave;
            // double deltaFR = fr.getCurrentPosition() - frontRightSave;
            // double deltaBR = br.getCurrentPosition() - backRightSave;
            
            // double deltaY = Math.PI * 3.77953 * (deltaFL + deltaBL + deltaFR + deltaBR) / (4 * 537.6);
            // double deltaX = Math.PI * 3.77953 * (-deltaFL + deltaBL + deltaFR - deltaBR) / (4 * 537.6);
            // double bh = imu.getAngularOrientation().firstAngle;
            // double rY = deltaY * Math.cos(bh) - deltaX * Math.sin(bh);
            // double rX = deltaY * Math.sin(bh) + deltaX * Math.cos(bh);
            // currentX += rX;
            // currentY += rY;
            // frontLeftSave = fl.getCurrentPosition();
            // backLeftSave = bl.getCurrentPosition();
            // frontRightSave = fr.getCurrentPosition();
            // backRightSave = br.getCurrentPosition();
            
            // telemetry.addData("x pos", currentX);
            // telemetry.addData("y pos", currentY);
            
            //makes you move perfect 90 if you click b
            if (gamepad1.b) {
            if (Math.abs(gamepad1.left_stick_x) > Math.abs(gamepad1.left_stick_y)) {
                newLeftStickx = gamepad1.left_stick_x;
                newLeftSticky = 0;
            }
            if (Math.abs(gamepad1.left_stick_y) > Math.abs(gamepad1.left_stick_x)) {
                newLeftStickx = 0;
                newLeftSticky = gamepad1.left_stick_y;
            }
            }
            else {
            newLeftStickx = gamepad1.left_stick_x * .7;
            newLeftSticky = gamepad1.left_stick_y * 1.15;
            }
            
            double y = newLeftSticky * .9; // Remember, this is reversed!
            double x = -newLeftStickx * 1.1 * .9; // Counteract imperfect strafing
            newRightStickx = gamepad1.right_stick_x;
            newRightSticky = gamepad1.right_stick_y;

            if (gamepad1.dpad_down) {newRightSticky = 1; turnSpeed = 2;}
            if (gamepad1.dpad_up) {newRightSticky = -1; turnSpeed = 2;}
            if (gamepad1.dpad_right) {newLeftStickx = 1; turnSpeed = 2;}
            if (gamepad1.dpad_left) {newLeftStickx = -1; turnSpeed = 2;}
            if (gamepad1.left_bumper) maxSpeed = .5;
            if (gamepad1.x) {newLeftStickx = .3; newRightSticky = .5;}
            if (gamepad1.y) { parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                            imu.initialize(parameters);}
            //if (gamepad1.left_trigger > .2) armMotor.setPower(-.1);
            //if (gamepad1.right_trigger > .2) armMotor.setPower(.1);
            if (gamepad2.y && gamepad2.a) maxPosition = 0;
            if (gamepad2.left_bumper) maxPosition = maxPosition - 5;
            if (gamepad2.right_bumper) maxPosition = maxPosition + 5;
          
            //makes the arm stop if no triggers are being clicked                
            if(gamepad2.right_trigger <.1 && gamepad2.left_trigger < .1)
            {
                armMotor.setPower(0);
                
            }
           //makes the arm go down when left trigger  clicked
           if (gamepad2.left_trigger > .1){
            if (armMotor.getCurrentPosition() > maxPosition) {
                double pow = -gamepad2.left_trigger*((armMotor.getCurrentPosition() - maxPosition)+.2)*.0003;
                double recorrect = Math.min(pow, -.5);
                armMotor.setPower(recorrect);
                //armMotor.setPower(-gamepad2.left_trigger*((armMotor.getCurrentPosition() - maxPosition)+.3)*.0003);
                //armMotor.setPower(-gamepad2.left_trigger);
            }}
            //makes the arm go up if it goes to down
            if (armMotor.getCurrentPosition() < maxPosition) {
                armMotor.setPower(.01);
            }
            if (armMotor.getCurrentPosition() > (maxPosition + 4480)) {
                armMotor.setPower(0);
            } 
            
            //makes the arm go up if right trigger  is cliked
            if (gamepad2.right_trigger > .1) {
            if (armMotor.getCurrentPosition() < (maxPosition + 4480)) {
                armMotor.setPower(gamepad2.right_trigger); }
                //armMotor.setPower(0);
            } 
            
            // if (armMotor.getCurrentPosition() > maxPosition + 3700) {
            //     armMotor.setPower(-.1);
            // }
            if (gamepad2.x == false && gamepad2.b == false) 
            {
                gripper.setPosition(0.35);
            }
            else
            {
                if (gamepad2.b) {
                gripper.setPosition(0.5); }
                if (gamepad2.x) {
                gripper.setPosition(.45); }
            } 
            
            // if (gamepad1.a && NewPos > minPosition && !prevA)
            // {
            //   GoForward(-1000);
            // }
            
            // if (gamepad1.y && NewPos > minPosition && !prevY)
            // {
            //     GoForward(1000);
            // }
            // if (gamepad1.b && (level < 3) && (t.seconds() > .2))
            // {
            //     //armMotor.setPower(gamepad1.right_trigger);
            //     GoForward(400);
            //     level = level +1;
            //     t.reset();
            // }
            // if (gamepad1.y && (level >= 0) && (t.seconds() > .2))
            // {
            //     //armMotor.setPower(-gamepad1.left_trigger*0.5);
            //     GoForward(-20);
            //     level = level - 1;
            //     t.reset();
            // }
            
            // if (armMotor.getCurrentPosition() > 0)
            // {
            //     if (gamepad1.left_trigger > 0 && (level >= 0))
            //     {
            //         //armMotor.setPower(-gamepad1.left_trigger*0.5);
            //         GoForward(-2000);
            //         level = level  - 1;
            //     }
            // }
            
            
            //if (gamepad1.y == true)
            //{
            //    GoForward(1000);
            //    gamepad1.y = false;
            //}
            //if (gamepad1.a == true && armMotor.getCurrentPosition() > 0)
            //{
            //    GoForward(1000);
            //    gamepad1.a = false;
            //}
            prevA = gamepad1.a;
            prevY = gamepad1.y;
            telemetry.addData("ServoPos", gripper.getPosition());
            telemetry.addData("ArmPos: ", armMotor.getCurrentPosition());
            telemetry.addData("level",level );
            telemetry.update();

            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle;
            telemetry.addData("botheading", botHeading);

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading) * .5;
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading) * .5;
            rx = ((newRightSticky*turnSpeed) * Math.sin(botHeading) + (newRightStickx*turnSpeed) *Math.cos(botHeading));
            //rx = (newRightStickx * Math.cos(botHeading) - newRightSticky * Math.sin(botHeading));
            //double ry = (newRightSticky * Math.sin(botHeading) - newRightStickx * Math.cos(botHeading));
            telemetry.addData("angular", ((0) * Math.sin(botHeading) + (1) *Math.cos(botHeading)));
        //     if (Math.abs(rx) >= .2) {
        //   if (rx < 0) rx = -.7;
        //     if (rx > 0) rx = .7; }
            
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + (rx), 1);
            double frontLeftPower = (rotY + rotX - rx) / denominator;
            double backLeftPower = (rotY - rotX - rx) / denominator;
            double frontRightPower = (rotY - rotX + rx) / denominator;
            double backRightPower = (rotY + rotX  + rx) / denominator;

            fl.setPower(frontLeftPower * maxSpeed);
            bl.setPower(backLeftPower * maxSpeed);
            fr.setPower(frontRightPower * maxSpeed);
            br.setPower(backRightPower * maxSpeed);
            
        }
    }
    public void GoForward(int armNum)
    {

        armMotor.setTargetPosition(armNum);
            
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
        armMotor.setVelocity(100000);  // 300 is on regular planetaries
    } 
//     public void GoDown()
//     {
        
//     }
//     public void GoUp()
//     {
        
//     }
}

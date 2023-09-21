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

@TeleOp(name="TurnTest", group="Linear Opmode")

public class UnitTurnGyroExample extends LinearOpMode {
    Servo gripper = null;
    DcMotorEx fl = null;
    DcMotorEx bl = null;
    DcMotorEx fr = null;
    DcMotorEx br = null;
    DcMotorEx armMotor = null;
    

    // todo: write your code here
    
    @Override
     public void runOpMode() {
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "gyro");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
        double angle = stickToDegrees(gamepad1.right_stick_x, gamepad1.right_stick_y);
        double botHeading = -imu.getAngularOrientation().firstAngle;
        double angleFromRobot = angleDifference(botHeading, angle);
        double turn = remap_range(-2*Math.PI, 2*Math.PI, -1, 1, angleFromRobot);
        telemetry.addData("TurnAngle", angle);
        telemetry.addData("x", gamepad1.right_stick_x);
        telemetry.addData("y", gamepad1.right_stick_y);
        telemetry.addData("wanted", angleFromRobot);
        telemetry.addData("turn", turn);
        telemetry.update();
            
        }
         
     }

public double stickToDegrees(float x,float y) {
    //just saving the values
    float xsave=x;
    float ysave=y;
    
    //swapping x and y
    x=ysave;
    y=xsave;
    //y = -y;
    x=-x;
    if (x == 0) {
        x += .0001;
    }
    if (y == 0) {
        y += .0001;
    }
    double angle = Math.atan(y/x);
    if (x < 0 && y >= 0) {
        angle = (Math.PI) + angle; }
    else if (x < 0 && y < 0) {
        angle = Math.PI + angle; }
    else if (x > 0 && y < 0) {
        angle = (Math.PI) * 2 + angle;
    }
    return angle;
    }
    
public double angleDifference(double actual, double desired) {
    double angle = actual - desired;
    double secondary = (2*Math.PI) - Math.abs(angle);
    double primary = angle;
    if (Math.abs(angle) < secondary) {
        primary = angle; }
    if (secondary < Math.abs(angle)) {
        primary = secondary;
    }
    return primary;
}
public double remap_range(double old_min, double old_max, double new_min, double new_max, double value) {
    return new_min + (value - old_min) * ((new_max - new_min) / (old_max - old_min));
}

}

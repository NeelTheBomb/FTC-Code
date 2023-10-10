package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class IKTest extends LinearOpMode {
    private DcMotorEx uArm;
    private DcMotorEx lArm;
    private static final double armLength1 = 49; // cm
    private static final double armLength2 = 32; // cm
    
    final double velocity = 2000.0;
    
    private double lastValidTargetX;
    private double lastValidTargetY;

    @Override
    public void runOpMode() {
        lArm = hardwareMap.get(DcMotorEx.class, "lArm");
        uArm = hardwareMap.get(DcMotorEx.class, "uArm");
        
        lArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        uArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        waitForStart();
        
        double targetX = 0.0;
        double targetY = 20.0;

        while (opModeIsActive()) {
            if (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) {
                targetX -= gamepad1.left_stick_y;
            }
            if (gamepad1.right_stick_y > 0.2 || gamepad1.right_stick_y < -0.2) {
                targetY -= gamepad1.right_stick_y;
            }
            
            moveArm(targetX, targetY);
            
            telemetry.update();
        }
    }
    
    
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
    
    
    private void moveArm(double targetX, double targetY) {
        double lArmCoefficient = 1000;
        double uArmCoefficient = 1000;
        
        double[] targetAngles = getTargetAngles(targetX, targetY);
        
        if (!Double.isNaN(targetAngles[0]) && !Double.isNaN(targetAngles[1])) {
            lastValidTargetX = targetX;
            lastValidTargetY = targetY;
        }
        else {
            targetX = lastValidTargetX;
            targetY = lastValidTargetY;
            targetAngles = getTargetAngles(targetX, targetY);
        }
        
        lArm.setTargetPosition((int) (targetAngles[0] * lArmCoefficient));
        lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lArm.setVelocity(velocity);
        
        uArm.setTargetPosition((int) (targetAngles[1] * uArmCoefficient));
        uArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        uArm.setVelocity(velocity);
        
        telemetry.addData("targetX", targetX);
        telemetry.addData("targetY", targetY);
        telemetry.addData("larm angle", targetAngles[0]);
        telemetry.addData("uarm angle", targetAngles[1]);
        telemetry.addData("larm tick", lArm.getCurrentPosition());
        telemetry.addData("uarm tick", uArm.getCurrentPosition());
    }
}

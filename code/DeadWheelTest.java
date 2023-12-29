package org.firstinspires.ftc.teamcode.CenterStage;

// import static org.firstinspires.ftc.teamcode.lib.MathStuff.findMedian;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.dfrobot.HuskyLens;
// import static org.firstinspires.ftc.teamcode.lib.MathStuff.smallestDiff;
import org.firstinspires.ftc.teamcode.lib.Vector;
import org.firstinspires.ftc.teamcode.lib.PreciseMovement;

import java.util.Queue;

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
import com.qualcomm.hardware.dfrobot.HuskyLens;


@Autonomous
public class DeadWheelTest extends LinearOpMode {
  
  HuskyLens husky;
    private DcMotorEx fl = null;
    private DcMotorEx bl = null;
    private DcMotorEx fr = null;
    private DcMotorEx br = null;
    private DcMotorEx opv = null;
    private DcMotorEx oph = null;
    private BNO055IMU imu = null;
    private DcMotorEx intake = null;
    private Servo gripper = null;
    
    private double pastX = 0.0;
    private double pastY = 0.0;
    
    //public past encoder values
    private double flPastTick = 0.0;
    private double frPastTick = 0.0;
    private double blPastTick = 0.0;
    private double brPastTick = 0.0;
    
    //steps
    public int step = 10;
    
    // gripper
    private boolean gripperToggled = false;
    
    public void runOpMode() {
        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        br = hardwareMap.get(DcMotorEx.class, "backRight");
        oph = hardwareMap.get(DcMotorEx.class, "frontLeft");
        opv = hardwareMap.get(DcMotorEx.class, "backLeft");
        // husky = hardwareMap.get(HuskyLens.class, "husky");
        // gripper = hardwareMap.get(Servo.class, "gripper");
        
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        br.setDirection(DcMotor.Direction.REVERSE);
        
        imu = hardwareMap.get(BNO055IMU.class, "gyro");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
        
        // INITIALIZING PRECISE MOVEMENT
        PreciseMovement.init(imu, fr, fl, br, bl, opv, oph);
        PreciseMovement.setMovementPID(.7, 0, 0);
        PreciseMovement.setAnglePID(.7, 0, 0);
        
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            ElapsedTime et = new ElapsedTime();
            double[] values = PreciseMovement.updateDeadWheelPos();
            telemetry.addData("ophDead", values[0]);
            telemetry.addData("opvDead", values[1]);
            telemetry.addData("opvEncoder", values[2]);
            telemetry.addData("ophEncoder", values[3]);
            
            // DO NOT CHANGE
            double targetXPos = 0;
            double targetYPos = 0;
            double targetAngle = 0;
            
            telemetry.addData("x", PreciseMovement.getPos()[0]);
            telemetry.addData("y", PreciseMovement.getPos()[1]);
            // telemetry.addData("fl", fl.getCurrentPosition());
            // telemetry.addData("fr", fr.getCurrentPosition());
            // telemetry.addData("bl", bl.getCurrentPosition());
            // telemetry.addData("br", br.getCurrentPosition());
            telemetry.addData("oph", oph.getCurrentPosition());
            telemetry.addData("opv", opv.getCurrentPosition());
            telemetry.addData("angle", -imu.getAngularOrientation().firstAngle);
            telemetry.update();
            
            
            
            // fl.setPower(0);
            // fr.setPower(0);
            // bl.setPower(0);
            // br.setPower(0);
        }
    }
    
    
    // private int findRealCenter(int left_pos, int middle_pos, int right_pos) {
    //     double largestHuskyX = 0;
    //     double largestHuskyY = 0;
    //     double threshold = 10;
    //     int center = 0;
    //     int[] centerMedian = new int[5];
            
    //     for (int c = 0; c < 5; c++) {
    //         husky.cmdRequest(husky.BLOCKS);
    //         int l = husky.blocks.length;
    //         telemetry.addData("blocks", l);
    //         for (int i = 0; i < l; i++) {
    //             telemetry.addData("block" + i, husky.blocks[i].center());
    //             double size = husky.blocks[i].width()*husky.blocks[i].height();
                    
    //             if (size > largestHuskyX*largestHuskyY && size > threshold) {
    //                 largestHuskyX = husky.blocks[i].width();
    //                 largestHuskyY = husky.blocks[i].height();
    //                 center = (int)(husky.blocks[i].center()); 
    //             }
    //         } 

    //         centerMedian[c] = center;
    //         sleep(20);
    //     }

    //     center = (int)(findMedian(centerMedian, centerMedian.length)); 
    //     int realCenter;
            
    //     int[] setValues = {left_pos, middle_pos, right_pos};
    //     realCenter = smallestDiff(setValues, center);
        
    //     return realCenter;
    // }
    
    
    private void toggleGripper() {
        if (gripperToggled) {
            gripper.setPosition(0.2);
            gripperToggled = false;
        }
        else {
            gripper.setPosition(0.0);
            gripperToggled = true;
        }
    }
}


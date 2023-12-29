package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.lib.AutonomousState;
import org.firstinspires.ftc.teamcode.lib.Alliance;
import org.firstinspires.ftc.teamcode.lib.Side;

@Autonomous
public class Autotest extends LinearOpMode {
    private DcMotorEx fl = null;
    private DcMotorEx bl = null;
    private DcMotorEx fr = null;
    private DcMotorEx br = null;
    private DcMotorEx opv = null;
    private DcMotorEx oph = null;
    
    private Alliance alliance = Alliance.RED;
    private Side side = Side.LEFT;
    
    @Override
    public void runOpMode() {
        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        br = hardwareMap.get(DcMotorEx.class, "backRight");
        oph = hardwareMap.get(DcMotorEx.class, "frontLeft");
        opv = hardwareMap.get(DcMotorEx.class, "backLeft");
            
        waitForStart();
        
        AutonomousState.init(fl, bl, fr, br, oph, opv, alliance, side);
        AutonomousState state = AutonomousState.Entry;

        while (opModeIsActive()) {
            state = state.run();
        }
    }
}
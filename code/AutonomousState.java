package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.lib.Alliance;
import org.firstinspires.ftc.teamcode.lib.Side;


public enum AutonomousState {
    Entry {
        @Override
        public AutonomousState run() {
            return Scan;
        }
    },
    
    Scan {
        @Override
        public AutonomousState run() {
            return Forward;
        }
    },
    
    Forward {
        @Override
        public AutonomousState run() {
            return Drop;
            // return Middle;
            // return Right;
        }
    },
    
    Left {
        @Override
        public AutonomousState run() {
            return Drop;
        }
    },
    
    Middle {
        @Override
        public AutonomousState run() {
            return Drop;
        }
    },
    
    Right {
        @Override
        public AutonomousState run() {
            return Drop;
        }
    },
    
    Drop {
        @Override
        public AutonomousState run() {
            return Stop;
        }
    },
    
    Stop {
        @Override
        public AutonomousState run() {
            return Stop;
        }
    };
    
    public abstract AutonomousState run();
            
    static DcMotorEx fl, fr, bl, br, oph, opv;
    static Alliance alliance;
    static Side side;
    
    public static void init(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, 
            DcMotorEx br, DcMotorEx oph, DcMotorEx opv, Alliance alliance, 
            Side side) {
                
        AutonomousState.fl = fl;
        AutonomousState.fr = fr;
        AutonomousState.bl = bl;
        AutonomousState.br = br;
        AutonomousState.oph = oph;
        AutonomousState.opv = opv;
        AutonomousState.alliance = alliance;
        AutonomousState.side = side;
    }
}
package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;


public enum AutonomousState {
    Entry {
        @Override
        public AutonomousState run(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, 
                DcMotorEx br, DcMotorEx oph, DcMotorEx opv) {

            return Scan;
        }
    },
    
    Scan {
        @Override
        public AutonomousState run(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, 
                DcMotorEx br, DcMotorEx oph, DcMotorEx opv) {

            return Forward;
        }
    },
    
    Forward {
        @Override
        public AutonomousState run(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, 
                DcMotorEx br, DcMotorEx oph, DcMotorEx opv) {

            return Drop;
            // return Middle;
            // return Right;
        }
    },
    
    Left {
        @Override
        public AutonomousState run(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, 
                DcMotorEx br, DcMotorEx oph, DcMotorEx opv) {

            return Drop;
        }
    },
    
    Middle {
        @Override
        public AutonomousState run(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, 
                DcMotorEx br, DcMotorEx oph, DcMotorEx opv) {

            return Drop;
        }
    },
    
    Right {
        @Override
        public AutonomousState run(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, 
                DcMotorEx br, DcMotorEx oph, DcMotorEx opv) {

            return Drop;
        }
    },
    
    Drop {
        @Override
        public AutonomousState run(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, 
                DcMotorEx br, DcMotorEx oph, DcMotorEx opv) {

            return Stop;
        }
    },
    
    Stop {
        @Override
        public AutonomousState run(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, 
                DcMotorEx br, DcMotorEx oph, DcMotorEx opv) {

            return Stop;
        }
    };
    
    
    public abstract AutonomousState run(DcMotorEx fl, DcMotorEx fr, 
            DcMotorEx bl, DcMotorEx br, DcMotorEx oph, DcMotorEx opv);
}

package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate {
    double gateClosed = 0.3;
    double gateOpen = .7;
    Servo gate;
    public Gate(HardwareMap hardwareMap){
        gate = hardwareMap.get(Servo.class, "gate");
    }
    public void gateOpen(){
        gate.setPosition(gateOpen);
    }
    public void gateClosed(){
        gate.setPosition(gateClosed);
    }
}

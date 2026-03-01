package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate {
//    public double gateClosed = 0.35;
//    public double gateOpen = .7;

    public double gateClosed = 0.87;
    public double gateOpen = .75;

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
    public double gatePos() {
        return gate.getPosition();
    }
}

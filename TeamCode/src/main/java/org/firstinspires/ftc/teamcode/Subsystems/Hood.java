package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
//min .09 max .33
public class Hood {
    public Servo hood;

    public Hood(HardwareMap hardwareMap){
        hood = hardwareMap.get(Servo.class, "hood");
        hood.setPosition(.09);
    }

    public void setPosition(double pos){
        hood.setPosition(Math.max(.09,Math.min(pos,.33)));
    }

}

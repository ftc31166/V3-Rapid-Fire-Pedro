package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    double intakePower = 1;
    DcMotor intake;
    public Intake(HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotor.class, "in");
    }
    public void intakeBalls(){
        intake.setPower(1);
    }
    public void slowIntake(){
        intake.setPower(.7);
    }

    public void stopIntake(){
        intake.setPower(0);
    }

}

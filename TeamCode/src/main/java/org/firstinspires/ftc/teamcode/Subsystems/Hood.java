package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Interpolator;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.PIDController;
//min .09 max .33
public class Hood {
    public Servo hood;
    public Follower follower;
    public Hood(HardwareMap hardwareMap){
        hood = hardwareMap.get(Servo.class, "hood");
        hood.setPosition(.1);
    }

    public void setPosition(double pos){
        hood.setPosition(Math.max(.1,Math.min(pos,.35)));
    }
    public double distanceToRPM(Pose drive, Pose goal){
        double dist = goal.distanceFrom(drive);

        return dist < 75 ? 0.45 : 0.09; //old max was 0.35 -> change if not working :P

    }


}

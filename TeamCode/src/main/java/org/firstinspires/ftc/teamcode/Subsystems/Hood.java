package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Interpolator;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.Interplut;
import org.firstinspires.ftc.teamcode.Utils.PIDController;
//min .09 max .33
public class Hood {
    public Servo hood;
    public Follower follower;
    Interplut hoodTable = new Interplut();
    public Hood(HardwareMap hardwareMap){
        hood = hardwareMap.get(Servo.class, "hood");
        hood.setPosition(.6);
        hoodTable.addPoint(40,.6);
        hoodTable.addPoint(60,.6);
        hoodTable.addPoint(80,.6);
        hoodTable.addPoint(100,.55);
        hoodTable.addPoint(120,.5);
        hoodTable.addPoint(140,.35);
        hoodTable.addPoint(160,.35);

    }

    public void setPosition(double pos){
        hood.setPosition(Math.max(.35,Math.min(pos,.6)));
    }
    public double distanceToRPM(Pose drive, Pose goal){
        double dist = goal.distanceFrom(drive);
        return hoodTable.getInterpolatedValue(dist);

    }
    public double distanceToRPM(double dist){

        return hoodTable.getInterpolatedValue(dist);


    }
    public double angle(double dist){

        double servoPos = hoodTable.getInterpolatedValue(dist);
        return (90-((.6-servoPos)*(1800)*(14.0/337.0) + 32.5));

    }

}

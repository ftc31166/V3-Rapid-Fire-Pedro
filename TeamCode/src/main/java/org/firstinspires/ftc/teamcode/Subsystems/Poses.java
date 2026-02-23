package org.firstinspires.ftc.teamcode.Subsystems;


import com.pedropathing.geometry.Pose;

public class Poses {
    public static Pose redGoal = new Pose(144,144,0);
    public static Pose redAutoEnd = new Pose(120,72,Math.toRadians(0));
    public static Pose redReinit = new Pose(138, 72,Math.toRadians(0));
    public static Pose blueGoal = redGoal.mirror(144);
    public static Pose blueAutoEnd = redAutoEnd.mirror(144);
    public static Pose blueReinit = redReinit.mirror(144);
    

}

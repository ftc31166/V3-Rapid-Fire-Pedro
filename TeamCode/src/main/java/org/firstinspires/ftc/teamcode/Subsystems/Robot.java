package org.firstinspires.ftc.teamcode.Subsystems;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utils.Interplut;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Robot {


    public Follower drive;

    public Flywheels flywheels;
    public Gate gate;
    public Intake intake;
    public Turret turret;
    public Hood hood;
    double lastTime = 0;
    ElapsedTime timer = new ElapsedTime();
    Pose goal;
    Interplut timeTable = new Interplut();

    public Robot(HardwareMap hardwareMap, Pose initPose, Pose goal){
        drive = Constants.createFollower(hardwareMap);
        drive.setStartingPose(initPose);
        flywheels = new Flywheels(hardwareMap);
        gate = new Gate( hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        hood = new Hood(hardwareMap);
        this.goal = goal;
        timeTable.addPoint(49,2800);
        timeTable.addPoint(70,3000);
        timeTable.addPoint(101,3550);
        timeTable.addPoint(107,3700);
        timeTable.addPoint(125,3950);
        timeTable.addPoint(135,4800);
        timeTable.addPoint(156,5500);

    }
    

    
    public double[] autoAimMove(){
        Pose drivePose = drive.getPose();
        double dx = -goal.getX() + drivePose.getX();
        double dy = -goal.getY() + drivePose.getY();
        double degrees = Math.atan2(dy,dx);
        
        double distance = goal.distanceFrom(drivePose);
        Vector velVector = drive.getVelocity();
        double ballTimeToGoal = timeTable.getInterpolatedValue(distance);
        double perp = velVector.getXComponent()*Math.cos(degrees) - velVector.getYComponent()*Math.sin(degrees);
        double parallel = -(velVector.getXComponent()*Math.sin(degrees) + velVector.getYComponent()*Math.cos(degrees));
        double angleOffset = Math.toDegrees(Math.atan2(ballTimeToGoal*perp,distance));
        double distancePredicted = parallel*ballTimeToGoal;
        return new double[]{turret.autoAim(drivePose,goal)-angleOffset, flywheels.distanceToRPM(drivePose,goal,distancePredicted)};
    }



}

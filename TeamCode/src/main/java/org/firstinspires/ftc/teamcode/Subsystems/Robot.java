package org.firstinspires.ftc.teamcode.Subsystems;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Utils.Interplut;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Robot {
    public GoBildaPinpointDriver pinpointDriver;



    public Flywheels flywheels;
    public Gate gate;
    public Intake intake;
    public Turret turret;
    public Hood hood;
    double lastTime = 0;
    ElapsedTime timer = new ElapsedTime();
    Pose goal;
    Interplut timeTable = new Interplut();
    DcMotor FL, BL, FR, BR;
    public Robot(HardwareMap hardwareMap, Pose initPose, Pose goal){
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//        drive = Constants.createFollower(hardwareMap);
//        drive.setStartingPose(initPose);
//        drive.update();
        pinpointDriver.setPosition(new Pose2D(DistanceUnit.INCH, initPose.getX(),initPose.getY(), AngleUnit.RADIANS,initPose.getHeading()));
        pinpointDriver.setOffsets(.5,-5.5,DistanceUnit.INCH);
        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,GoBildaPinpointDriver.EncoderDirection.REVERSED);
        flywheels = new Flywheels(hardwareMap);
        gate = new Gate( hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        hood = new Hood(hardwareMap);
        this.goal = goal;

        timeTable.addPoint(60,5.57-5.07);
        timeTable.addPoint(102,1.71-.98);
        timeTable.addPoint(110,7.74-7);
        timeTable.addPoint(80,3.58-3.03);
        timeTable.addPoint(120,3.3-2.6);
        timeTable.addPoint(130,11.50-10.77);
        timeTable.addPoint(160,1.99-1.09);

         FL = hardwareMap.get(DcMotor.class, "FL");
         BL = hardwareMap.get(DcMotor.class, "BL");
         FR = hardwareMap.get(DcMotor.class, "FR");
         BR = hardwareMap.get(DcMotor.class, "BR");
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gate.gateClosed();


    }
    public double[] velocities(){
        return (new double[]{pinpointDriver.getVelX(DistanceUnit.INCH), pinpointDriver.getVelY(DistanceUnit.INCH)});
    }

    
    public double[] autoAimMove(Pose drivePose){
        double dx = -goal.getX() + drivePose.getX();
        double dy = -goal.getY() + drivePose.getY();
        double degrees = Math.atan2(dy,dx);
        
        double distance = goal.distanceFrom(drivePose);
        double[] velVector = velocities();
        double ballTimeToGoal = timeTable.getInterpolatedValue(distance);
        double perp = velVector[0]*Math.cos(degrees) - velVector[1]*Math.sin(degrees);
        double parallel = -(velVector[0]*Math.sin(degrees) + velVector[1]*Math.cos(degrees));
        double angleOffset = Math.toDegrees(Math.atan2(ballTimeToGoal*perp,distance));
        double distancePredicted = parallel*ballTimeToGoal;
        double predictedTotalDistance = distance + distancePredicted;
        return new double[]{turret.autoAim(drivePose,goal)-angleOffset, flywheels.flywheelTable.getInterpolatedValue(predictedTotalDistance), hood.distanceToRPM(predictedTotalDistance)};
//        return new double[]{turret.autoAim(drivePose,goal), flywheels.flywheelTable.getInterpolatedValue(distance), hood.distanceToRPM(distance)};
    }

    public void driveRoboCentric(double left_stick_y, double left_stick_x, double right_stick_x){
        double y = -left_stick_y; // Remember, Y stick value is reversed
        double x = left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double fl = (y + x + rx) / denominator;
        double bl = (y - x + rx) / denominator;
        double fr = (y - x - rx) / denominator;
        double br = (y + x - rx) / denominator;

        FL.setPower(fl);
        BL.setPower(bl);
        FR.setPower(fr);
        BR.setPower(br);

    }

}

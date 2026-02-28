package org.firstinspires.ftc.teamcode.Subsystems;


import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.SerialNumber;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Utils.Interplut;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Robot {
    public GoBildaPinpointDriver pinpointDriver;



    public Flywheels flywheels;
    public Gate gate;
    public Intake intake;
    public Turret turret;
    public Hood hood;

    ElapsedTime timer = new ElapsedTime();
    Pose goal;
    Interplut timeTable = new Interplut();
    DcMotor FL, BL, FR, BR;
    private Pose lastPose = new Pose(0, 0, 0);
    private long lastTime = System.currentTimeMillis();

    public Limelight3A cam;
    public Robot(HardwareMap hardwareMap, Pose initPose, Pose goal){
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//        drive = Constants.createFollower(hardwareMap);
//        drive.setStartingPose(initPose);
//        drive.update();
        cam = hardwareMap.get(Limelight3A.class, "Webcam1");
        cam.pipelineSwitch(1);
        cam.start();
        pinpointDriver.setPosition(new Pose2D(DistanceUnit.INCH, initPose.getX(),initPose.getY(), AngleUnit.RADIANS,initPose.getHeading()));
        pinpointDriver.setOffsets(.4375,-5.4125,DistanceUnit.INCH);
        pinpointDriver.setEncoderResolution(19.89437,DistanceUnit.MM);
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
//    public double[] velocities(){
//        return (new double[]{pinpointDriver.getVelX(DistanceUnit.INCH), pinpointDriver.getVelY(DistanceUnit.INCH)});
//    }
public double[] velocities(Pose currentPose) {
    long currentTime = System.currentTimeMillis();

    // Calculate time difference in seconds
    double dt = (currentTime - lastTime) / 1000.0;

    // Prevent division by zero if loop is too fast
    if (dt <= 0) return new double[]{0, 0};

    // Velocity = (Current Position - Last Position) / Time
    double vx = (currentPose.getX() - lastPose.getX()) / dt;
    double vy = (currentPose.getY() - lastPose.getY()) / dt;

    // Save for next loop
    lastPose = currentPose;
    lastTime = currentTime;

    return new double[]{(Math.abs(vx)>1)?vx:0, (Math.abs(vy)>1)?vy:0};
}

    
    public double[] autoAimMove(Pose drivePose){
//        double dx = -goal.getX() + drivePose.getX();
//        double dy = -goal.getY() + drivePose.getY();
//        double degrees = Math.atan2(dy,dx);
//
        double distance = goal.distanceFrom(drivePose);
        double rpm = flywheels.flywheelTable.getInterpolatedValue(distance);
//        double[] velVector = velocities(drivePose);
//        double ballTimeToGoal = rpmToTime(rpm,distance);
//        double perp = velVector[0]*Math.cos(degrees) - velVector[1]*Math.sin(degrees);
//        double parallel = -(velVector[0]*Math.sin(degrees) + velVector[1]*Math.cos(degrees));
//        double angleOffset = Math.toDegrees(Math.atan2(ballTimeToGoal*perp,distance));


        return new double[]{turret.autoAim(drivePose,goal), rpm, hood.distanceToRPM(distance)};
//        return new double[]{turret.autoAim(drivePose,goal), flywheels.flywheelTable.getInterpolatedValue(distance), hood.distanceToRPM(distance)};
    }
//    public double[ ] autoAimMove(Pose drivePose){
//        double dx1 = -goal.getX() + drivePose.getX();
//        double dy1 = -goal.getY() + drivePose.getY();
//        double degrees = Math.atan2(dy1,dx1);
//        double dist = goal.distanceFrom(drivePose);
////        double ballTime = timeTable.getInterpolatedValue(dist);
//        double rpm = flywheels.flywheelTable.getInterpolatedValue(dist);
//        double ballTime = rpmToTime(rpm,dist);
//        double[] velVector = velocities(drivePose);
//        double dx = goal.getPose().getX() -drivePose.getX() -velVector[0]*ballTime;
//        double dy = goal.getPose().getY() - drivePose.getY()-velVector[1]*ballTime;
//        double parallel = -(velVector[0]*Math.sin(degrees) + velVector[1]*Math.cos(degrees));
//        return new double[]{turret.autoAim(drivePose,goal),  rpm, hood.hoodTable.getInterpolatedValue(dist)};
//
//    }

//    public double velToRPM(double vel){
//        double k = 1.1;
//        return k*(vel*30/(Math.PI*1.41732));
//
//    }
//    public double rpmToTime(double rpm, double dist){
//        double k = 1.1;
//        double vel=(rpm*Math.PI*1.41732/30);
//        return k*dist/(vel*Math.cos(Math.toRadians(hood.angle(dist))));
//
//    }




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

    public Pose2D getRobotPoseFromCamera() {

        cam.updateRobotOrientation(pinpointDriver.getHeading(AngleUnit.RADIANS));
        LLResult result = cam.getLatestResult();
        Pose3D pose;
        if((result!=null)&&result.isValid()) {
            pose = result.getBotpose_MT2();
            return new Pose2D(DistanceUnit.INCH,pose.getPosition().x*39.3701+72, pose.getPosition().y*39.3701+72, AngleUnit.RADIANS,pose.getOrientation().getYaw(AngleUnit.RADIANS)+90);
        }
        else {
            return new Pose2D(DistanceUnit.INCH,pinpointDriver.getPosition().getX(DistanceUnit.INCH),pinpointDriver.getPosition().getY(DistanceUnit.INCH),AngleUnit.RADIANS,pinpointDriver.getHeading(AngleUnit.RADIANS));
        }

    }

}

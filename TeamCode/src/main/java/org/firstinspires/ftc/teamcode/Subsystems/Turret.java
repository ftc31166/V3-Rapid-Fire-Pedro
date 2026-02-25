package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utils.Interplut;
import org.firstinspires.ftc.teamcode.Utils.PIDController;


public class Turret {

    public double kP = 0.008;
    public double kI = 0;
    public double kD = 0.000008;
    public double turretOffset = 0;
     public double targetTicks = 0;
    public double TICKS_PER_REV = 1673;
    public DcMotorEx turret;
    PIDController pid;




    public Turret(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "tur");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setCoeffs();
    }
    public void stop(){
        turret.setPower(0);
    }
    public void setCoeffs(){
        pid = new PIDController(kP, kI, kD);
    }

    public void setTargetAngle(double angleDeg) {
        double angle =  wrapToRange(angleDeg, -100, 280);

        targetTicks = angle * TICKS_PER_REV/ 360.0 ;
    }
    public double autoAim(Pose drive, Pose goal){

//        double turretOffsetX = -2.5;
//        double turretOffsetY = 0.0;
//
//        double turretX = drive.getX()
//                + turretOffsetX * Math.cos(drive.getHeading())
//                - turretOffsetY * Math.sin(drive.getHeading());
//
//        double turretY = drive.getY()
//                + turretOffsetX * Math.sin(drive.getHeading())
//                + turretOffsetY * Math.cos(drive.getHeading());

        double dx = goal.getX() - drive.getX();
        double dy = goal.getY() - drive.getY();
//        double dx = goal.getX() - turretX;
//        double dy = goal.getY() - turretY;

        double degrees = Math.toDegrees(Math.atan2(dy, dx));

        double heading = Math.toDegrees(drive.getHeading());

        return degrees-heading+turretOffset;
    }




    public double[] update() {
        double currentTicks = turret.getCurrentPosition();
        double power = pid.calculate(currentTicks, targetTicks);
        power = Math.max(-.5,Math.min(.5,power));
        turret.setPower(power);
        double[] ret = {currentTicks, targetTicks, power};
        return ret;
    }
    public static double wrapToRange(double angle, double min, double max) {
        double width = max - min;
        return ((angle - min) % width + width) % width + min;
    }
}

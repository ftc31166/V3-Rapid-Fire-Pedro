package org.firstinspires.ftc.teamcode.Subsystems;


import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.Interplut;
import org.firstinspires.ftc.teamcode.Utils.PIDFController;


public class Flywheels {

     public double targetRPM = 0;
    public double closeRPM = 3000;
    public double kF = 1.0/5600;
    public double kP = 0.004;
    public double kI = 0;
    public double kD = 0;
    double TICKS_PER_REV = 28.0;


    public  DcMotorEx lfly;
    public DcMotorEx rfly;
     PIDFController pidf;
    Interplut flywheelTable = new Interplut();


    public Flywheels(HardwareMap hardwareMap){
        lfly = hardwareMap.get(DcMotorEx.class,"lfly");
        rfly = hardwareMap.get(DcMotorEx.class,"rfly");
        rfly.setDirection(DcMotorSimple.Direction.REVERSE);
        lfly.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfly.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lfly.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfly.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lfly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rfly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setCoeffs();


        flywheelTable.addPoint(40,2900);
        flywheelTable.addPoint(60,2900);
        flywheelTable.addPoint(80,3000);
        flywheelTable.addPoint(100,3400);
        flywheelTable.addPoint(120,3800);
        flywheelTable.addPoint(140,4100);
        flywheelTable.addPoint(160,4400);
    }
    public void stop(){
        lfly.setPower(0);
        rfly.setPower(0);
    }
    public void setCoeffs(){
        pidf = new PIDFController(kP, kI, kD,kF);
    }

    public double distanceToRPM(Pose drive, Pose goal, double posOffset){

        double dist = drive.distanceFrom(goal);


        return flywheelTable.getInterpolatedValue(dist+posOffset);//equation for auto power that we used for ftc
    }
    public void setTargetRPM( double target){
        targetRPM = target;

    }
    public double getRPM() {
        double ticksPerSecond = lfly.getVelocity();
        return (ticksPerSecond / TICKS_PER_REV) * 60.0;
    }



    public double[] update(){

        double power = pidf.calculate(getRPM(),targetRPM);
        power = Math.max(0,Math.min(1,power));
        lfly.setPower(power);
        rfly.setPower(power);
        double[] ret = {getRPM(), targetRPM, power};
        return ret;
    }

    public boolean atRPM() {
        return Math.abs(getRPM() - targetRPM) < 75;
    }
}

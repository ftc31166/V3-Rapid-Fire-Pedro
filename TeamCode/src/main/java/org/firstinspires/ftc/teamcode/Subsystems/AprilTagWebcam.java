//package org.firstinspires.ftc.teamcode.subsystems;
//
//import android.util.Size;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import java.util.ArrayList;
//import java.util.List;
//
//public class AprilTagWebcam {
//    public class AprilTagWebacam{
//        private AprilTagProcessor aprilTagprocessor;
//        private VisionPortal visionportal;
//        private List<AprilTagDetection> detectedTags = new ArrayList<>();
//        private Telemetry telemetry;
//        public void init(HardwareMap hardwareMap, Telemetry telemetry){
//            this.telemetry = telemetry;
//            aprilTagprocessor = new AprilTagProcessor.Builder()
//			        .setDrawTagID(true)
//                    .setDrawTagOutline(true)
//                    .setDrawAxes(true)
//                    .setDrawCubeProjection(true)
//                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
//                    .setCameraPose( new Position(DistanceUnit.INCH, 0,7,0,0), new YawPitchRollAngles(AngleUnit.RADIANS,0,0,0,0))
//                    .build();
//            VisionPortal.Builder builder = new VisionPortal.Builder();
//            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//            builder.setCameraResolution(new Size(640,480));
//            builder.addProcessor(aprilTagprocessor);
//
//            visionportal = builder.build();
//        }
//
//        public void update(){
//            detectedTags = aprilTagprocessor.getDetections();
//        }
//
//        public List<AprilTagDetection> getDetectedTags(){
//            return detectedTags;
//        }
//
//        public AprilTagDetection getSpecificTag(int id){
//            for(AprilTagDetection detection: detectedTags){
//                if(detection.id == id){
//                    return detection;
//                }
//            }
//            return null;
//        }
//
//        public void stop(){
//            if(visionportal != null){
//                visionportal.close();
//            }
//        }
//
//        public Pose2d getTrustedRobotPose(AprilTagDetection detection) {
//
//                if (detection.metadata != null && detection.decisionMargin > 20) {
//                    if (detection.ftcPose.range < 48) {
//                        return new Pose2d(
//                                detection.robotPose.getPosition().x,
//                                detection.robotPose.getPosition().y,
//                                detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)
//                        );
//                    }
//                }
//
//            return null;
//        }
//
//
//    }
//
//}

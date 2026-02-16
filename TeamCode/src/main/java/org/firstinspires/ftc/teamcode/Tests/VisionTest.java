package org.firstinspires.ftc.teamcode.Tests;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@TeleOp(name = "vision")
public class VisionTest extends OpMode {
    AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
            // The Logitech C270 typically uses these intrinsics at 640x480 resolution
            // fx, fy, cx, cy (standard values for C270)
            .setLensIntrinsics(822.317, 822.317, 319.495, 242.502)
            .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
            .setCameraPose(
                    new Position(DistanceUnit.INCH, 7, 0, 0, 0), // 7 inches forward
                    new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0)
            )
            .build();

    VisionPortal visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .setCameraResolution(new Size(640, 480)) // C270 standard res
            .addProcessor(aprilTag)
            .setAutoStopLiveView(false)
            .build();
    @Override
    public void init() {

        visionPortal.setProcessorEnabled(aprilTag, true);

        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        GainControl gain = visionPortal.getCameraControl(GainControl.class);

        if (exposure.getMode() != ExposureControl.Mode.Manual) {
            exposure.setMode(ExposureControl.Mode.Manual);
        }

        exposure.setExposure(6, java.util.concurrent.TimeUnit.MILLISECONDS);

        gain.setGain(250);
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("Waiting for camera...");
            telemetry.update();
        }
        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
    }

    @Override
    public void loop() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        telemetry.addData("Detections", detections.size());

        for (AprilTagDetection d : detections) {
            telemetry.addLine("----");
            telemetry.addData("ID", d.id);
            telemetry.addData("Metadata null", d.metadata == null);

            if (d.metadata != null) {
                telemetry.addData("Name", d.metadata.name);
                telemetry.addData("Field X", d.metadata.fieldPosition.get(0));
                telemetry.addData("Field Y", d.metadata.fieldPosition.get(1));
            }
        }
    }
}

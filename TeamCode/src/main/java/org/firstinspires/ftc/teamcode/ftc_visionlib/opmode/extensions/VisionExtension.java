package org.firstinspires.ftc.teamcode.ftc_visionlib.opmode.extensions;

/**
 * Created by skantare on 9/15/16.
 */
import org.firstinspires.ftc.teamcode.ftc_visionlib.opmode.VisionOpMode;
import org.opencv.core.Mat;

/**
 * Interface for vision extensions for VisionOpMode
 */
public interface VisionExtension {
    void init(VisionOpMode opmode);

    void loop(VisionOpMode opmode);

    Mat frame(VisionOpMode opmode, Mat rgba, Mat gray);

    void stop(VisionOpMode opmode);
}

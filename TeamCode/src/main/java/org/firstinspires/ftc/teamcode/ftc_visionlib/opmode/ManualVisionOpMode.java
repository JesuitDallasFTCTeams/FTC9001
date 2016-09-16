package org.firstinspires.ftc.teamcode.ftc_visionlib.opmode;

/**
 * Created by skantare on 9/15/16.
 */
import org.opencv.core.Mat;

public abstract class ManualVisionOpMode extends VisionOpModeCore {
    public final Mat frame(Mat rgba, Mat gray, boolean ready) {
        return frame(rgba, gray);
    }

    /**
     * Returns every frame an image received from the camera.
     * If your method runs for too long, frames will be skipped (this is normal behaviour, reducing
     * FPS).
     *
     * @param rgba RGBA image
     * @param gray Grayscale image
     * @return Return an image to draw onto the screen
     */
    public abstract Mat frame(Mat rgba, Mat gray);
}


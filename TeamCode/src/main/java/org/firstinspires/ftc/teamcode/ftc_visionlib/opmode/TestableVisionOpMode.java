package org.firstinspires.ftc.teamcode.ftc_visionlib.opmode;

/**
 * Created by skantare on 9/15/16.
 */
/**
 * Vision Op Mode designed ONLY for testing applications, such as the Camera Test Activity
 * This OpMode essentially unifies testing applications and the robot controller
 */
public abstract class TestableVisionOpMode extends VisionOpMode {

    /**
     * Creates the Testable OpMode.
     */
    public TestableVisionOpMode() {
        super(false); //disable OpenCV core functions
    }

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        super.loop();
    }

    @Override
    public void stop() {
        super.stop();
    }
}

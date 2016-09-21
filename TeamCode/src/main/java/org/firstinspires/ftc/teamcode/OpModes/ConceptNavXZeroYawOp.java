package org.firstinspires.ftc.teamcode.OpModes;

/**
 * Created by skantare on 9/19/16.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.navx_ftc.ftc.AHRS;

import java.text.DecimalFormat;



/**
 *  navX-Micro Zero Yaw Op
 * <p>
 * Acquires processed data from navX-Micro
 * and displays it in the Robot DriveStation
 * as telemetry data.  This opmode demonstrates how to "zero"
 * (reset to zero degrees) the yaw.  This causes whatever
 * direction the navX-Model device is currently pointing to
 * now be zero degrees, and is an effective method for dealing
 * with accumulating Yaw Drift.
 *
 * For more information on Yaw drift, please see the online help at:
 * http://navx-micro.kauailabs.com/guidance/yaw-drift/
 */
@TeleOp(name = "Concept: navX Zero Yaw", group = "Concept")
// @Disabled Comment this in to remove this from the Driver Station OpMode List
public class ConceptNavXZeroYawOp extends OpMode {

    /* This is the port on the Core Device Interace Module */
  /* in which the navX-Micro is connected.  Modify this  */
  /* depending upon which I2C port you are using.        */
    private final int NAVX_DIM_I2C_PORT = 0;

    private String startDate;
    private ElapsedTime runtime = new ElapsedTime();
    private AHRS navx_device;

    @Override
    public void init() {
        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData);
    }

    @Override
    public void stop() {
        navx_device.close();
    }

    /*
       * Code to run when the op mode is first enabled goes here
       * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
       */
    @Override
    public void init_loop() {
        telemetry.addData("navX Op Init Loop", runtime.toString());
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {

        boolean connected = navx_device.isConnected();
        telemetry.addData("1 navX-Device", connected ?
                "Connected" : "Disconnected");
        String gyrocal, magcal, yaw, pitch, roll, compass_heading;
        String fused_heading, ypr, cf, motion;
        DecimalFormat df = new DecimalFormat("#.##");

        if (connected) {
            gyrocal = (navx_device.isCalibrating() ?
                    "CALIBRATING" : "Calibration Complete");
            magcal = (navx_device.isMagnetometerCalibrated() ?
                    "Calibrated" : "UNCALIBRATED");
            yaw = df.format(navx_device.getYaw());
            pitch = df.format(navx_device.getPitch());
            roll = df.format(navx_device.getRoll());
            ypr = yaw + ", " + pitch + ", " + roll;
            compass_heading = df.format(navx_device.getCompassHeading());
            fused_heading = df.format(navx_device.getFusedHeading());
            if (!navx_device.isMagnetometerCalibrated()) {
                compass_heading = "-------";
            }
            cf = compass_heading + ", " + fused_heading;
            if (navx_device.isMagneticDisturbance()) {
                cf += " (Mag. Disturbance)";
            }
            motion = (navx_device.isMoving() ? "Moving" : "Not Moving");
            if (navx_device.isRotating()) {
                motion += ", Rotating";
            }
        } else {
            gyrocal =
                    magcal =
                            ypr =
                                    cf =
                                            motion = "-------";
        }
        telemetry.addData("2 GyroAccel", gyrocal);
        telemetry.addData("3 Y,P,R", ypr);
        telemetry.addData("4 Magnetometer", magcal);
        telemetry.addData("5 Compass,9Axis", cf);
        telemetry.addData("6 Motion", motion);

      /* If the left 'bumper' button pressed,
         reset (zero) the current yaw angle.  This causes whatever
         direction the navX-Model device is currently pointing to
         now be zero degrees.
       */
        if (gamepad1.left_bumper) {
            if ((navx_device.getUpdateCount() % 500) == 0) {
                navx_device.zeroYaw();
            }
        }

    }
}

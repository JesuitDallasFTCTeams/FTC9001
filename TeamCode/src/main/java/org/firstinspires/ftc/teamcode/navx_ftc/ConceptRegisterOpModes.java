package org.firstinspires.ftc.teamcode.navx_ftc;

/**
 * Created by skantare on 9/14/16.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.teamcode.OpModes.ConceptNavXDriveStraightPIDLoopOp;
import org.firstinspires.ftc.teamcode.OpModes.ConceptNavXRotateToAnglePIDLoopOp;

/**
 * This class demonstrates how to manually register opmodes.
 *
 * Note: It is NOT required to manually register OpModes, but this method is provided to support teams that
 * want to have centralized control over their opmode lists.
 * This sample is ALSO handy to use to selectively register the other sample opmodes.
 * Just copy/paste this sample to the TeamCode module and then uncomment the opmodes you wish to run.
 *
 * How it works:
 * The registerMyOpmodes method will be called by the system during the automatic registration process
 * You can register any of the opmodes in your App. by making manager.register() calls inside registerMyOpmodes();
 *
 * Note:
 * Even though you are performing a manual Registration, you should set the Annotations on your classes so they
 * can be placed into the correct Driver Station OpMode list...  eg:
 *
 * @Autonomous(name="DriveAndScoreRed", group ="Red")
 * or
 * @TeleOp(name="FastDrive", group ="A-Team")
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Then uncomment and copy the manager.register() call to register as many of your OpModes as you like.
 * You can even use it to temporarily register samples directly from the robotController/external/samples folder.
 */
public class ConceptRegisterOpModes
{

    @OpModeRegistrar
    public static void registerMyOpModes(OpModeManager manager) {
        // Un-comment any line to enable that sample.
        // Or add your own lines to register your Team opmodes.

        // Basic Templates
        // manager.register("Iterative Opmode",       TemplateOpMode_Iterative.class);
        // manager.register("Linear Opmode",          TemplateOpMode_Linear.class);

        // Driving Samples
        // manager.register("Teleop POV",             PushbotTeleopPOV_Linear.class);
        // manager.register("Teleop Tank",            PushbotTeleopTank_Iterative.class);
        // manager.register("Auto Drive Gyro",        PushbotAutoDriveByGyro_Linear.class);
        // manager.register("Auto Drive Encoder",     PushbotAutoDriveByEncoder_Linear.class);
        // manager.register("Auto Drive Time",        PushbotAutoDriveByTime_Linear.class);
        // manager.register("Auto Drive Line",        PushbotAutoDriveToLine_Linear.class);
        // manager.register("K9 Telop",               K9botTeleopTank_Linear.class);

        // Sensor Samples
        // manager.register("AdaFruit Color",         SensorAdafruitRGB.class);
        // manager.register("HT Color",               SensorHTColor.class);
        // manager.register("LEGO Light",             SensorLEGOLight.class);
        // manager.register("LEGO Touch",             SensorLEGOTouch.class);
        // manager.register("MR Color",               SensorMRColor.class);
        // manager.register("MR Gyro",                SensorMRGyro.class);
         manager.register("ConceptNavXDriveStraightPIDLoopOp",           ConceptNavXDriveStraightPIDLoopOp.class);
         manager.register("ConceptNavXRotateToAnglePIDLoopOp",                 ConceptNavXRotateToAnglePIDLoopOp.class);

        //  Concept Samples
        // manager.register("Null Op",                ConceptNullOp.class);
        // manager.register("Compass Calibration",    ConceptCompassCalibration.class);
        // manager.register("DIM as Indicator",       ConceptDIMAsIndicator.class);
        // manager.register("I2C Address Change",     ConceptI2cAddressChange.class);
        // manager.register("Ramp Motor Speed",       ConceptRampMotorSpeed.class);
        // manager.register("Scan Servo",             ConceptScanServo.class);
        // manager.register("Telemetry",              ConceptTelemetry.class);
        // manager.register("Vuforia Navigation",     ConceptVuforiaNavigation.class);
    }
}

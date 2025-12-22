/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.Arrays;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Robot: Auto Drive By Encoder", group="Robot")
//@Disabled
public class Magazine extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor magazine = null;

    private ElapsedTime runtime = new ElapsedTime();
    private PredominantColorProcessor colorSensor;
    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final int COUNTS_PER_FULL_REV = 96;    // This should be 47.1 at some point but it's fine for now
    //static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    //static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    //static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
    //(WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    //static final double     TURN_SPEED              = 0.5;

    String[] colors = new String[3];
    float slotNumber = 0f;
    float topSlotNumber = 1.5f;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        magazine = hardwareMap.get(DcMotor.class, "magazine");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        magazine.setDirection(DcMotor.Direction.FORWARD);

        magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magazine.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", magazine.getCurrentPosition());
        telemetry.update();
        // Wait for the game to start (driver presses START)

        colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        // PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();

        /*
         * Build a vision portal to run the Color Sensor process.
         *
         *  - Add the colorSensor process created above.
         *  - Set the desired video resolution.
         *      Since a high resolution will not improve this process, choose a lower resolution
         *      supported by your camera.  This will improve overall performance and reduce latency.
         *  - Choose your video source.  This may be
         *      .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  .....   for a webcam
         *  or
         *      .setCamera(BuiltinCameraDirection.BACK)    ... for a Phone Camera
         */
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.setMsTransmissionInterval(100);  // Speed up telemetry updates, for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        //PredominantColorProcessor.Result result = colorSensor.getAnalysis();

        waitForStart();
        PredominantColorProcessor.Result result = colorSensor.getAnalysis();
        for (int i = 0; i < 3; i++) {
            while (magazine.isBusy()) {
                //hold loop while function moving
            }
            sleep(1000);
            fullRotation(1);
            telemetry.addData("Best Match", result.closestSwatch.toString());
            telemetry.addData("Color Array", colors[0] + ", " + colors[1] + ", " + colors[2]);
            telemetry.update();
        }


        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        //telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(3000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    /*public void halfRotation() {
        PredominantColorProcessor.Result result = colorSensor.getAnalysis();
        sleep(1500);
        if (slotNumber % 1.0 == 0) {
            colors[(int) slotNumber] = result.closestSwatch.toString();
        }
        if (slotNumber < 2.5) {
            slotNumber += 0.5;
        } else {
            slotNumber = 0;
        }
        magazine.setTargetPosition(magazine.getCurrentPosition() + COUNTS_PER_FULL_REV*0.5);
        magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        magazine.setPower(DRIVE_SPEED);
    } */
public void fullRotation(float numOfRotations) {
    sleep(1000);
    PredominantColorProcessor.Result result = colorSensor.getAnalysis();
    sleep(1000);
    if (slotNumber % 1.0 == 0) {
        colors[(int) slotNumber] = result.closestSwatch.toString();
    }
    slotNumber += (1*numOfRotations);
    if (slotNumber > 3) {
        slotNumber = slotNumber - 3;
    }

    topSlotNumber += (1*numOfRotations);
    if (topSlotNumber > 3) {
        topSlotNumber = topSlotNumber - 3;
    }

    magazine.setTargetPosition(magazine.getCurrentPosition() + (int)(COUNTS_PER_FULL_REV*numOfRotations));
    magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    magazine.setPower(DRIVE_SPEED);
}

public void launchMotif(int aprilTagID) {
    String[] motif = new String[3];
    switch(aprilTagID) {
        case 21:
            motif[0] = "ARTIFACT_GREEN";
            motif[1] = "ARTIFACT_PURPLE";
            motif[2] = "ARTIFACT_PURPLE";             // optional, prevents fall-through
        case 22:
            motif[0] = "ARTIFACT_PURPLE";
            motif[1] = "ARTIFACT_GREEN";
            motif[2] = "ARTIFACT_PURPLE";
        case 23:

            motif[0] = "ARTIFACT_PURPLE";
            motif[1] = "ARTIFACT_PURPLE";
            motif[2] = "ARTIFACT_GREEN";
        default:
            telemetry.addData("no case")
                // code block to execute if no case matches (optional)
    }

    /// insert apriltag motifs last
    for (int i = 0; i < 3; i++) {
        while (colors[(int) topSlotNumber] != motif[i]) {
            fullRotation(1);
        }
        telemetry.addData("Status:", "launch");
        sleep(3000);//launch
    }
}

    // Stop all motion;
    //leftDrive.setPower(0);
    //rightDrive.setPower(0);

    // Turn off RUN_TO_POSITION
    //leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    //sleep(250);   // optional pause after each move.
}



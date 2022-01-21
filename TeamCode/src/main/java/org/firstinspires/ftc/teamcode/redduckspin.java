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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Constants.SamplingLocation;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import java.util.List;
import java.util.List;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 * hi
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="redduckspin", group="Pushbot")
public class redduckspin extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_DM.tflite";
    private ElapsedTime runtime = new ElapsedTime();
    private static final String[] LABELS = {
        //"Ball",
        //"Cube",
        "Duck",
        "Marker"
    };
    private static final String VUFORIA_KEY =
        "ASxSfhX/////AAABmWcpvgdyP053gmhPvX7/JZ5yybQKAVFnqMk+WjYvbuuiectzmcdkuftxSIgVawrOZ7CQOqdHzISXbHCAom4FhIzrDceJIIEGozFWpgAu5dUKc3q843Hd3x875VOBf8B7DlD7g9TgqxqgQRw9coEUBBeEJqy2KGy4NLPoIKLdiIx8yxSWm7SlooFSgmrutF/roBtVM/N+FhY6Sgdy9fgWssccAhd2IxdYllAaw4s1oC1jqtwbjIsdjNVogmwwXdTmqiKHait1PFyF2FDNfKi+7qs4Mc6KbvXD2FHA6RljkcN5Oo080o2QSVCzDuQtJeagh/CglB2PcatFWnebiWN+a43kEdrUaY+uq0YQ8m9IRBWE";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    //    /**
//     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
//     * Detection engine.
//     */
    private TFObjectDetector tfod;

    private org.firstinspires.ftc.teamcode.TeamMarkerDetector detector;

    private SamplingLocation samplingLocation = SamplingLocation.RIGHT;


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforiaLocalizer.enableConvertFrameToBitmap();
        vuforiaLocalizer.setFrameQueueCapacity(1);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
//        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
//        if (tfod != null) {
//            tfod.activate();

//             // The TensorFlow software will scale the input images from the camera to a lower resolution.
//             // This can result in lower detection accuracy at longer distances (> 55cm or 22").
//             // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
//             // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
//             // should be set to the value of the images used to create the TensorFlow Object Detection model
//             // (typically 16/9).
//            tfod.setZoom(1.0, 16 / 9.0);
//         }


            /* Declare OpMode members. */
            HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
            ElapsedTime     runtime = new ElapsedTime();


            final double     FORWARD_SPEED = 0.3;
            final double     TURN_SPEED    = 0.3;
            int markerPosition = 3;

            /*
             * Initialize the drive system variables.
             * The init() method of the hardware class does all the work here
             */
            robot.init(hardwareMap);

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Ready to run");    //
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
//        List<Recognition> updatedRecognitions = tfod.getRecognitions();
            robot.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            waitForStart();

            double currenttime = runtime.seconds();

            // For Sampling. Note: change imageSavingEnabled to see what the Detector is sampling against
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId"," id", hardwareMap.appContext.getPackageName());
            detector = new org.firstinspires.ftc.teamcode.TeamMarkerDetector(cameraMonitorViewId);

            while(opModeIsActive() && (runtime.seconds() - currenttime < 2)){
                telemetry.addData("before", "listupdate");
                telemetry.update();
                //sleep(1000);
//            updatedRecognitions = tfod.getUpdatedRecognitions();
                telemetry.addData("after","listupdate");
                telemetry.update();
                sleep(1000);

//            telemetry.addData("# Object Detected", updatedRecognitions.size());
                telemetry.update();
            }

            // Perform sampling
            samplingLocation = detector.sample(false, true);
            sleep(1);

            switch (samplingLocation) {
                case CENTER:
                    robot.frontLeft.setPower(.30);
                    robot.frontRight.setPower(.30);
                    robot.backLeft.setPower(.30);
                    robot.backRight.setPower(.30);
                    break;
                case RIGHT:
                    robot.frontLeft.setPower(.30);
                    robot.frontRight.setPower(.30);
                    robot.backLeft.setPower(.30);
                    robot.backRight.setPower(.30);
                    break;
                default:
                    robot.frontLeft.setPower(.30);
                    robot.frontRight.setPower(.30);
                    robot.backLeft.setPower(.30);
                    robot.backRight.setPower(.30);
            }



//red warhouse
        /*robot.frontLeft.setPower(.30);
        robot.frontRight.setPower(.30);
        robot.backLeft.setPower(.30);
        robot.backRight.setPower(.30);
        sleep(1300);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-TURN_SPEED);
        robot.frontRight.setPower(TURN_SPEED);
        robot.backRight.setPower(-TURN_SPEED);
        robot.backLeft.setPower(TURN_SPEED);
        sleep(2400);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);

        robot.frontLeft.setPower(.30);
        robot.frontRight.setPower(.30);
        robot.backLeft.setPower(.30);
        robot.backRight.setPower(.30);
        sleep(1000);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);

        robot.frontLeft.setPower(.30);
        robot.frontRight.setPower(.30);
        robot.backLeft.setPower(.30);
        robot.backRight.setPower(.30);
        sleep(375);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);*/
// blue warhouse
        /*robot.frontLeft.setPower(.30);
        robot.frontRight.setPower(.30);
        robot.backLeft.setPower(.30);
        robot.backRight.setPower(.30);
        sleep(1300);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(TURN_SPEED);
        robot.frontRight.setPower(-TURN_SPEED);
        robot.backRight.setPower(TURN_SPEED);
        robot.backLeft.setPower(-TURN_SPEED);
        sleep(2400);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);

        robot.frontLeft.setPower(.30);
        robot.frontRight.setPower(.30);
        robot.backLeft.setPower(.30);
        robot.backRight.setPower(.30);
        sleep(1000);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);

        robot.frontLeft.setPower(.30);
        robot.frontRight.setPower(.30);
        robot.backLeft.setPower(.30);
        robot.backRight.setPower(.30);
        sleep(375);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);*/

            //Duck spinning red
        /*robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(0.30);
        sleep(560);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(750);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        sleep(2000);

        robot.spinServo.setPower(0.4);
        sleep(2600);
        robot.spinServo.setPower(0);

        robot.frontLeft.setPower(0.40);
        robot.frontRight.setPower(-0.40);
        robot.backLeft.setPower(-0.40);
        robot.backRight.setPower(0.40);
        sleep(1200);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(250);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
//new stuff
        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(-0.30);
        sleep(725);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.shuteServo.setPower(0.3);
        sleep(1450);
        robot.shuteServo.setPower(0);

        robot.gatherServo.setPower(-0.4);
        sleep(2200);
        robot.gatherServo.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(500);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.shuteServo.setPower(-0.3);
        sleep(1450);
        robot.shuteServo.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(725);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        /*robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(700);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(960);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(0.30);
        sleep(1200);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.25);
        robot.frontRight.setPower(0.25);
        robot.backLeft.setPower(0.25);
        robot.backRight.setPower(0.25);
        sleep(420);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.spinServo.setPower(-0.4);
        sleep(2600);
        robot.spinServo.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(-0.30);
        sleep(1120);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(200);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);*/

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(0.30);
            robot.backRight.setPower(-0.30);
            sleep(700);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.frontLeft.setPower(-0.20);
            robot.frontRight.setPower(-0.20);
            robot.backLeft.setPower(-0.20);
            robot.backRight.setPower(-0.20);
            sleep(2000);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
            robot.frontLeft.setPower(0);
            sleep(2000);

            robot.spinServo.setPower(0.4);
            sleep(2600);
            robot.spinServo.setPower(0);

            robot.frontLeft.setPower(-0.40);
            robot.frontRight.setPower(0.40);
            robot.backLeft.setPower(0.40);
            robot.backRight.setPower(-0.40);
            sleep(1030);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(250);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.frontLeft.setPower(0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(0.30);
            robot.backRight.setPower(0.30);
            sleep(300);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.frontLeft.setPower(0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(0.30);
            robot.backRight.setPower(0.30);
            sleep(300);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.frontLeft.setPower(0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(0.30);
            robot.backRight.setPower(0.30);
            sleep(300);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(0.30);
            sleep(2000);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);


            robot.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            if(markerPosition == 3) {
                robot.liftLeft.setTargetPosition(-200);
                robot.liftRight.setTargetPosition(-200);

                robot.liftLeft.setPower(0.30);
                robot.liftRight.setPower(0.30);

                sleep(2020);

                robot.frontLeft.setPower(-0.30);
                robot.frontRight.setPower(-0.30);
                robot.backLeft.setPower(-0.30);
                robot.backRight.setPower(-0.30);
                sleep(730);
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);

                robot.gatherServo.setPower(-0.4);
                sleep(2200);
                robot.gatherServo.setPower(0);

                robot.liftLeft.setTargetPosition(0);
                robot.liftRight.setTargetPosition(0);

            } else if (markerPosition == 2){

                robot.liftLeft.setTargetPosition(-250);
                robot.liftRight.setTargetPosition(-250);

                robot.frontLeft.setPower(-0.30);
                robot.frontRight.setPower(-0.30);
                robot.backLeft.setPower(-0.30);
                robot.backRight.setPower(-0.30);
                sleep(80);
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);

                robot.liftLeft.setPower(0.30);
                robot.liftRight.setPower(0.30);

                sleep(2020);

                robot.frontLeft.setPower(-0.30);
                robot.frontRight.setPower(-0.30);
                robot.backLeft.setPower(-0.30);
                robot.backRight.setPower(-0.30);
                sleep(700);
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);

                robot.gatherServo.setPower(-0.4);
                sleep(2200);
                robot.gatherServo.setPower(0);

                robot.frontLeft.setPower(-0.30);
                robot.frontRight.setPower(0.30);
                robot.backLeft.setPower(-0.30);
                robot.backRight.setPower(0.30);
                sleep(500);
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);

                robot.liftLeft.setTargetPosition(0);
                robot.liftRight.setTargetPosition(0);

            }else if(markerPosition == 1){
                robot.liftLeft.setTargetPosition(-200);
                robot.liftRight.setTargetPosition(-200);

                robot.frontLeft.setPower(-0.30);
                robot.frontRight.setPower(-0.30);
                robot.backLeft.setPower(-0.30);
                robot.backRight.setPower(-0.30);
                sleep(80);
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);

                robot.liftLeft.setPower(0.30);
                robot.liftRight.setPower(0.30);

                sleep(2020);

                robot.frontLeft.setPower(-0.30);
                robot.frontRight.setPower(-0.30);
                robot.backLeft.setPower(-0.30);
                robot.backRight.setPower(-0.30);
                sleep(700);
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);

                robot.gatherServo.setPower(-0.4);
                sleep(2200);
                robot.gatherServo.setPower(0);

                robot.frontLeft.setPower(-0.30);
                robot.frontRight.setPower(0.30);
                robot.backLeft.setPower(-0.30);
                robot.backRight.setPower(0.30);
                sleep(500);
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);

                robot.liftLeft.setTargetPosition(0);
                robot.liftRight.setTargetPosition(0);

            } else if (markerPosition == 1){
                robot.liftLeft.setTargetPosition(-564);
                robot.liftRight.setTargetPosition(-564);

                robot.frontLeft.setPower(-0.30);
                robot.frontRight.setPower(-0.30);
                robot.backLeft.setPower(-0.30);
                robot.backRight.setPower(-0.30);
                sleep(60);
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);

                robot.liftLeft.setPower(0.30);
                robot.liftRight.setPower(0.30);

                sleep(2020);

                robot.frontLeft.setPower(-0.30);
                robot.frontRight.setPower(-0.30);
                robot.backLeft.setPower(-0.30);
                robot.backRight.setPower(-0.30);
                sleep(700);
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);

                robot.gatherServo.setPower(-0.4);
                sleep(2200);
                robot.gatherServo.setPower(0);

                robot.frontLeft.setPower(-0.30);
                robot.frontRight.setPower(0.30);
                robot.backLeft.setPower(-0.30);
                robot.backRight.setPower(0.30);
                sleep(500);
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);

                robot.liftLeft.setTargetPosition(0);
                robot.liftRight.setTargetPosition(0);
            }


            robot.frontLeft.setPower(0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(0.30);
            robot.backRight.setPower(0.30);
            sleep(1600);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            sleep(300);









        /*robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(-0.3);
        sleep(720);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(.30);
        robot.backRight.setPower(-0.3);
        sleep(500);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);










        /*if(updatedRecognitions.size() >= 1) {
            //tfod.setZoom(1.0, 16 / 19);
            robot.frontLeft.setPower(0.2);
            robot.frontRight.setPower(0.2);
            robot.backLeft.setPower(0.2);
            robot.backRight.setPower(0.2);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
            currenttime = runtime.seconds();
            while(opModeIsActive() && (runtime.seconds() - currenttime < 2)){
                telemetry.addData("before", "listupdate");
                telemetry.update();
                sleep(1000);
                updatedRecognitions = tfod.getUpdatedRecognitions();
                telemetry.addData("after","listupdate");
                telemetry.update();
                sleep(1000);

                telemetry.addData("# Object Detected", updatedRecognitions.size());
                telemetry.update();
            }
            if(updatedRecognitions.size() >= 1) {
                markerPosition = 2;
                telemetry.addData("Duck in", "number 2");
                telemetry.update();
            } else {
                markerPosition = 1;
                telemetry.addData("Duck in", "number 1");
                telemetry.update();
            }

        } else {
            markerPosition = 3;
            telemetry.addData("Duck in", "number 3");
            telemetry.update();
        }sleep(5000);

        /*if(markerPosition == 3){
            robot.frontLeft.setPower(0.2);
            robot.frontRight.setPower(0.2);
            robot.backLeft.setPower(0.2);
            robot.backRight.setPower(0.2);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
        }
        robot.frontLeft.setPower(0.2);
        robot.frontRight.setPower(0.2);
        robot.backLeft.setPower(0.2);
        robot.backRight.setPower(0.2);
        sleep(500);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-TURN_SPEED);
        robot.frontRight.setPower(TURN_SPEED);
        robot.backRight.setPower(TURN_SPEED);
        robot.backLeft.setPower(-TURN_SPEED);
        sleep(650);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);

        if(markerPosition == 1) {
            robot.liftLeft.setPower(-0.1);
            robot.liftRight.setPower(-0.1);
            sleep(200);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            sleep(30);

            robot.frontLeft.setPower(0.2);
            robot.frontRight.setPower(0.2);
            robot.backLeft.setPower(0.2);
            robot.backRight.setPower(0.2);
            sleep(10);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
        } else if(markerPosition == 2){
            robot.liftLeft.setPower(-0.1);
            robot.liftRight.setPower(-0.1);
            sleep(150);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            sleep(30);

            robot.frontLeft.setPower(0.2);
            robot.frontRight.setPower(0.2);
            robot.backLeft.setPower(0.2);
            robot.backRight.setPower(0.2);
            sleep(20);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
        }else {
            robot.liftLeft.setPower(-0.1);
            robot.liftRight.setPower(-0.1);
            sleep(100);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            sleep(30);

            robot.frontLeft.setPower(0.2);
            robot.frontRight.setPower(0.2);
            robot.backLeft.setPower(0.2);
            robot.backRight.setPower(0.2);
            sleep(30);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
        }

        robot.gatherServo.setPower(0.25);
        sleep(100);
        robot.gatherServo.setPower(0);

        if(markerPosition == 3){
            robot.frontLeft.setPower(0.2);
            robot.frontRight.setPower(0.2);
            robot.backLeft.setPower(0.2);
            robot.backRight.setPower(0.2);
            sleep(500);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
        }
        robot.frontLeft.setPower(0.2);
        robot.frontRight.setPower(0.2);
        robot.backLeft.setPower(0.2);
        robot.backRight.setPower(0.2);
        sleep(500);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-TURN_SPEED);
        robot.frontRight.setPower(TURN_SPEED);
        robot.backRight.setPower(TURN_SPEED);
        robot.backLeft.setPower(-TURN_SPEED);
        sleep(300);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);

        if(markerPosition == 1) {
            robot.frontLeft.setPower(-0.2);
            robot.frontRight.setPower(-0.2);
            robot.backLeft.setPower(-0.2);
            robot.backRight.setPower(-0.2);
            sleep(10);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.1);
            robot.liftRight.setPower(0.1);
            sleep(200);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            sleep(30);
        } else if(markerPosition == 2){
            robot.frontLeft.setPower(-0.2);
            robot.frontRight.setPower(-0.2);
            robot.backLeft.setPower(-0.2);
            robot.backRight.setPower(-0.2);
            sleep(20);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.1);
            robot.liftRight.setPower(0.1);
            sleep(150);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            sleep(30);
        }else {

            robot.frontLeft.setPower(-0.2);
            robot.frontRight.setPower(-0.2);
            robot.backLeft.setPower(-0.2);
            robot.backRight.setPower(-0.2);
            sleep(30);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.liftLeft.setPower(0.1);
            robot.liftRight.setPower(0.1);
            sleep(100);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            sleep(30);
        }*/












        /*if(markerPosition == 2) {
            robot.frontLeft.setPower(FORWARD_SPEED);
            robot.frontRight.setPower(FORWARD_SPEED);
            robot.backLeft.setPower(FORWARD_SPEED);
            robot.backRight.setPower(FORWARD_SPEED);
            sleep();
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
//this makes it go foward if it reconizes the maker/duck
            robot.frontLeft.setPower(-TURN_SPEED);
            robot.frontRight.setPower(TURN_SPEED);
            robot.backRight.setPower(TURN_SPEED);
            robot.backLeft.setPower(-TURN_SPEED);
            sleep(650);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);
//this turns it
            robot.frontLeft.setPower(FORWARD_SPEED);
            robot.frontRight.setPower(FORWARD_SPEED);
            robot.backLeft.setPower(FORWARD_SPEED);
            robot.backRight.setPower(FORWARD_SPEED);
            sleep(50);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
//makes it go foward
            robot.liftLeft.setPower(-0.1);
            robot.liftRight.setPower(-0.1);
            sleep(200);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            sleep(7000);

            robot.gatherServo.setPower(0.25);
            sleep(100);
            robot.gatherServo.setPower(0);

            robot.liftLeft.setPower(0.5);
            robot.liftRight.setPower(0.5);
            sleep(200);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);

        } else if(markerPosition == 1) {
            robot.frontLeft.setPower(FORWARD_SPEED);
            robot.frontRight.setPower(FORWARD_SPEED);
            robot.backLeft.setPower(FORWARD_SPEED);
            robot.backRight.setPower(FORWARD_SPEED);
            sleep(5000);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
//moves forward if it finds marker in 3
            robot.frontLeft.setPower(-TURN_SPEED);
            robot.frontRight.setPower(TURN_SPEED);
            robot.backRight.setPower(TURN_SPEED);
            robot.backLeft.setPower(-TURN_SPEED);
            sleep(650);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);

            robot.liftLeft.setPower(-0.3);
            robot.liftRight.setPower(-0.3);
            sleep(200);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            sleep(7000);

            robot.gatherServo.setPower(0.25);
            sleep(100);
            robot.gatherServo.setPower(0);

            robot.liftLeft.setPower(0.1);
            robot.liftRight.setPower(0.1);
            sleep(200);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);

        }else if(markerPosition == 3){
            robot.frontLeft.setPower(FORWARD_SPEED);
            robot.frontRight.setPower(FORWARD_SPEED);
            robot.backLeft.setPower(FORWARD_SPEED);
            robot.backRight.setPower(FORWARD_SPEED);
            sleep(100);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.frontLeft.setPower(-TURN_SPEED);
            robot.frontRight.setPower(TURN_SPEED);
            robot.backRight.setPower(TURN_SPEED);
            robot.backLeft.setPower(-TURN_SPEED);
            sleep(50);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);

            robot.liftLeft.setPower(-0.1);
            robot.liftRight.setPower(-0.1);
            sleep(200);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            sleep(7000);

            robot.gatherServo.setPower(0.25);
            sleep(100);
            robot.gatherServo.setPower(0);

            robot.liftLeft.setPower(0.1);
            robot.liftRight.setPower(0.1);
            sleep(200);
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
            //if it does not spot it in 3 it proceds like it would if its at 1
        }*\
         */





            //robot.gatherServo.setPower(0.5);
            //sleep(1200);
            //robot.gatherServo.setPower(0);


        /*}else if(updatedRecognitions.size() == 0) {
            robot.frontLeft.setPower(-TURN_SPEED);
            robot.frontRight.setPower(TURN_SPEED);
            robot.backRight.setPower(-TURN_SPEED);
            Trobot.backLeft.setPower(TURN_SPEED);
            sleep(650);*\

         */
//I FUCKING GIVE UP

            //robot.frontLeft.setPower(TURN_SPEED);
            //robot.frontRight.setPower(-TURN_SPEED);
            //robot.backRight(-TURN_SPEED);
            //robot.backLeft(TURN_SPEED);
            //sleep()

            //robot.frontLeft.setPower(-TURN_SPEED)
            //robot.frontRight.setPower(TURN_SPEED)
            //robot.backRight.setPower(-TURN_SPEED)
            //robot.backLeft.setPower(TURN_SPEED)
            //sleep()

            //robot.frontLeft.setPower(TURN_SPEED)
            //robot.frontRight.setPower(TURN_SPEED)
            //robot.backRight.setPower(TURN_SPEED)
            //robot.backLeft.setPower(TURN_SPEED);


            // Step 1:  Drive forward for 3 seconds
            //robot.frontLeft.setPower(FORWARD_SPEED);
            //robot.frontRight.setPower(FORWARD_SPEED);
            //robot.backLeft.setPower(FORWARD_SPEED);
            //robot.backRight.setPower(FORWARD_SPEED);
        /*runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Spin right for 1.3 seconds
        robot.frontLeft.setPower(TURN_SPEED);
        robot.frontRight.setPower(-TURN_SPEED);
        robot.backRight.setPower(-TURN_SPEED);
        robot.backLeft.setPower(TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3:  Drive Backwards for 1 Second
        robot.frontLeft.setPower(-FORWARD_SPEED);
        robot.frontRight.setPower(-FORWARD_SPEED);
        robot.backRight.setPower(-FORWARD_SPEED);
        robot.backLeft.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop and close the claw.
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);*/

        }
//    /**
//     * Initialize the TensorFlow Object Detection engine.
//     */
//    private void initTfod() {
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        tfodParameters.minResultConfidence = 0.75f ;
//        tfodParameters.isModelTensorFlow2 = true;
//        tfodParameters.inputSize = 320;
//        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
//    }
    }


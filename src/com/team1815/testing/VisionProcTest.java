package com.team1815.testing;

import com.team1815.robot.VisionProcessor;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.RGBImage;

/**
 *
 * @author FRC
 */
public class VisionProcTest {
    static VisionProcessor visionProcessor;
    public static void main(String[] args) {
        RGBImage positiveImage = null;
        RGBImage negativeImage = null;
        try {
            negativeImage = new RGBImage("VISION_PROC_NEGATIVE.jpg");
            positiveImage = new RGBImage("VISION_PROC_POSITIVE.jpg");
        } catch (NIVisionException ex) {
            ex.printStackTrace();
            System.exit(-1);
        }
        visionProcessor = new VisionProcessor(positiveImage, negativeImage);
        
        
        visionProcessor.robotInit();
        visionProcessor.autonomousInit();
        System.out.println("Testing positive image: ");
        visionProcessor.autonomousPeriodic(positiveImage);
        System.out.println("Testing negative image: ");
        visionProcessor.autonomousPeriodic(negativeImage);
    }
}

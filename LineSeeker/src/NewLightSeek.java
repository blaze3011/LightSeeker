// --- Brief description
// --- LightSeeker1 --- (greedy)
// - Repeat until finished
//    - Make random motor action
//    - If the action decreases light, backtrack
// --- History
// 12/03/15. LightSeeker1 implemented. See "Versions.txt"
// 12/03/15. Initial testing. Simplistic and ineffective. Lots of back & forth. Excessively stuck. 

import java.io.IOException;
import java.util.Random;

import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.device.NXTMMX;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.video.Video;
import lejos.robotics.EncoderMotor;
import lejos.robotics.RegulatedMotor;
//import lejos.utility.Delay;
//import lejos.utility.Delay;

public class NewLightSeek {

	private static final int WIDTH = 160;
    private static final int HEIGHT = 120;
    //private static final int NUM_PIXELS = WIDTH * HEIGHT;
    
    // Frames and motion maps
    private static byte [][] luminanceFrame = new byte[HEIGHT][WIDTH];
    private static int threshold = 70;
    //private static MotionMap aMotMap = new MotionMap();
    // Motors
    //private static RegulatedMotor motorB = new EV3LargeRegulatedMotor(MotorPort.B);
    private static EncoderMotor motorB = new UnregulatedMotor(MotorPort.B);
    //private static RegulatedMotor motorC = new EV3LargeRegulatedMotor(MotorPort.C);
	private static EncoderMotor motorC = new UnregulatedMotor(MotorPort.C);
	//private static float alpha = 180; // amplification for motor signals
	// Light features
	private static LightFeat aLightFeat = new LightFeat();
	private static double oldMeanLight, newMeanLight;
	// Randomness
	//private static Random randGenerator = new Random();
	private static int randDegLeft, randDegRight;
	private static int randMotor; // 0 = left; 1 = right
	
	private static float[] sample = new float[10];
    
    public NewLightSeek() {
    	// Various initializations
    	randDegLeft = 0;
    	randDegRight = 0;
		// Initialize luminance frame
    	for (int x=0; x<WIDTH; x += 1) {
    		for (int y=0; y<HEIGHT; y += 1) {
    			luminanceFrame[y][x] = 0;
    		}
    	}
	}
    
    public static void main(String[] args) throws IOException  {
         
        EV3 ev3 = (EV3) BrickFinder.getLocal();
        Video video = ev3.getVideo();
        video.open(WIDTH, HEIGHT);
        byte[] frame = video.createFrame();
        EV3ColorSensor evColour = new EV3ColorSensor(SensorPort.S2);
        Sound makeSound = null;
    	evColour.getAmbientMode().fetchSample(sample, 0);
        double mot_amplif_larger = 1.2*0.6;
        double mot_amplif_smaller = 1.2*0.3;
        double left_field = 0;
        double right_field = 0;
        int state = 0;
        
        // Grab frame
        video.grabFrame(frame);
    	// Extract luminanceFrame
        extractLuminanceValues(frame);
    	// Compute light features
        aLightFeat.compLeftRight(luminanceFrame, HEIGHT, WIDTH);
        //oldMeanLight = aLightFeat.meanTot;
    	 
        while(Button.ESCAPE.isUp()) {
        	
        	// --- Get webcam information
        	// Grab frame
        	video.grabFrame(frame);
        	// Extract luminanceFrame
        	extractLuminanceValues(frame);
        	// Compute light features
        	aLightFeat.compLeftRight(luminanceFrame, HEIGHT, WIDTH);
        	// Display
        	//System.out.println("Mean right: " + aLightFeat.meanRight);
        	//System.out.println("Mean left: " + aLightFeat.meanLeft);
        	//dispFrame();
        	
        	// Regulated motors
        	//motorB.rotate((int) (mot_amplif*(aLightFeat.meanRight/255)*180), true); // true is for immediate return -> to parallelize motors
        	//motorC.rotate((int) (mot_amplif*(aLightFeat.meanLeft/255)*180));
        	// Unregulated motors
        	
        	right_field = (aLightFeat.meanRight/255)*180;
        	left_field = (aLightFeat.meanLeft/255)*180;
        	
        	if(right_field < 10 && left_field < 10){
        		state = 0;
        	} else{
        		state = 1;
        	}
    		
//        	System.out.println("Right Field = " + right_field);
//        	System.out.println("Left Field = " + left_field);
        	// B = left motor
        	// C = right motor
        	if(state == 1 ){
        		if (right_field > left_field) {
            		left_field = right_field * mot_amplif_larger;
            		right_field = left_field * mot_amplif_smaller;
            		state = 1;
            	} else if(left_field > right_field){
            		right_field = right_field * mot_amplif_larger;
            		left_field = left_field * mot_amplif_smaller;
            		state = 1;
            	} else{
            		
            	}
	        	motorC.setPower((int) (right_field)); 
	        	motorB.setPower((int) (left_field)); 
	        	motorB.forward();
	        	motorC.forward();
        	} else if(state == 0){
        		if(left_field < 10.0 || right_field < 10.0) {
            		right_field = 60;
            		left_field= 60;
            		state = 0;
            		System.out.println("state =: " + state);
            	} else if(sample[0] > 0.20 && sample[0]< 0.26){
            		motorB.stop();
            		motorC.stop();
            		makeSound.twoBeeps();
            		
            	}else{
            		right_field = 60;
            		left_field= 60;
            		state = 0;
            	}
        		motorC.setPower((int) (right_field)); 
	        	motorB.setPower((int) (left_field)); 
	        	motorB.backward();
	        	motorC.backward();
        	}
        	/**
        	// Compute a random move
        	randDegLeft = randGenerator.nextInt(181);
        	randDegRight = randGenerator.nextInt(181);  	
        	// Make move
        	motorB.rotate(randDegRight, true); // true is for immediate return -> to parallelize motors
        	motorC.rotate(randDegLeft);
        	// Grab frame
        	video.grabFrame(frame);
        	// Extract luminanceFrame
        	extractLuminanceValues(frame);
        	// Compute light features
        	aLightFeat.compLeftRight(luminanceFrame, HEIGHT, WIDTH);
        	newMeanLight = aLightFeat.meanTot;
        	// Display mean light
        	System.out.println("Mean light: " + newMeanLight);
        	// If mean light has decreased, backtrack
        	if (newMeanLight < oldMeanLight) {
        		motorB.rotate(-randDegRight, true); // true is for immediate return -> to parallelize motors
            	motorC.rotate(-randDegLeft);
        	} else {
        		oldMeanLight = newMeanLight;
        	}
        	**/
        	        	        
        }
        video.close();
    }
    
    // DO: Improve this possibly by combining with chrominance values.
    public static void extractLuminanceValues(byte [] frame) {
    	int x,y;
    	int doubleWidth = 2*WIDTH; // y1: pos 0; u: pos 1; y2: pos 2; v: pos 3.
    	int frameLength = frame.length;
    	for(int i=0;i<frameLength;i+=2) {
    		x = (i / 2) % WIDTH;
    		y = i / doubleWidth;
    		luminanceFrame[y][x] = frame[i];
    	}
    }
    
    public static void dispFrame() {
    	for (int y=0; y<HEIGHT; y++) {
    		for (int x=0; x<WIDTH; x++) {
    			if (luminanceFrame[y][x] <= threshold) {
    				LCD.setPixel(x, y, 1);
    			}
    			else {
    				LCD.setPixel(x, y, 0);
    			}	
    		}
    	}
    	
    }
    

}



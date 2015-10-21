package EV3search;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LightLocalizer {
	private Odometer odo;
	private SampleProvider colorSensor;
	private float[] colorData;	
	private int sensorLineValue,speed,turnSpeed, initPosition;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private Navigation navi;
	private double sensorOffset,radius;
	private double[] lineAngles; 

	
	public LightLocalizer(Odometer odo, SampleProvider colorSensor, float[] colorData) {
		this.odo = odo;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
		sensorLineValue = 40;
		speed = 100;
		turnSpeed = 50;
		this.leftMotor = this.odo.getLeftMotor();
		this.rightMotor = this.odo.getRightMotor();
		this.navi = new Navigation(odo);
		sensorOffset = 14.4;
		radius = 2.1;
		lineAngles = new double[4];
		initPosition = 7;
	}
	
	public void doLocalization() {
		/*drive to location listed in tutorial*/
		
		/*
		 * since USlocalizer already position the robot roughly facing north
		 * the following method is letting the robot go forward till it seed a line
		 * and go backward for a certain distance and turn 90 degrees clockwise and do the same thing
		 * to make sure the robot is at the best position to do lightLocalizer
		 */
		while(this.getData() > sensorLineValue){		/*goes forward until it sees the line*/
			leftMotor.setSpeed(speed);
			rightMotor.setSpeed(speed);
			leftMotor.forward();
			rightMotor.forward();
		}
		navi.setSpeeds(0, 0);							/*stop the robot*/

		leftMotor.setSpeed(speed);						/*goes back to the best y position*/
		rightMotor.setSpeed(speed);
		leftMotor.rotate(-(navi.convertDistance(radius, initPosition+sensorOffset)), true);
		rightMotor.rotate(-(navi.convertDistance(radius, initPosition+sensorOffset)), false);
		
		navi.turnTo(0.0, true);							/*rotate to 0 degree*/
		
		while(this.getData() > sensorLineValue){		/*goes forward until it seed the line*/
			leftMotor.setSpeed(speed);
			rightMotor.setSpeed(speed);
			leftMotor.forward();
			rightMotor.forward();
		}
		navi.setSpeeds(0, 0);							/*stop the robot*/

		leftMotor.setSpeed(speed);						/*goes back to the best x position*/
		rightMotor.setSpeed(speed);
		leftMotor.rotate(-(navi.convertDistance(radius, initPosition+sensorOffset)), true);
		rightMotor.rotate(-(navi.convertDistance(radius, initPosition+sensorOffset)), false);
		
		
		/*start rotating and clock all 4 gridlines*/
		leftMotor.setSpeed(turnSpeed);		
		rightMotor.setSpeed(turnSpeed);
		leftMotor.backward();
		rightMotor.forward();
		
		int i = 0;
		while(i < 4){
			if (this.getData() < sensorLineValue){			/*record all 4 angles and stop rotating */
															/*after collected all of them*/
				lineAngles[i] = odo.getAng();
				i++;
				Sound.buzz();
				Delay.msDelay(500);							/*delay to make sure it won't read the line twice*/
			}
		}
		navi.setSpeeds(0, 0);								/*stop the robot*/
		
		/*do trig to compute (0,0) and 0 degrees*/
		double xOffset = 0-sensorOffset*Math.cos((((360 + lineAngles[2] - lineAngles[0])%360)/2)*Math.PI/180);
		double yOffset = 0-sensorOffset*Math.cos((((360 + lineAngles[3] - lineAngles[1])%360)/2)*Math.PI/180);
		double deltaOffset = (lineAngles[0]-180)+(((360 + lineAngles[2] - lineAngles[0])%360)/2);
				
		odo.setPosition(new double[] { xOffset, yOffset, odo.getAng()-deltaOffset }, new boolean[] { true, true, true });
		/*when done travel to (0,0) and turn to 0 degrees*/
		
		Delay.msDelay(1000);								/*delay to stable the robot*/
		navi.travelTo(0, 0);								/*travel to (0,0) and turn to 0*/
		navi.turnTo(90, true);
	}

	/*
	 * fetch light sensor data into array colorData
	 * and enlarge the number by times 100
	 */
	private float getData() {
		colorSensor.fetchSample(colorData, 0);
		float data = colorData[0]*100;
		return data;
	}
}

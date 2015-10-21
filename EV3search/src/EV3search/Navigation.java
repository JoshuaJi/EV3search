package EV3search;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation {
	//change ACCELERATION from 4000 to 1100 to reduce slipping
	final static int FAST = 200, SLOW = 100, ACCELERATION = 1100, TURNSPEED = 50;
	final static double DEG_ERR = 1.0, CM_ERR = 1.0, radius = 2.1;
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	public Navigation(Odometer odo) {
		this.odometer = odo;

		EV3LargeRegulatedMotor[] motors = this.odometer.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];

		// set acceleration
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
	}

	/*
	 * Functions to set the motor speeds jointly
	 */
	public void setSpeeds(float lSpd, float rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.backward();
		else
			this.leftMotor.forward();
		if (rSpd < 0)
			this.rightMotor.backward();
		else
			this.rightMotor.forward();
	}

	public void setSpeeds(int lSpd, int rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.backward();
		else
			this.leftMotor.forward();
		if (rSpd < 0)
			this.rightMotor.backward();
		else
			this.rightMotor.forward();
	}

	/*
	 * Float the two motors jointly
	 */
	public void setFloat() {
		this.leftMotor.stop();
		this.rightMotor.stop();
		this.leftMotor.flt(true);
		this.rightMotor.flt(true);
	}

	/*
	 * TravelTo function which takes as arguments the x and y position in cm
	 * Will travel to designated position, while constantly updating it's
	 * heading
	 */

	/*the travelTo that comes with this package is not properly performed 
	  we replaced this by our own travelTo*/
	public void travelTo(double xfinal, double yfinal) {
		double x=this.odometer.getX(); /*current positions*/
		double y=this.odometer.getY();
		double deltaX = xfinal - x; /*difference between current and final positions*/
		double deltaY = yfinal - y;
		
		double newDeg, distance;

		newDeg = calculateNewDegree(xfinal,yfinal,deltaX,deltaY); /*absolute Theta that it needs to turn to*/
		this.turnTo(newDeg*180.0/Math.PI, true);				/*since we pass a degree into this function,*/
																/*we convert it into radius*/
		distance = Math.sqrt(deltaX*deltaX + deltaY*deltaY); /*calculates distance from final position*/

		leftMotor.setSpeed(SLOW);
		rightMotor.setSpeed(SLOW);
		leftMotor.rotate(convertDistance(radius, distance), true); /*tells motor how many times each wheel should turn*/
		rightMotor.rotate(convertDistance(radius, distance), false); /*convertDistance is converting distance to*/
																		   /* angle (number of rotations)*/
		}

	/*
	 * TurnTo function which takes an angle and boolean as arguments The boolean
	 * controls whether or not to stop the motors when the turn is completed
	 */

	public void turnTo(double angle, boolean stop) {

		double error = angle - this.odometer.getAng();

		while (Math.abs(error) > DEG_ERR) {

			error = angle - this.odometer.getAng();

			if (error < -180.0) {
				this.setSpeeds(-TURNSPEED, TURNSPEED);
			} else if (error < 0.0) {
				this.setSpeeds(TURNSPEED, -TURNSPEED);
			} else if (error > 180.0) {
				this.setSpeeds(TURNSPEED, -TURNSPEED);
			} else {
				this.setSpeeds(-TURNSPEED, TURNSPEED);
			}
		}

		if (stop) {
			this.setSpeeds(0, 0);
		}
	}

	public int convertAngle(double radius, double width, double angle) { /*converts rotations of robot to*/
		return convertDistance(radius, Math.PI * width * angle / (2 * Math.PI)); /*the required angle rotation of*/
																					/*the wheel*/
	}

	public int convertDistance(double radius, double distance) { /*converts distance to angle*/
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	/* Based on current and final positions, what is the absolute angle of the final position*/
	public double calculateNewDegree(double x, double y, double dx, double dy){ 
		if (dx >= 0) {
			if (dy >= 0) {
				return Math.atan(Math.abs(dx) / Math.abs(dy));
			} else {
				return Math.PI/2 + Math.atan(Math.abs(dy) / Math.abs(dx));
			}
		} else {
			if (dy >= 0) {
				return Math.PI*3/2 + Math.atan(Math.abs(dy) / Math.abs(dx));
			} else {
				return Math.PI + Math.atan(Math.abs(dx) / Math.abs(dy));
			}
		}
	}

	/*
	 * Go foward a set distance in cm
	 */
	public void goForward(double distance) {
		leftMotor.rotate(convertDistance(radius, distance), true); 
		rightMotor.rotate(convertDistance(radius, distance), false);
	}
}

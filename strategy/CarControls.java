package strategy;

public class CarControls {
	private float throttle;
	private float steering;
	private float brake;

	public CarControls() {
		this.throttle = 0.0f;
		this.steering = 0.0f;
		this.brake = 0.0f;
	}

	public CarControls(float throttle, float steering, float brake) {
		super();
		this.throttle = throttle;
		this.steering = steering;
		this.brake = brake;
	}

	public void setThrottle(float throttle) {
		this.throttle = throttle;
	}

	public void setSteering(float steering) {
		this.steering = steering;
	}

	public void setBrake(float brake) {
		this.brake = brake;
	}

	public float getThrottle() {
		return throttle;
	}

	public float getSteering() {
		return steering;
	}

	public float getBrake() {
		return brake;
	}
}

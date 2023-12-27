import DrivingInterface.*;
import strategy.CarControls;
import strategy.DrivingPathStrategy2;
import strategy.DrivingStrategy;
import strategy.EmergencyStrategy;
import strategy.MovingStrategy;
import strategy.NormalStrategy;

public class MyCar {
	
	int accident_count =0;
    float accident_time=0;
    boolean is_accident;
    private int recovery_count;

	// 멤버 변수
	boolean is_debug = true;
	static boolean enable_api_control = true;

	private boolean isAccident(DrivingInterface.DrivingInterface.CarStateValues sensing_info){
		if(sensing_info.lap_progress > 0.5 && !is_accident && (sensing_info.speed < 1.0 && sensing_info.speed > -1.0)){
			accident_count+=1;
			System.out.println("충돌!!!!!!!!!!!!!!!!!!");
		}
		else {
			accident_count = 0;
		}

		if(accident_count > 6){ // 차량이 멈췄다고 판단
			is_accident=true;
		}

		recovery_count += 1;

		// 충돌 회복
		if(recovery_count > 20){
			is_accident=false;
			recovery_count=0;
			accident_count=0;
		}
		return is_accident;
	}

	public void control_driving(boolean a1, float a2, float a3, float a4, float a5, float a6, float a7, float a8,
			float[] a9, float[] a10, float[] a11, float[] a12) {

		// ===========================================================
		// Don't remove this area. ===================================
		// ===========================================================
		DrivingInterface di = new DrivingInterface();
		DrivingInterface.CarStateValues sensing_info = di.get_car_state(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11,
				a12);
		// ===========================================================

		if (is_debug) {
			System.out.println("=========================================================");
			System.out.println("[MyCar] to middle: " + sensing_info.to_middle);

			System.out.println("[MyCar] collided: " + sensing_info.collided);
			System.out.println("[MyCar] car speed: " + sensing_info.speed + "km/h");

			System.out.println("[MyCar] is moving forward: " + sensing_info.moving_forward);
			System.out.println("[MyCar] moving angle: " + sensing_info.moving_angle);
			System.out.println("[MyCar] lap_progress: " + sensing_info.lap_progress);

			StringBuilder forward_angles = new StringBuilder("[MyCar] track_forward_angles: ");
			for (Float track_forward_angle : sensing_info.track_forward_angles) {
				forward_angles.append(track_forward_angle).append(", ");
			}
			System.out.println(forward_angles);

			StringBuilder to_way_points = new StringBuilder("[MyCar] distance_to_way_points: ");
			for (Float distance_to_way_point : sensing_info.distance_to_way_points) {
				to_way_points.append(distance_to_way_point).append(", ");
			}
			System.out.println(to_way_points);

			StringBuilder forward_obstacles = new StringBuilder("[MyCar] track_forward_obstacles: ");
			for (DrivingInterface.ObstaclesInfo track_forward_obstacle : sensing_info.track_forward_obstacles) {
				forward_obstacles.append("{dist:").append(track_forward_obstacle.dist).append(", to_middle:")
						.append(track_forward_obstacle.to_middle).append("}, ");
			}
			System.out.println(forward_obstacles);

			StringBuilder opponent_cars = new StringBuilder("[MyCar] opponent_cars_info: ");
			for (DrivingInterface.CarsInfo carsInfo : sensing_info.opponent_cars_info) {
				opponent_cars.append("{dist:").append(carsInfo.dist).append(", to_middle:").append(carsInfo.to_middle)
						.append(", speed:").append(carsInfo.speed).append("km/h}, ");
			}
			System.out.println(opponent_cars);

			System.out.println("=========================================================");
		}

		// ===========================================================
		// Area for writing code about driving rule ==================
		// ===========================================================
		// Editing area starts from here
		//
		
		// 충돌로 인해 멈춘 상태인가 확인


		// 차량 상태 평가 및 전략 선택
		DrivingStrategy currentStrategy;
		if (!isAccident(sensing_info)) {
			currentStrategy = new DrivingPathStrategy2();
		} else {
			currentStrategy = new EmergencyStrategy();
		}
		


		// 선택된 전략 적용
		CarControls result = currentStrategy.applyDrivingStrategy(sensing_info);

		if (is_debug) {
			System.out.println("[MyCar] steering:" + car_controls.steering + ", throttle:" + car_controls.throttle
					+ ", brake:" + car_controls.brake
					+ ", middle:" + sensing_info.to_middle + ", .moving_angle:" + sensing_info.moving_angle
					+ ", speed:" + sensing_info.speed);
		}
		
		car_controls.throttle = result.getThrottle();
		car_controls.steering = result.getSteering();
		car_controls.brake = result.getBrake();
		

		//
		// Editing area ends
		// =======================================================
	}


	// ===========================================================
	// Don't remove below area. ==================================
	// ===========================================================
	public native int StartDriving(boolean enable_api_control);

	static MyCar car_controls;

	float throttle;
	float steering;
	float brake;

	static {
		System.loadLibrary("DrivingInterface/DrivingInterface");
	}



	public static void main(String[] args) {
		System.out.println("[MyCar] Start Bot! (JAVA)");

		car_controls = new MyCar();
		int return_code = car_controls.StartDriving(enable_api_control);

		System.out.println("[MyCar] End Bot! (JAVA), return_code = " + return_code);

		System.exit(return_code);
	}
	// ===========================================================
}

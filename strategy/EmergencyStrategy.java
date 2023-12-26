package strategy;
import DrivingInterface.DrivingInterface.CarStateValues;

public class EmergencyStrategy implements DrivingStrategy {
	
    int accident_count =0;
    float accident_time=0;
    boolean is_accident;
    private int recovery_count;
    
	@Override
	public CarControls applyDrivingStrategy(CarStateValues sensing_info) {
		// 차량 핸들 조정을 위해 참고할 전방의 커브 값 가져오기
		int angle_num = (int) (sensing_info.speed / 45);
		float ref_angle = angle_num > 0 ? sensing_info.track_forward_angles.get(angle_num) : 0;
		
		// 전방의 커브 각도에 따라 throttle 값을 조절하여 속도를 제어함
		float throttle_factor = 0.9f / (Math.abs(ref_angle) + 0.1f);		
		float set_throttle = 0.9f + throttle_factor;
		
		// 차량의 Speed 에 따라서 핸들을 돌리는 값을 조정함
		float steer_factor = sensing_info.speed * 1.6f;
		
		// (참고할 전방의 커브 - 내 차량의 주행 각도) / (계산된 steer factor) 값으로 steering 값을 계산
		float set_steering = (ref_angle - sensing_info.moving_angle) / (steer_factor + 0.001f);
		
		float set_brake = 0.0f;
		
        set_steering = 0;
        set_brake = 0;
        set_throttle = -1;
        recovery_count += 1;
        System.out.println("후진...");
        System.out.println(sensing_info.track_forward_obstacles);

        return new CarControls(set_throttle, set_steering, set_brake);
	}
}

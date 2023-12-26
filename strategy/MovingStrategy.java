package strategy;

import java.util.ArrayList;
import java.util.Comparator;

import DrivingInterface.DrivingInterface;
import DrivingInterface.DrivingInterface.CarStateValues;

public class MovingStrategy implements DrivingStrategy {
	float coolliedTime = 0;
	int accident_count = 0;
	float accident_time = 0;
	boolean is_accident;
	private int recovery_count;

	@Override
	public CarControls applyDrivingStrategy(CarStateValues sensing_info) {

		// 도로의 실제 폭의 1/2 로 계산됨
		float half_load_width = sensing_info.half_road_limit - 1.25f;

		// 차량 핸들 조정을 위해 참고할 전방의 커브 값 가져오기
		int angle_num = (int) (sensing_info.speed / 45);
		float ref_angle = angle_num > 0 ? sensing_info.track_forward_angles.get(angle_num) : 0;

		// 차량의 차선 중앙 정렬을 위한 미세 조정 값 계산
		float middle_add = ((sensing_info.to_middle - 1.8f) / 80) * -1;

		// 전방의 커브 각도에 따라 throttle 값을 조절하여 속도를 제어함
		float throttle_factor = 0.9f / (Math.abs(ref_angle) + 0.1f);
		if (throttle_factor > 0.11f)
			throttle_factor = 0.11f; // throttle 값을 최대 0.81 로 설정
		float set_throttle = 0.9f + throttle_factor;
		if (sensing_info.speed < 60)
			set_throttle = 1.0f; // 속도가 60Km/h 이하인 경우 0.9 로 설정
		if (sensing_info.speed > 100)
			set_throttle = 1.0f; // 최대속도를 80km/h로 설정
		if (sensing_info.speed > 150)
			set_throttle = 0.97f; // 최대속도를 80km/h로 설정

// 차량의 Speed 에 따라서 핸들을 돌리는 값을 조정함
		float steer_factor = sensing_info.speed * 1.6f;
		if (sensing_info.speed > 70)
			steer_factor = sensing_info.speed * 0.85f;
		if (sensing_info.speed > 120)
			steer_factor = sensing_info.speed * 0.62f;

//// 긴급 및 예외 상황 처리 ////////////////////////////////////////////////////////////////////////////////////////
		boolean full_throttle = true;
		boolean emergency_brake = false;

// 전방 커브의 각도가 큰 경우 속도를 제어함
// 차량 핸들 조정을 위해 참고하는 커브 보다 조금 더 멀리 참고하여 미리 속도를 줄임
		int road_range = (int) (sensing_info.speed / 30);
		for (int i = 0; i < road_range; i++) {
			float fwd_angle = Math.abs(sensing_info.track_forward_angles.get(i));
			if (fwd_angle > 60) { // 커브가 45도 이상인 경우 brake, throttle 을 제어
				full_throttle = false;
			}
			if (fwd_angle > 80) { // 커브가 80도 이상인 경우 steering 까지 추가로 제어
				emergency_brake = true;
				break;
			}
		}

// brake, throttle 제어
		float set_brake = 0.0f;
		if (!full_throttle) {
			if (sensing_info.speed > 115) {
				set_brake = 0.1f;
			}
			if (sensing_info.speed > 135) {
				set_throttle = 0.8f;
				set_brake = 0.3f;
			}
			if (sensing_info.speed > 145) {
				set_throttle = 0.7f;
				set_brake = 0.4f;
			}
			if (sensing_info.speed > 155) {
				set_throttle = 0.55f;
				set_brake = 0.7f;
			}
		} else {
			if (sensing_info.speed > 155) {
				set_throttle = 0.81f;
			}
			if (sensing_info.speed > 164) {
				set_throttle = 0.65f;
				set_brake = 0.2f;
			}
			if (sensing_info.speed > 166) {
				set_throttle = 0.5f;
				set_brake = 0.3f;
			}
			if (sensing_info.speed > 168) {
				set_throttle = 0.4f;
				set_brake = 0.7f;
			}
		}
		
		// 튜토리얼 장애물회피 개조
		if (sensing_info.track_forward_obstacles.size() > 0) {
			float avoid_width = 3.1f;
			int count = 0;
			DrivingInterface.ObstaclesInfo fwd_obstacle = sensing_info.track_forward_obstacles.get(0);
			// 장애물 중에 내 경로상에 존재하는 장애물 찾기
			for (DrivingInterface.ObstaclesInfo fwd_obstacles : sensing_info.track_forward_obstacles) {
				if (fwd_obstacles.dist < 100 && fwd_obstacles.dist > 0
						&& Math.abs(fwd_obstacles.to_middle) < sensing_info.half_road_limit) {
					if (Math.abs(fwd_obstacles.to_middle - sensing_info.to_middle) < avoid_width) {
						fwd_obstacle = fwd_obstacles;
						break;
					}
				}
			}
			
			int left = 0;
			int right = 0;
			ArrayList<Float> midArr = new ArrayList<>();			
			
			// 경로상 존재하는 장애물이 여러개가 붙어있는 장애물인지 확인
			for (DrivingInterface.ObstaclesInfo fwd_obstacles : sensing_info.track_forward_obstacles) {
				// 장애물 리스트 거리랑 위에서 구한 경로 상에 존재하는 장애물 거리
				if (Math.abs(fwd_obstacle.dist - fwd_obstacles.dist) > 1) continue;
				
				midArr.add(fwd_obstacles.to_middle);
			}
			
			midArr.sort(Comparator.naturalOrder());
			
			// 내 정면 장애물 왼쪽에 몇개의 장애물이 붙어있는지
			if (midArr.indexOf(fwd_obstacle.to_middle) == 0) {
				left = 0;
			}
			else {
				int index = midArr.indexOf(fwd_obstacle.to_middle);
				while(true) {
					if (index == 0) break;
					
					// 정면 장애물과 왼쪽 장애물 사이의 간격이 차가 지나갈 수 있는 간격인가 확인
					if (Math.abs(midArr.get(index) - midArr.get(index - 1)) < avoid_width) {
						index--;
						left++;
					}
					else break;
				}
			}
			
			// 내 정면 장애물 오른쪽에 몇개의 장애물이 붙어있는지
			if (midArr.indexOf(fwd_obstacle.to_middle) == midArr.size() - 1) {
				right = 0;
			}
			else {
				int index = midArr.indexOf(fwd_obstacle.to_middle);
				while(true) {
					if (index == midArr.size() - 1) break;
					
					// 정면 장애물과 오른쪽 장애물 사이의 간격이 차가 지나갈 수 있는 간격인가 확인
					if (Math.abs(midArr.get(index) - midArr.get(index + 1)) < avoid_width) {
						index++;
						right++;
					}
					else break;
				}
			}
			
			if (midArr.size() > 0) {
				float diff = fwd_obstacle.to_middle - sensing_info.to_middle;
				if (Math.abs(diff) < avoid_width) {
					// 장애물이 1개
					if (midArr.size() == 1) {				
						ref_angle = (float) (Math.abs(Math.atan((diff - avoid_width) / fwd_obstacle.dist) * 57.29579));
						middle_add = 0;
						if (diff > 0)
							ref_angle *= -1;			
					}
					
					// 장애물이 1개 이상
					else if (midArr.size() > 1) {
						ref_angle = (float) (Math.abs(Math.atan((diff - (avoid_width * (Math.min(left, right) + 1))) / fwd_obstacle.dist) * 57.29579));
						middle_add = 0;
						// 오른쪽에 붙어있는 장애물이 더 적다. 우회전
						if (left > right) {
							ref_angle *= 1;
						}						
						else {
							ref_angle *= -1;
						}
					}
				}

			}
			
			

		}
// (참고할 전방의 커브 - 내 차량의 주행 각도) / (계산된 steer factor) 값으로 steering 값을 계산
		float set_steering = (ref_angle - sensing_info.moving_angle) / (steer_factor + 0.001f);

// 차선 중앙정렬 값을 추가로 고려함
		set_steering += middle_add * 1.1f;

// steering 까지 추가로 제어
		if (emergency_brake) {
			if (set_steering > 0) {
				set_steering += 0.32f;
			} else {
				set_steering -= 0.32f;
			}
		}		

		return new CarControls(set_throttle, set_steering, set_brake);
	}
}

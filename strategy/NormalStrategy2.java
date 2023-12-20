package strategy;

import java.util.ArrayList;

import DrivingInterface.DrivingInterface;
import DrivingInterface.DrivingInterface.CarStateValues;
import pid.PIDController;
import strategy.DrivingPathStrategy.PathNode;

public class NormalStrategy2 implements DrivingStrategy {
	float coolliedTime = 0;
	int accident_count = 0;
	float accident_time = 0;
	boolean is_accident;
	private int recovery_count;
	float previousGoalSteer;

	private float getDistance(PathNode node1, PathNode node2) {
		float dx = node1.x - node2.x;
		float dy = node1.y - node2.y;
		return (float) Math.sqrt(dx * dx + dy * dy);
	}

	class PathNode {
		float x;
		float y;
		float pivot;

		float cost;
		float first;

		public PathNode() {
			cost = Float.MAX_VALUE;
		}
	}

	@Override
	public CarControls applyDrivingStrategy(CarStateValues sensing_info) {

		// 도로의 실제 폭의 1/2 로 계산됨
		float half_load_width = sensing_info.half_road_limit - 1.25f;

		// 차량 핸들 조정을 위해 참고할 전방의 커브 값 가져오기
		int angle_num = (int) (sensing_info.speed / 45);
		float ref_angle = angle_num > 0 ? sensing_info.track_forward_angles.get(0) : 0;

		// 차량의 차선 중앙 정렬을 위한 미세 조정 값 계산
		ArrayList<Float> trackForwardAngles = sensing_info.track_forward_angles;
		final float roadWidth = 5;
		final float trackDistance = 10;

		ArrayList<ArrayList<PathNode>> nodes = new ArrayList<>();

		float currX = 0;
		float currY = 0;

		PathNode start = new PathNode();
		start.x = 0;
		start.y = 0;

		for (int i = 0; i < trackForwardAngles.size(); i++) {
			float trackAngleRadians = (float) Math.toRadians(trackForwardAngles.get(i)); // 라디안으로 한 번만 변환

			ArrayList<PathNode> layer = new ArrayList<>();

			for (float j = -roadWidth + 1; j < roadWidth; j += 2) {
				PathNode node = new PathNode();

				float distance = (float) Math.sqrt(trackDistance * trackDistance + j * j);
				float nodeAngleRadians = (float) Math.atan2(j, trackDistance);

				node.x = currX + distance * (float) Math.cos(trackAngleRadians + nodeAngleRadians);
				node.y = currY + distance * (float) Math.sin(trackAngleRadians + nodeAngleRadians);
				node.pivot = j;

				if (i == 0) {
					node.first = j;
					node.cost = getDistance(node, start);
				} else {
					float first = 0;
					float min = Float.MAX_VALUE;

					for (PathNode n : nodes.get(nodes.size() - 1)) {
						float cost = getDistance(node, n) + n.cost;

						if (cost < min) {
							min = cost;
							first = n.first;
						}
					}

					node.first = first;
					node.cost = min;
				}

				// System.out.println("(" + node.x + ", " + node.y + "),");
				System.out.println("node[" + i + "][" + j + "](" + node.x + ", " + node.y + ", " + node.first + "),");

				layer.add(node);
			}

			currX += trackDistance * (float) Math.cos(trackAngleRadians);
			currY += trackDistance * (float) Math.sin(trackAngleRadians);

			nodes.add(layer);
		}

		float middle_add = ((nodes.get(19).get(3).first - sensing_info.to_middle) / 80);

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
			DrivingInterface.ObstaclesInfo fwd_obstacle = sensing_info.track_forward_obstacles.get(0);
			if (sensing_info.track_forward_obstacles.size() > 1
					&& Math.abs(sensing_info.track_forward_obstacles.get(0).dist
							- sensing_info.track_forward_obstacles.get(1).dist) < 1.5
					&& sensing_info.track_forward_obstacles.get(0).to_middle > sensing_info.track_forward_obstacles
							.get(1).to_middle)
				fwd_obstacle = sensing_info.track_forward_obstacles.get(1);
			if (fwd_obstacle.dist < 100 && fwd_obstacle.dist > 0
					&& Math.abs(fwd_obstacle.to_middle) < sensing_info.half_road_limit) {

				float avoid_width = 2.7f;
				float diff = fwd_obstacle.to_middle - sensing_info.to_middle;
				if (Math.abs(diff) < avoid_width) {
					ref_angle = (float) (Math.abs(Math.atan((diff - avoid_width) / fwd_obstacle.dist) * 57.29579));
					middle_add = 0;
					if (diff > 0)
						ref_angle *= -1;
				}
			}
		}
// (참고할 전방의 커브 - 내 차량의 주행 각도) / (계산된 steer factor) 값으로 steering 값을 계산
		float set_steering = (ref_angle - sensing_info.moving_angle) / (steer_factor + 0.001f);

// 차선 중앙정렬 값을 추가로 고려함
		set_steering += middle_add * 1.1f;

		double kp = 0.7;
		double ki = 0.2;
		double kd = 0.1;
		PIDController pidController = new PIDController(kp, ki, kd);

		
		float pidOut = (float) pidController.calculatePIDOutput(set_steering, previousGoalSteer);
		
		previousGoalSteer = set_steering;
		
		set_steering = pidOut;
				
// steering 까지 추가로 제어
		if (emergency_brake) {
			if (set_steering > 0) {
				set_steering += 0.32f;
			} else {
				set_steering -= 0.32f;
			}
		}
		// 튜토리얼 충돌처리
		if (sensing_info.lap_progress > 0.5 && (sensing_info.speed < 1.0 && sensing_info.speed > -1.0)) {
			accident_count += 1;

		}

		if (accident_count > 6) {
			is_accident = true;
		}

		if (is_accident) {
			set_steering = 0.2f;
			set_brake = 0;
			set_throttle = -1;
			recovery_count += 1;
		}

		if (recovery_count > 30) {
			is_accident = false;
			recovery_count = 0;
			accident_count = 0;
			set_throttle = 0;
			set_brake = 0;
			set_steering = 0;
		}

		return new CarControls(set_throttle, set_steering, set_brake);
	}
}

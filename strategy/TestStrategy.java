package strategy;

import DrivingInterface.DrivingInterface;
import DrivingInterface.DrivingInterface.CarStateValues;
import pid.PIDController;


public class TestStrategy implements DrivingStrategy {
    float coolliedTime = 0;
    int accident_count = 0;
    float accident_time = 0;
    private double previousError;
    boolean is_accident;
    private int recovery_count;
    private boolean is_debug_pid=true;
    private float previousGoalSteer=0;

    @Override
    public CarControls applyDrivingStrategy(CarStateValues sensing_info) {
        double kp = 0.5;
        double ki = 0.3;
        double kd = 0.9;

        PIDController pidController = new PIDController(kp, ki, kd);
        // 도로의 실제 폭의 1/2 로 계산됨
        float half_load_width = sensing_info.half_road_limit - 1.25f;

        // 차량 핸들 조정을 위해 참고할 전방의 커브 값 가져오기
        int angle_num = (int) (sensing_info.speed / 36);
        float ref_angle = angle_num > 0 ? sensing_info.track_forward_angles.get(angle_num) : 0;

        // 차량의 차선 중앙 정렬을 위한 미세 조정 값 계산
        float middle_add = ((sensing_info.to_middle - 1.25f) / 80) * -1;


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
        int road_range = (int) (sensing_info.speed / 10);
        float fwd_angle = 0;
        for (int i = 0; i < road_range; i++) {
            fwd_angle = Math.abs(sensing_info.track_forward_angles.get(i));
            if (fwd_angle > 34) { // 커브가 45도 이상인 경우 brake, throttle 을 제어
                full_throttle = false;
            }
            if (fwd_angle > 50&&sensing_info.speed>100) { // 커브가 80도 이상인 경우 steering 까지 추가로 제어
                emergency_brake = true;
                break;
            }
        }


//        float waypoint_diff=sensing_info.distance_to_way_points.get(0)-sensing_info.to_middle;
        float test_ref_angle=(float) (Math.abs(Math.atan((sensing_info.to_middle) / sensing_info.distance_to_way_points.get((int) (sensing_info.speed/45))) * 57.29579));
        if(sensing_info.to_middle>0) test_ref_angle*=-1;
        System.out.println(test_ref_angle);
        float goalSteeringValue = 0;//(ref_angle - sensing_info.moving_angle) / (steer_factor + 0.001f);
        goalSteeringValue=(test_ref_angle- sensing_info.moving_angle)/180;
// 차선 중앙정렬 값을 추가로 고려함
        goalSteeringValue += middle_add;
        System.out.println(goalSteeringValue);


        float fwd_waypoint=sensing_info.distance_to_way_points.get(1);
        goalSteeringValue = goalSteeringValue > 1 ? 1 : goalSteeringValue < -1 ? -1 : goalSteeringValue;
        System.out.println(goalSteeringValue);
        float pidOut = (float) pidController.calculatePIDOutput(goalSteeringValue, previousGoalSteer);
        if(false){
            pidOut=(float) pidController.calculatePDOutput(goalSteeringValue, previousGoalSteer);
        }
        float set_steering = pidOut;
        printPIDOutput(pidOut);
        float ref_speed = (set_steering < 0.05 ? 0 : Math.abs(pidOut * 2) * (sensing_info.speed / 100));
        previousGoalSteer = set_steering;
        float set_brake = 0;//!full_throttle? 0.2f+(ref_speed * 3) :ref_speed/2;
        if(emergency_brake){
            set_brake+=Math.abs(pidOut * 2);
            set_steering=0;
        }
        float set_throttle = 0.9f;
        if(sensing_info.speed>90){
            set_throttle=0.7f;
        }

        return new CarControls(set_throttle, set_steering, set_brake);
    }

    private void printPIDOutput(double pidOut) {
        System.out.println("pidOut:"+ pidOut);
    }
}

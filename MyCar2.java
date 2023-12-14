import DrivingInterface.*;

public class MyCar2 {

    boolean is_debug = true;
    float coolliedTime=0;
    int accident_count =0;
    float accident_time=0;
    static boolean enable_api_control = true; // true(Controlled by code) /false(Controlled by keyboard)
    boolean is_accident;
    private int recovery_count;

    public void control_driving(boolean a1, float a2, float a3, float a4, float a5, float a6, float a7, float a8,
                                float[] a9, float[] a10, float[] a11, float[] a12) {

        // ===========================================================
        // Don't remove this area. ===================================
        // ===========================================================
        DrivingInterface di = new DrivingInterface();
        DrivingInterface.CarStateValues sensing_info = di.get_car_state(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12);
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
                forward_obstacles.append("{dist:").append(track_forward_obstacle.dist)
                        .append(", to_middle:").append(track_forward_obstacle.to_middle).append("}, ");
            }
            System.out.println(forward_obstacles);

            StringBuilder opponent_cars = new StringBuilder("[MyCar] opponent_cars_info: ");
            for (DrivingInterface.CarsInfo carsInfo : sensing_info.opponent_cars_info) {
                opponent_cars.append("{dist:").append(carsInfo.dist)
                        .append(", to_middle:").append(carsInfo.to_middle)
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

        // 도로의 실제 폭의 1/2 로 계산됨
        float half_load_width = sensing_info.half_road_limit - 1.25f;

// 차량 핸들 조정을 위해 참고할 전방의 커브 값 가져오기
        int angle_num = (int)(sensing_info.speed / 45);
        float ref_angle = angle_num > 0 ? sensing_info.track_forward_angles.get(angle_num) : 0;

// 차량의 차선 중앙 정렬을 위한 미세 조정 값 계산
        float middle_add = ((sensing_info.to_middle-1.8f) / 80) * -1;

// 전방의 커브 각도에 따라 throttle 값을 조절하여 속도를 제어함
        float throttle_factor = 0.9f / (Math.abs(ref_angle) + 0.1f);
        if (throttle_factor > 0.11f) throttle_factor = 0.11f;   // throttle 값을 최대 0.81 로 설정
        float set_throttle = 0.9f + throttle_factor;
        if (sensing_info.speed < 60) set_throttle = 1.0f;  // 속도가 60Km/h 이하인 경우 0.9 로 설정
        if (sensing_info.speed > 100) set_throttle = 1.0f;  // 최대속도를 80km/h로 설정
        if (sensing_info.speed > 150) set_throttle = 0.97f;  // 최대속도를 80km/h로 설정

// 차량의 Speed 에 따라서 핸들을 돌리는 값을 조정함
        float steer_factor = sensing_info.speed * 1.6f;
        if (sensing_info.speed > 70) steer_factor = sensing_info.speed * 0.85f;
        if (sensing_info.speed > 120) steer_factor = sensing_info.speed * 0.62f;


//// 긴급 및 예외 상황 처리 ////////////////////////////////////////////////////////////////////////////////////////
        boolean full_throttle = true;
        boolean emergency_brake = false;

// 전방 커브의 각도가 큰 경우 속도를 제어함
// 차량 핸들 조정을 위해 참고하는 커브 보다 조금 더 멀리 참고하여 미리 속도를 줄임
        int road_range = (int)(sensing_info.speed / 30);
        for (int i = 0; i < road_range; i++) {
            float fwd_angle = Math.abs(sensing_info.track_forward_angles.get(i));
            if (fwd_angle > 60) {   // 커브가 45도 이상인 경우 brake, throttle 을 제어
                full_throttle = false;
            }
            if (fwd_angle > 80) {   // 커브가 80도 이상인 경우 steering 까지 추가로 제어
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
        }
        else {
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
// 충돌 조건
//        boolean isColliedWidth = sensing_info.collided ||half_load_width - Math.abs(sensing_info.to_middle) < 1f;
//        if((sensing_info.collided || coolliedTime>0)&&coolliedTime<15f){
//            System.out.println("어 이게 왜?");
//            if( sensing_info.speed<100 &&isColliedWidth) {
//                if(accident_time>0){
//
//                    accident_count+=1;
//                    if(accident_count>10){
//                        coolliedTime=0;
//                    }
//                }else{
//                    accident_count=0;
//                }
//                accident_time=5.5f;
//                coolliedTime += 10f;
//            }
//
//            if(coolliedTime>0.5f) {
//                set_throttle = -0.9f;
//                coolliedTime-=0.1f;
//                if(sensing_info.speed<0){
//
//                    ref_angle*=-1;
//                }
//            }
//        }else{
//            if(coolliedTime>10f  ) {
//                set_throttle = 1f;
//            }
//                coolliedTime-=0.1f;
//        }
//        if(accident_time>0){
//            accident_time-=0.1f;
//        }else{
//            accident_count=0;
//        }



        //튜토리얼 장애물회피 개조
        if(sensing_info.track_forward_obstacles.size()>0){
            DrivingInterface.ObstaclesInfo fwd_obstacle = sensing_info.track_forward_obstacles.get(0);
            if(sensing_info.track_forward_obstacles.size()>1&&Math.abs(sensing_info.track_forward_obstacles.get(0).dist-sensing_info.track_forward_obstacles.get(1).dist)<1.5&&sensing_info.track_forward_obstacles.get(0).to_middle>sensing_info.track_forward_obstacles.get(1).to_middle)
                fwd_obstacle = sensing_info.track_forward_obstacles.get(1);
            if(fwd_obstacle.dist<100 && fwd_obstacle.dist>0 && Math.abs(fwd_obstacle.to_middle)<sensing_info.half_road_limit){

                float avoid_width=2.7f;
                float diff = fwd_obstacle.to_middle - sensing_info.to_middle;
                if(Math.abs(diff)<avoid_width){
                    ref_angle = (float) (Math.abs(Math.atan((diff-avoid_width)/fwd_obstacle.dist) *57.29579));
                    middle_add=0;
                    if(diff>0) ref_angle *=-1;
                }
            }
        }
// (참고할 전방의 커브 - 내 차량의 주행 각도) / (계산된 steer factor) 값으로 steering 값을 계산
        float set_steering = (ref_angle - sensing_info.moving_angle) / (steer_factor + 0.001f);

// 차선 중앙정렬 값을 추가로 고려함
        set_steering += middle_add*1.1f;

// steering 까지 추가로 제어
        if (emergency_brake) {
            if (set_steering > 0) {
                set_steering += 0.32f;
            }
            else {
                set_steering -= 0.32f;
            }
        }
        //튜토리얼 충돌처리
        if(sensing_info.lap_progress>0.5 && (sensing_info.speed<1.0 && sensing_info.speed>-1.0)){
            accident_count+=1;

        }
        if(accident_count>6){
            is_accident=true;
        }
        if(is_accident){
            set_steering=0.2f;
            set_brake=0;
            set_throttle=-1;
            recovery_count+=1;
        }
        if(recovery_count>30){
            is_accident=false;
            recovery_count=0;
            accident_count=0;
            set_throttle=0;
            set_brake=0;
            set_steering=0;
        }

//        cout << "[내차] 설정 변수 상황:" << "set_steering : "
//                << set_steering << ", set_throttle : "<< set_throttle
//                << ", set_brake :"<< set_brake
//                << ", full_throttle : "<< full_throttle << endl;


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        car_controls.steering = set_steering;
        car_controls.throttle = set_throttle;
        car_controls.brake = set_brake;

        if (is_debug) {
            System.out.println("[MyCar] steering:" + car_controls.steering + ", accident_count:" + accident_count +
                    ", throttle:" + car_controls.throttle + ", brake:" + car_controls.brake);
        }

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

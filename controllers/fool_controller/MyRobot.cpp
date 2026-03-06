#include "MyRobot.h"

// ─────────────────────────────────────────────────────────────────────────────
MyRobot::MyRobot() : Robot() {
    _time_step   = 64;
    _left_speed  = 0;
    _right_speed = 0;

    _x = _y = _theta = 0.0f;
    _sr = _sl = 0.0f;

    _goal_pos.x = GOAL_X;
    _goal_pos.y = GOAL_Y;
    _start_pos.x = 0.0f;
    _start_pos.y = 0.0f;

    _last_saved_x = _x;
    _last_saved_y = _y;

    _state = ALIGN_TO_GOAL;

    // Inicialización Sensores Ruedas
    _left_wheel_sensor  = getPositionSensor("left wheel sensor");
    _right_wheel_sensor = getPositionSensor("right wheel sensor");
    _left_wheel_sensor->enable(_time_step);
    _right_wheel_sensor->enable(_time_step);

    // Motores
    _left_wheel_motor  = getMotor("left wheel motor");
    _right_wheel_motor = getMotor("right wheel motor");
    _left_wheel_motor->setPosition(INFINITY);
    _right_wheel_motor->setPosition(INFINITY);
    _left_wheel_motor->setVelocity(0.0);
    _right_wheel_motor->setVelocity(0.0);

    // Sensores Distancia
    _ds_front = getDistanceSensor("ds1");
    _ds_left  = getDistanceSensor("ds0"); 
    _ds_right = getDistanceSensor("ds3");

    _ds_front->enable(_time_step);
    _ds_left->enable(_time_step);
    _ds_right->enable(_time_step);

    // Cámara (Asegúrate de que el nombre coincide con el del árbol de Webots)
    _front_camera = getCamera("camera_f"); 
    _front_camera->enable(_time_step);
}

MyRobot::~MyRobot() {
    _left_wheel_motor->setVelocity(0.0);
    _right_wheel_motor->setVelocity(0.0);
}

// ─────────────────────────────────────────────────────────────────────────────
void MyRobot::run() {
    cout << "Buscando víctimas verdes. Meta de exploración -> x: " << _goal_pos.x << " y: " << _goal_pos.y << endl;

    Point p = {_x, _y};
    _waypoints.push_back(p);

    while (step(_time_step) != -1) {
        compute_odometry(); 
        update_state();     

        _left_wheel_motor->setVelocity(_left_speed);
        _right_wheel_motor->setVelocity(_right_speed);

        save_waypoint(); 
        
        if (_state == DONE || _state == VICTIM_DETECTED) {
            // Detenemos los motores si llegamos a la meta o vemos a alguien
            _left_wheel_motor->setVelocity(0.0);
            _right_wheel_motor->setVelocity(0.0);
            if (_state == DONE) break; 
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Visión Artificial: Detección de color verde
// ─────────────────────────────────────────────────────────────────────────────
bool MyRobot::look_for_green_person() {
    const unsigned char *image = _front_camera->getImage();
    if (!image) return false;

    int width = _front_camera->getWidth();
    int height = _front_camera->getHeight();
    int green_pixels = 0;

    // Recorremos la matriz de la imagen
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            int r = Camera::imageGetRed(image, width, x, y);
            int g = Camera::imageGetGreen(image, width, x, y);
            int b = Camera::imageGetBlue(image, width, x, y);

            // Filtro heurístico: El verde debe ser dominante y brillante
            if (g > (r + 40) && g > (b + 40) && g > 80) {
                green_pixels++;
            }
        }
    }

    // Si más del 2% de los píxeles de la cámara son "muy verdes", detectamos a la persona
    float threshold = width * height * 0.02;
    if (green_pixels > threshold) {
        return true;
    }

    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
void MyRobot::update_state() {
    // 1. ANTES DE NADA: ¿Vemos a una persona?
    if (_state != VICTIM_DETECTED && _state != DONE) {
        if (look_for_green_person()) {
            _state = VICTIM_DETECTED;
            cout << "\n¡¡PERSONA VERDE DETECTADA!! Deteniendo el robot." << endl;
            _left_speed = 0.0;
            _right_speed = 0.0;
            return; // Salimos de update_state para no hacer caso a la odometría
        }
    }

    double front = _ds_front->getValue();
    double left  = _ds_left->getValue();
    double right = _ds_right->getValue(); 

    bool obs_front = (front > OBSTACLE_DIST_THRESHOLD);
    bool obs_left  = (left  > OBSTACLE_DIST_THRESHOLD);
    bool obs_right = (right > OBSTACLE_DIST_THRESHOLD);

    float angle_to_goal = atan2(_goal_pos.y - _y, _goal_pos.x - _x);
    float theta_err = angle_to_goal - _theta;
    
    while (theta_err > M_PI)  theta_err -= 2.0 * M_PI;
    while (theta_err < -M_PI) theta_err += 2.0 * M_PI;

    float current_dist_to_goal = get_distance_to_goal(_x, _y);

    if (current_dist_to_goal < 0.5f) {
        cout << "Meta física de exploración alcanzada." << endl;
        _state = DONE;
        return;
    }

    if (_state == FOLLOW_WALL && abs(theta_err) < 0.2f && !obs_front) {
        _state = GO_TO_GOAL;
    }

    switch (_state) {
        case ALIGN_TO_GOAL:
            if (abs(theta_err) > 0.15) {
                _left_speed  = (theta_err > 0) ? -MAX_SPEED*0.5 : MAX_SPEED*0.5;
                _right_speed = (theta_err > 0) ? MAX_SPEED*0.5 : -MAX_SPEED*0.5;
            } else {
                _state = GO_TO_GOAL;
            }
            break;

        case GO_TO_GOAL:
            if (obs_front) {
                _state = FOLLOW_WALL;
            } else {
                _left_speed  = MAX_SPEED - (theta_err * 2.0);
                _right_speed = MAX_SPEED + (theta_err * 2.0);
            }
            break;

        case FOLLOW_WALL:
            if (obs_left && !obs_front) {
                if (left > OBSTACLE_DIST_THRESHOLD * 1.5) {
                    _left_speed  = MAX_SPEED * 0.8;
                    _right_speed = MAX_SPEED * 0.6;
                } else {
                    _left_speed  = MAX_SPEED * 0.8;
                    _right_speed = MAX_SPEED * 0.8;
                }
            } 
            else if (obs_left && obs_front) {
                _left_speed  = MAX_SPEED * 0.6;
                _right_speed = -MAX_SPEED * 0.6;
            } 
            else if (!obs_left && !obs_front) {
                _left_speed  = MAX_SPEED * 0.3;
                _right_speed = MAX_SPEED * 0.8;
            } 
            else if (!obs_left && obs_front) {
                _left_speed  = MAX_SPEED * 0.6;
                _right_speed = -MAX_SPEED * 0.6;
            }
            
            if (obs_right && (!obs_left && !obs_front)) {
               _left_speed = MAX_SPEED * 0.5;
               _right_speed = MAX_SPEED * 0.5; 
            }
            break;

        case VICTIM_DETECTED:
            _left_speed = 0.0;
            _right_speed = 0.0;
            break;
            
        case DONE:
            break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
void MyRobot::compute_odometry() {
    float cur_sl = encoder_tics_to_meters(_left_wheel_sensor->getValue());
    float cur_sr = encoder_tics_to_meters(_right_wheel_sensor->getValue());

    float d_sl = cur_sl - _sl;
    float d_sr = cur_sr - _sr;

    _sl = cur_sl;
    _sr = cur_sr;

    float d_s     = (d_sr + d_sl) / 2.0f;
    float d_theta = (d_sr - d_sl) / WHEELS_DISTANCE;

    _x += d_s * cos(_theta + d_theta / 2.0f);
    _y += d_s * sin(_theta + d_theta / 2.0f);
    _theta += d_theta; 
}

float MyRobot::encoder_tics_to_meters(float tics) {
    return tics / ENCODER_TICS_PER_RADIAN * WHEEL_RADIUS;
}

float MyRobot::get_distance_to_goal(float current_x, float current_y) {
    return sqrt(pow(_goal_pos.x - current_x, 2) + pow(_goal_pos.y - current_y, 2));
}

void MyRobot::save_waypoint() {
    float dist_since_last = sqrt(pow(_x - _last_saved_x, 2) + pow(_y - _last_saved_y, 2));
    if (dist_since_last > 0.5f && _state == GO_TO_GOAL) {
        Point p = {_x, _y};
        _waypoints.push_back(p);
        _last_saved_x = _x;
        _last_saved_y = _y;
    }
}
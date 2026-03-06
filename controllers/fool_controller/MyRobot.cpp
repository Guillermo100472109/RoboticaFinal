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
}

MyRobot::~MyRobot() {
    _left_wheel_motor->setVelocity(0.0);
    _right_wheel_motor->setVelocity(0.0);
}

// ─────────────────────────────────────────────────────────────────────────────
void MyRobot::run() {
    cout << "Misión Iniciada (Modo Reactivo Simple). Meta -> x: " << _goal_pos.x << "  y: " << _goal_pos.y << endl;

    Point p = {_x, _y};
    _waypoints.push_back(p);

    while (step(_time_step) != -1) {
        compute_odometry(); 
        update_state();     

        _left_wheel_motor->setVelocity(_left_speed);
        _right_wheel_motor->setVelocity(_right_speed);

        save_waypoint(); 
        
        if (_state == DONE) break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
void MyRobot::update_state() {
    double front = _ds_front->getValue();
    double left  = _ds_left->getValue();
    double right = _ds_right->getValue(); 

    bool obs_front = (front > OBSTACLE_DIST_THRESHOLD);
    bool obs_left  = (left  > OBSTACLE_DIST_THRESHOLD);
    bool obs_right = (right > OBSTACLE_DIST_THRESHOLD);

    // Error angular hacia la meta
    float angle_to_goal = atan2(_goal_pos.y - _y, _goal_pos.x - _x);
    float theta_err = angle_to_goal - _theta;
    
    // Normalizar a [-PI, PI]
    while (theta_err > M_PI)  theta_err -= 2.0 * M_PI;
    while (theta_err < -M_PI) theta_err += 2.0 * M_PI;

    float current_dist_to_goal = get_distance_to_goal(_x, _y);

    if (current_dist_to_goal < 0.5f) {
        cout << "¡Meta alcanzada!" << endl;
        _left_speed = 0; _right_speed = 0;
        _state = DONE;
        return;
    }

    // ── NUEVA CONDICIÓN DE SALIDA (Heurística simple a petición) ──────────────
    // Si estamos siguiendo la pared, apuntamos a la meta (error menor a 0.2 rads) 
    // y no hay obstáculos enfrente, rompemos el seguimiento y vamos recto.
    if (_state == FOLLOW_WALL && abs(theta_err) < 0.2f && !obs_front) {
        _state = GO_TO_GOAL;
        cout << "[FSM] → GO_TO_GOAL (Vía libre en dirección a la meta)" << endl;
    }

    switch (_state) {
        // ── Apuntar a la meta ─────────────────────────────────────────────────
        case ALIGN_TO_GOAL:
            if (abs(theta_err) > 0.15) {
                _left_speed  = (theta_err > 0) ? -MAX_SPEED*0.5 : MAX_SPEED*0.5;
                _right_speed = (theta_err > 0) ? MAX_SPEED*0.5 : -MAX_SPEED*0.5;
            } else {
                _state = GO_TO_GOAL;
            }
            break;

        // ── Avanzar a la meta ─────────────────────────────────────────────────
        case GO_TO_GOAL:
            if (obs_front) {
                _state = FOLLOW_WALL;
                cout << "[FSM] → FOLLOW_WALL (Obstáculo detectado)" << endl;
            } else {
                _left_speed  = MAX_SPEED - (theta_err * 2.0);
                _right_speed = MAX_SPEED + (theta_err * 2.0);
            }
            break;

        // ── Seguidor de Pared Izquierda (Lógica estable) ──────────────────────
        case FOLLOW_WALL:
            if (obs_left && !obs_front) {
                // Caso 1: Pared a la izquierda, frente libre
                if (left > OBSTACLE_DIST_THRESHOLD * 1.5) {
                    _left_speed  = MAX_SPEED * 0.8;
                    _right_speed = MAX_SPEED * 0.6;
                } else {
                    _left_speed  = MAX_SPEED * 0.8;
                    _right_speed = MAX_SPEED * 0.8;
                }
            } 
            else if (obs_left && obs_front) {
                // Caso 2: Esquina interior -> Giro fuerte derecha
                _left_speed  = MAX_SPEED * 0.6;
                _right_speed = -MAX_SPEED * 0.6;
            } 
            else if (!obs_left && !obs_front) {
                // Caso 3: Esquina exterior -> Arco izquierda para buscar pared
                _left_speed  = MAX_SPEED * 0.3;
                _right_speed = MAX_SPEED * 0.8;
            } 
            else if (!obs_left && obs_front) {
                // Caso 4: Muro enfrente, nada a la izq -> Giro fuerte derecha
                _left_speed  = MAX_SPEED * 0.6;
                _right_speed = -MAX_SPEED * 0.6;
            }
            
            // Seguridad: si rozamos por la derecha haciendo un arco exterior
            if (obs_right && (!obs_left && !obs_front)) {
               _left_speed = MAX_SPEED * 0.5;
               _right_speed = MAX_SPEED * 0.5; 
            }
            break;

        case DONE:
            break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Odometría Clásica (Pura, sin alteraciones)
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

// ─────────────────────────────────────────────────────────────────────────────
float MyRobot::get_distance_to_goal(float current_x, float current_y) {
    return sqrt(pow(_goal_pos.x - current_x, 2) + pow(_goal_pos.y - current_y, 2));
}

void MyRobot::save_waypoint() {
    float dist_since_last = sqrt(pow(_x - _last_saved_x, 2) + pow(_y - _last_saved_y, 2));
    // Guardamos solo si vamos hacia el objetivo (evitamos guardar rutas enrevesadas esquivando muros)
    if (dist_since_last > 0.5f && _state == GO_TO_GOAL) {
        Point p = {_x, _y};
        _waypoints.push_back(p);
        _last_saved_x = _x;
        _last_saved_y = _y;
    }
}
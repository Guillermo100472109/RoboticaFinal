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

    _state = INITIAL_ALIGN;

    // Seguimiento de víctimas
    _spin_steps    = 0;
    _scan_steps    = 0;

    // Seguimiento dual de paredes
    _follow_left_wall     = true;
    _prev_obs_front       = false;
    _front_obstacle_count = 0;
    _turn_180_steps       = 0;

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
    _ds_front       = getDistanceSensor("ds0");
    _ds_front_right = getDistanceSensor("ds15");
    _ds_left        = getDistanceSensor("ds2"); 
    _ds_right       = getDistanceSensor("ds13");
    _ds_left_perp   = getDistanceSensor("ds3");
    _ds_right_perp  = getDistanceSensor("ds12");

    _ds_front->enable(_time_step);
    _ds_front_right->enable(_time_step);
    _ds_left->enable(_time_step);
    _ds_right->enable(_time_step);
    _ds_left_perp->enable(_time_step);
    _ds_right_perp->enable(_time_step);

    // Cámara (Asegúrate de que el nombre coincide con el del árbol de Webots)
    _front_camera = getCamera("camera_f"); 
    _front_camera->enable(_time_step);

    // Brújula (para obtener la orientación real del robot)
    _compass = getCompass("compass");
    _compass->enable(_time_step);
    _gps = getGPS("gps");
    _gps->enable(_time_step);
}

MyRobot::~MyRobot() {
    _left_wheel_motor->setVelocity(0.0);
    _right_wheel_motor->setVelocity(0.0);
}

// ─────────────────────────────────────────────────────────────────────────────
void MyRobot::run() {
    // Paso inicial para que los sensores tengan datos
    step(_time_step);

    float initial_compass_rad = get_compass_heading();
    float initial_compass_deg = initial_compass_rad * 180.0f / M_PI;
    if (initial_compass_deg < 0.0f) initial_compass_deg += 360.0f;

    cout << "=================================================" << endl;
    cout << "[DEBUG] Ángulo inicial leido (brújula): " << initial_compass_deg << "º" << endl;
    cout << "[DEBUG] Ángulo objetivo absoluto: 270º (equivalente a -90º)" << endl;
    cout << "=================================================" << endl;

    cout << "Iniciando alineación... Meta de exploración -> x: " << _goal_pos.x << " y: " << _goal_pos.y << endl;

    Point p = {_x, _y};
    _waypoints.push_back(p);

    int debug_steps = 0;
    bool first_step = true;
    while (step(_time_step) != -1) {
        compute_odometry(); 
        float _x_gps = _gps->getValues()[2];
        float _y_gps = _gps->getValues()[0];

        if (first_step) {
            _start_pos.x = _x;
            _start_pos.y = _y;
            first_step = false;
        }

        // Debug prints cada ~1 segundo (64ms * 15 ≈ 1s)
        if (debug_steps++ % 15 == 0) {
            float current_compass_rad = get_compass_heading();
            float current_compass_deg = current_compass_rad * 180.0f / M_PI;
            if (current_compass_deg < 0.0f) current_compass_deg += 360.0f;

            float theta_deg = _theta * 180.0f / M_PI;
            while (theta_deg < 0.0f) theta_deg += 360.0f;
            while (theta_deg >= 360.0f) theta_deg -= 360.0f;

            const char* state_names[] = {
                "INITIAL_ALIGN",
                "ALIGN_TO_GOAL",
                "GO_TO_GOAL",
                "FOLLOW_WALL",
                "APPROACH_VICTIM",
                "SPIN_360",
                "SCAN_FOR_MORE",
                "VICTIM_DETECTED",
                "RETURN_TO_START",
                "TURN_180",
                "DONE"
            };

            cout << "[DEBUG] State: " << state_names[_state] << " | Víctimas: " << _victim_positions.size() 
                 << " | Pos GPS: (" << _x_gps << ", " << _y_gps 
                 << ") | Odom: (" << _x << ", " << _y << ") | _theta: " << theta_deg 
                 << "º | Brújula actual: " << current_compass_deg << "º" << endl;
        }

        update_state();     

        _left_wheel_motor->setVelocity(_left_speed);
        _right_wheel_motor->setVelocity(_right_speed);

        save_waypoint();

        if (_state == DONE) {
            _left_wheel_motor->setVelocity(0.0);
            _right_wheel_motor->setVelocity(0.0);
            break;
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

// Devuelve la fracción [0,1] de píxeles verdes dominantes en la imagen
float MyRobot::get_green_ratio() {
    const unsigned char *image = _front_camera->getImage();
    if (!image) return 0.0f;

    int width  = _front_camera->getWidth();
    int height = _front_camera->getHeight();
    int green_pixels = 0;

    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            int r = Camera::imageGetRed(image, width, x, y);
            int g = Camera::imageGetGreen(image, width, x, y);
            int b = Camera::imageGetBlue(image, width, x, y);
            if (g > (r + 40) && g > (b + 40) && g > 80)
                green_pixels++;
        }
    }
    return (float)green_pixels / (float)(width * height);
}

// ─────────────────────────────────────────────────────────────────────────────
void MyRobot::update_state() {
    if (process_victim_detection()) {
        return; // Salir tempranamente si se acaba de detectar una víctima
    }

    // Obtención de datos factorizada (usando odometría)
    Point current_pos = get_current_position();
    float theta_err = calculate_theta_error(current_pos);
    float current_dist_to_goal = get_distance_to_goal(current_pos.x, current_pos.y);

    bool obs_front      = is_obstacle_front();
    bool obs_left_diag  = is_obstacle_left();
    bool obs_right_diag = is_obstacle_right();
    bool obs_left_perp  = is_obstacle_left_perp();
    bool obs_right_perp = is_obstacle_right_perp();

    // Detección de flanco de subida en el obstáculo frontal
    if (obs_front && !_prev_obs_front) {
        if (_state == GO_TO_GOAL) {
            _front_obstacle_count = 1; // Primer obstáculo que nos mete en FOLLOW_WALL
        } else if (_state == FOLLOW_WALL) {
            _front_obstacle_count++;   // Siguiente obstáculo frontal (esquina/callejón)
        }
    }
    _prev_obs_front = obs_front;

    if (current_dist_to_goal < 0.5f &&
        (_state == GO_TO_GOAL || _state == ALIGN_TO_GOAL || _state == FOLLOW_WALL)) {
        if ((int)_victim_positions.size() >= 2) {
            // Regreso al inicio completado
            cout << "Regreso al inicio completado. Misión finalizada." << endl;
            _state = DONE;
        } else {
            // Zona de búsqueda alcanzada, empezar a escanear
            cout << "Zona de búsqueda alcanzada. Iniciando exploración..." << endl;
            _state      = SCAN_FOR_MORE;
            _scan_steps = 0;
        }
        return;
    }

    if (_state == FOLLOW_WALL && abs(theta_err) < 0.2f && !obs_front) {
        _state = GO_TO_GOAL;
        _front_obstacle_count = 0; // Camino despejado
    }

    // Comprobar escape de callejón
    if (_state == FOLLOW_WALL && _front_obstacle_count >= 2) {
        _state = TURN_180;
        _turn_180_steps = 0;
        _follow_left_wall = !_follow_left_wall; // Cambiamos de lado de pared
        _front_obstacle_count = 0;
    }

    switch (_state) {
        case INITIAL_ALIGN: {
            float current_compass_deg = get_compass_heading() * 180.0f / M_PI;
            if (current_compass_deg < 0.0f) current_compass_deg += 360.0f;

            // Mantenemos tu lógica exacta adaptada al ciclo de Webots:
            // Girar de forma constante (0.1, -0.1) hasta llegar a 270.
            // Usamos un pequeño margen porque comprobar == en floats no suele funcionar con ruido.
            if (fabsf(current_compass_deg - 270.0f) > 5.0f) {
                _left_speed = 2.0;
                _right_speed = -2.0;
            } else {
                _left_speed = 0.0;
                _right_speed = 0.0;
                cout << "Alineación inicial completada. Avanzando hacia el objetivo." << endl;
                _theta = 0.0f;
                _x = 0.0f;
                _y = 0.0f;
                _state = GO_TO_GOAL;
            }
            break;
        }

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
            if (_follow_left_wall) {
                // Seguimiento Pared Izquierda
                if (obs_front || obs_left_diag) {
                    _left_speed  = MAX_SPEED * 0.8;
                    _right_speed = -MAX_SPEED * 0.8;
                } 
                else if (obs_left_perp) {
                    if (_ds_left_perp->getValue() > OBSTACLE_DIST_THRESHOLD * 1.5) {
                        _left_speed  = MAX_SPEED * 0.8;
                        _right_speed = MAX_SPEED * 0.6;
                    } else {
                        _left_speed  = MAX_SPEED * 0.8;
                        _right_speed = MAX_SPEED * 0.8;
                    }
                } 
                else {
                    _left_speed  = MAX_SPEED * 0.3;
                    _right_speed = MAX_SPEED * 0.8;
                }
            } else {
                // Seguimiento Pared Derecha (Lógica espejo)
                if (obs_front || obs_right_diag) {
                    _left_speed  = -MAX_SPEED * 0.8;
                    _right_speed = MAX_SPEED * 0.8;
                } 
                else if (obs_right_perp) {
                    if (_ds_right_perp->getValue() > OBSTACLE_DIST_THRESHOLD * 1.5) {
                        _left_speed  = MAX_SPEED * 0.6;
                        _right_speed = MAX_SPEED * 0.8;
                    } else {
                        _left_speed  = MAX_SPEED * 0.8;
                        _right_speed = MAX_SPEED * 0.8;
                    }
                } 
                else {
                    _left_speed  = MAX_SPEED * 0.8;
                    _right_speed = MAX_SPEED * 0.3;
                }
            }
            break;

        case TURN_180:
            _turn_180_steps++;
            if (_turn_180_steps < 80) { // ~180° de giro (ajustar si es necesario)
                _left_speed  = -MAX_SPEED * 0.8;
                _right_speed =  MAX_SPEED * 0.8;
            } else {
                cout << "Giro de 180 completado. Cambiando a seguir la pared " << (_follow_left_wall ? "IZQUIERDA" : "DERECHA") << endl;
                _state = FOLLOW_WALL;
            }
            break;

        case VICTIM_DETECTED:  // fallback — no debería alcanzarse
            _left_speed  = 0.0;
            _right_speed = 0.0;
            break;

        case APPROACH_VICTIM:
            _spin_steps++;   // también actúa como timeout de aproximación
            if (_ds_front->getValue() > VICTIM_APPROACH_THRESHOLD ||  _ds_front_right->getValue() > VICTIM_APPROACH_THRESHOLD || _spin_steps > 260) {
                // Cerca (sensor frontal), o víctima ocupa >10% del frame (cámara), o timeout
                cout << "Posición de inspección alcanzada. Iniciando giro 360°..." << endl;
                _left_speed  = 0.0;
                _right_speed = 0.0;
                _state       = SPIN_360;
                _spin_steps  = 0;
            } else {
                // Avanzar hacia la víctima
                _left_speed  = MAX_SPEED;
                _right_speed = MAX_SPEED;
            }
            break;

        case SPIN_360:
            _spin_steps++;
            if (_spin_steps <= 155) {
                // Girar en sentido antihorario (~360° a 30% velocidad)
                _left_speed  = -MAX_SPEED;
                _right_speed =  MAX_SPEED;
            } else {
                // Giro completado
                _left_speed  = 0.0;
                _right_speed = 0.0;
                if ((int)_victim_positions.size() >= 2) {
                    // Ambas víctimas inspeccionadas → volver al inicio
                    cout << "\n¡Ambas víctimas inspeccionadas! Regresando a posición inicial..." << endl;
                    _goal_pos = _start_pos;
                    _state    = RETURN_TO_START;
                } else {
                    // Primera completada → buscar la segunda
                    cout << "Giro completado. Buscando segunda víctima..." << endl;
                    _state      = SCAN_FOR_MORE;
                    _scan_steps = 0;
                }
            }
            break;

        case SCAN_FOR_MORE:
            _scan_steps++;
            // Fase 1 (~160 pasos ≈ 1 vuelta completa): girar a la izquierda
            if (_scan_steps <= 160) {
                _left_speed  =  MAX_SPEED * 0.3;
                _right_speed = -MAX_SPEED * 0.3;
            }
            // Fase 2 (~60 pasos): avanzar para cambiar ángulo de visión
            else if (_scan_steps <= 220) {
                _left_speed  = MAX_SPEED * 0.5;
                _right_speed = MAX_SPEED * 0.5;
            }
            // Fase 3 (~160 pasos ≈ 1 vuelta completa): girar a la derecha
            else if (_scan_steps <= 380) {
                _left_speed  = -MAX_SPEED * 0.3;
                _right_speed =  MAX_SPEED * 0.3;
            }
            // Búsqueda agotada sin resultado
            else {
                cout << "Búsqueda agotada sin encontrar la segunda víctima." << endl;
                _state = DONE;
            }
            break;

        case RETURN_TO_START:
            // Primero se alinea perfectamente con la meta inicial
            if (abs(theta_err) > 0.15) {
                _left_speed  = (theta_err > 0) ? -MAX_SPEED*0.5 : MAX_SPEED*0.5;
                _right_speed = (theta_err > 0) ? MAX_SPEED*0.5 : -MAX_SPEED*0.5;
            } else {
                // Luego avanza hacia ella (la lógica GO_TO_GOAL esquiva obstáculos automáticamente)
                _state = GO_TO_GOAL;
            }
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

// ─────────────────────────────────────────────────────────────────────────────
// Brújula: Convierte la lectura del compass al heading del robot
// en el mismo sistema de referencia que la odometría.
// La odometría usa: _x avanza con cos(_theta), _y avanza con sin(_theta).
// El Pioneer 2 en este mundo avanza en la dirección -Z local.
// El compass devuelve el vector "norte" (world +Z) en el frame local del robot.
// ─────────────────────────────────────────────────────────────────────────────
float MyRobot::get_compass_heading() {
    const double *values = _compass->getValues();
    // values[0] = componente X del norte en frame local
    // values[2] = componente Z del norte en frame local
    // atan2(x, z) da el ángulo del norte respecto al frente del robot
    // Como el robot avanza en -Z local, necesitamos ajustar:
    float heading = atan2(values[0], values[2]);
    return heading;
}

// ─────────────────────────────────────────────────────────────────────────────
// Refactorización: Funciones auxiliares
// ─────────────────────────────────────────────────────────────────────────────
Point MyRobot::get_current_position() {
    Point p;
    p.x = _x;
    p.y = _y;
    return p;
}

float MyRobot::calculate_theta_error(Point current_pos) {
    float angle_to_goal = atan2(_goal_pos.y - current_pos.y, _goal_pos.x - current_pos.x);
    float theta_err = angle_to_goal - _theta;
    
    while (theta_err > M_PI)  theta_err -= 2.0 * M_PI;
    while (theta_err < -M_PI) theta_err += 2.0 * M_PI;
    return theta_err;
}

bool MyRobot::is_obstacle_front() {
    return (_ds_front->getValue() > OBSTACLE_DIST_THRESHOLD || _ds_front_right->getValue() > OBSTACLE_DIST_THRESHOLD);
}

bool MyRobot::is_obstacle_left() {
    return (_ds_left->getValue() > OBSTACLE_DIST_THRESHOLD);
}

bool MyRobot::is_obstacle_right() {
    return (_ds_right->getValue() > OBSTACLE_DIST_THRESHOLD);
}

bool MyRobot::is_obstacle_left_perp() {
    return (_ds_left_perp->getValue() > OBSTACLE_DIST_THRESHOLD);
}

bool MyRobot::is_obstacle_right_perp() {
    return (_ds_right_perp->getValue() > OBSTACLE_DIST_THRESHOLD);
}

bool MyRobot::process_victim_detection() {
    // ── Detección de víctimas (solo activa cuando el robot ha llegado a 18,0 y está en estado de búsqueda) ──
    if (_state == SCAN_FOR_MORE && _victim_positions.size() < 2) {
        if (look_for_green_person()) {
            // Estimar posición mundial de la víctima: robot + 2.5m en dirección de la mirada
            const float EST_DIST = 2.5f;
            Point est_pos = {_x + EST_DIST * cosf(_theta),
                             _y + EST_DIST * sinf(_theta)};

            // Ignorar si apunta hacia una víctima ya registrada (radio <2 m en espacio mundo)
            bool is_new = true;
            for (const auto& vp : _victim_positions) {
                float d = sqrtf(powf(est_pos.x - vp.x, 2) + powf(est_pos.y - vp.y, 2));
                if (d < 2.0f) { is_new = false; break; }
            }
            if (is_new) {
                _victim_positions.push_back(est_pos);   // guardar posición estimada, no la del robot
                int n = (int)_victim_positions.size();
                cout << "\n¡VÍCTIMA " << n << "/2 DETECTADA! Posición: ("
                     << _x << ", " << _y << ")\n"
                     << "Aproximándome..." << endl;
                _left_speed  = 0.0;
                _right_speed = 0.0;
                _state       = APPROACH_VICTIM;
                _spin_steps  = 0;
                return true;
            }
        }
    }
    return false;
}
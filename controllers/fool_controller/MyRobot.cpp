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
    _pending_victim_pos = {0.0f, 0.0f};

    // Alineación inicial
    _align_direction  = 0;
    _gps_start[0] = _gps_start[1] = _gps_start[2] = 0.0f;

    // Seguimiento dual de paredes
    _follow_left_wall     = true;
    _prev_obs_front       = false;
    _front_obstacle_count = 0;
    _turn_duration        = 0;
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

    cout << "Iniciando misión. Meta: (" << _goal_pos.x << ", " << _goal_pos.y << ")" << endl;

    Point p = {_x, _y};
    _waypoints.push_back(p);

    int debug_steps = 0;
    bool first_step = true;
    while (step(_time_step) != -1) {
        compute_odometry();

        if (first_step) {
            _start_pos.x = _x;
            _start_pos.y = _y;
            first_step = false;
        }

        // Debug ~1 vez por segundo
        if (debug_steps++ % 15 == 0) {
            float theta_deg = _theta * 180.0f / M_PI;
            while (theta_deg <    0.0f) theta_deg += 360.0f;
            while (theta_deg >= 360.0f) theta_deg -= 360.0f;

            switch (_state) {
                case INITIAL_ALIGN: {
                    float cd = get_compass_heading() * 180.0f / M_PI;
                    if (cd < 0.0f) cd += 360.0f;
                    cout << "[INITIAL_ALIGN] Brujula: " << cd << "° → objetivo: 270°" << endl;
                    break;
                }
                case GO_TO_GOAL:
                    cout << "[GO_TO_GOAL] Pos: (" << _x << ", " << _y
                         << ") | θ: " << theta_deg
                         << "° | Dist meta: " << get_distance_to_goal(_x, _y) << " m" << endl;
                    break;
                case FOLLOW_WALL:
                    cout << "[FOLLOW_WALL-" << (_follow_left_wall ? "IZQ" : "DER")
                         << "] Pos: (" << _x << ", " << _y
                         << ") | Front: " << _ds_front->getValue()
                         << " | Giros: " << _front_obstacle_count << endl;
                    break;
                case APPROACH_VICTIM:
                    cout << "[APPROACH_VICTIM] Front: " << _ds_front->getValue()
                         << " | Paso: " << _spin_steps << "/300" << endl;
                    break;
                case SPIN_360:
                    cout << "[SPIN_360] Paso: " << _spin_steps << "/86" << endl;
                    break;
                case SCAN_FOR_MORE:
                    cout << "[SCAN_FOR_MORE] Paso: " << _scan_steps
                         << " | Rescatadas: " << _victim_positions.size() << "/2" << endl;
                    break;
                case RETURN_TO_START:
                    cout << "[RETURN] Pos: (" << _x << ", " << _y
                         << ") | Dist inicio: " << get_distance_to_goal(_x, _y) << " m" << endl;
                    break;
                case TURN_180:
                    cout << "[TURN_180] Paso: " << _turn_180_steps << endl;
                    break;
                default:
                    break;
            }
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

// Devuelve la posición horizontal normalizada del centroide verde: -1=izquierda, 0=centro, +1=derecha
// Devuelve 0 si no hay píxeles verdes.
float MyRobot::get_green_centroid_x() {
    const unsigned char *image = _front_camera->getImage();
    if (!image) return 0.0f;

    int width  = _front_camera->getWidth();
    int height = _front_camera->getHeight();
    long sum_x = 0;
    int  count = 0;

    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            int r = Camera::imageGetRed(image, width, x, y);
            int g = Camera::imageGetGreen(image, width, x, y);
            int b = Camera::imageGetBlue(image, width, x, y);
            if (g > (r + 40) && g > (b + 40) && g > 80) {
                sum_x += x;
                count++;
            }
        }
    }

    if (count == 0) return 0.0f;
    return ((float)sum_x / count - width * 0.5f) / (width * 0.5f);
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

    // Cuenta giros reales de ~90°: obs_front sostenido >= 10 pasos (≈ 0.64 s girando)
    if (obs_front) {
        _turn_duration++;
    } else {
        if (_prev_obs_front && _turn_duration >= 10) {
            if (_state == GO_TO_GOAL) {
                _front_obstacle_count = 1;
            } else if (_state == FOLLOW_WALL) {
                _front_obstacle_count++;
            }
        }
        _turn_duration = 0;
    }
    _prev_obs_front = obs_front;

    // Zona de víctimas (viaje de ida): odometría ≥ 14 m O desplazamiento GPS máximo eje ≥ 13 m
    if ((int)_victim_positions.size() < 2 &&
        (_state == GO_TO_GOAL || _state == FOLLOW_WALL)) {
        const double* g = _gps->getValues();
        float dx      = fabsf((float)g[0] - _gps_start[0]);
        float dy      = fabsf((float)g[1] - _gps_start[1]);
        float dz      = fabsf((float)g[2] - _gps_start[2]);
        float gps_fwd = fmaxf(fmaxf(dx, dy), dz);
        bool odom_ok  = (_x >= 14.0f);
        bool gps_ok   = (gps_fwd >= 13.0f);
        if (odom_ok || gps_ok) {
            cout << "Zona de búsqueda alcanzada (odom=" << _x << " m | GPS_max=" << gps_fwd << " m). Explorando..." << endl;
            _state      = SCAN_FOR_MORE;
            _scan_steps = 0;
            return;
        }
    }

    // Regreso al inicio (viaje de vuelta)
    if (current_dist_to_goal < 0.5f && (int)_victim_positions.size() >= 2 &&
        (_state == GO_TO_GOAL || _state == FOLLOW_WALL)) {
        cout << "Regreso al inicio completado. Misión finalizada." << endl;
        _state = DONE;
        return;
    }

    if (_state == FOLLOW_WALL && abs(theta_err) < 0.2f && !obs_front) {
        _state = GO_TO_GOAL;
        _front_obstacle_count = 0;
    }

    // Comprobar escape de callejón
    if (_state == FOLLOW_WALL && _front_obstacle_count >= 2) {
        _state = TURN_180;
        _turn_180_steps = 0;
        _follow_left_wall = !_follow_left_wall;
        _front_obstacle_count = 0;
    }

    switch (_state) {
        case INITIAL_ALIGN: {
            float current_compass_deg = get_compass_heading() * 180.0f / M_PI;
            if (current_compass_deg < 0.0f) current_compass_deg += 360.0f;

            float diff = 270.0f - current_compass_deg;
            if (diff >  180.0f) diff -= 360.0f;
            if (diff < -180.0f) diff += 360.0f;

            if (fabsf(diff) > 3.0f) {
                // Decidir dirección una sola vez al inicio para evitar oscilaciones en la frontera
                if (_align_direction == 0)
                    _align_direction = (diff > 0.0f) ? 1 : -1;

                _left_speed  =  _align_direction * 2.0;
                _right_speed = -_align_direction * 2.0;
            } else {
                _left_speed  = 0.0;
                _right_speed = 0.0;
                _theta = 0.0f;
                _x    = 0.0f;
                _y    = 0.0f;
                const double* g0 = _gps->getValues();
                _gps_start[0] = (float)g0[0];
                _gps_start[1] = (float)g0[1];
                _gps_start[2] = (float)g0[2];
                cout << "Alineación completada. GPS inicio: ("
                     << _gps_start[0] << ", " << _gps_start[1] << ", " << _gps_start[2] << ")" << endl;
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
                if (_left_speed  >  MAX_SPEED) _left_speed  =  MAX_SPEED;
                if (_left_speed  < -MAX_SPEED) _left_speed  = -MAX_SPEED;
                if (_right_speed >  MAX_SPEED) _right_speed =  MAX_SPEED;
                if (_right_speed < -MAX_SPEED) _right_speed = -MAX_SPEED;
            }
            break;

        case FOLLOW_WALL:
            if (_follow_left_wall) {
                // Seguimiento Pared Izquierda
                if (obs_front) {
                    // Pared frontal: girar a la DERECHA siempre
                    _left_speed  =  MAX_SPEED * 0.8;
                    _right_speed = -MAX_SPEED * 0.8;
                }
                else if (obs_left_diag) {
                    // Pared diagonal izquierda: ajuste suave a la derecha
                    _left_speed  = MAX_SPEED * 0.8;
                    _right_speed = MAX_SPEED * 0.4;
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
                    // Sin pared izquierda: girar a la izquierda para buscarla
                    _left_speed  = MAX_SPEED * 0.3;
                    _right_speed = MAX_SPEED * 0.8;
                }
            } else {
                // Seguimiento Pared Derecha (lógica espejo)
                if (obs_front) {
                    // Pared frontal: girar a la IZQUIERDA siempre
                    _left_speed  = -MAX_SPEED * 0.8;
                    _right_speed =  MAX_SPEED * 0.8;
                }
                else if (obs_right_diag) {
                    // Pared diagonal derecha: ajuste suave a la izquierda
                    _left_speed  = MAX_SPEED * 0.4;
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
                    // Sin pared derecha: girar a la derecha para buscarla
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
            _spin_steps++;
            if (_ds_front->getValue() > VICTIM_APPROACH_THRESHOLD || _spin_steps > 300) {
                cout << "Posición de inspección alcanzada. Iniciando giro 360°..." << endl;
                // Guardar posición confirmada: víctima está ~1 m por delante
                _pending_victim_pos = {
                    _x + 1.0f * cosf(_theta),
                    _y + 1.0f * sinf(_theta)
                };
                _left_speed  = 0.0;
                _right_speed = 0.0;
                _state       = SPIN_360;
                _spin_steps  = 0;
            } else {
                float centroid = get_green_centroid_x();
                float ratio    = get_green_ratio();
                if (ratio > 0.01f && fabsf(centroid) > 0.15f) {
                    // Víctima visible pero descentrada: rotar hacia ella en el sitio
                    float turn   = centroid * MAX_SPEED * 0.5f;
                    _left_speed  =  turn;
                    _right_speed = -turn;
                } else {
                    // Centrada o no visible: avanzar hacia la víctima
                    _left_speed  = MAX_SPEED * 0.5f;
                    _right_speed = MAX_SPEED * 0.5f;
                }
            }
            break;

        case SPIN_360:
            _spin_steps++;
            if (_spin_steps <= 86) {
                // Girar en sentido antihorario (360° a 50% velocidad)
                _left_speed  = -MAX_SPEED * 0.5;
                _right_speed =  MAX_SPEED * 0.5;
            } else {
                // Giro completado: ahora sí se cuenta como rescate
                _left_speed  = 0.0;
                _right_speed = 0.0;
                _victim_positions.push_back(_pending_victim_pos);
                int n = (int)_victim_positions.size();
                cout << "\n¡VÍCTIMA " << n << "/2 RESCATADA! Posición confirmada." << endl;
                if (n >= 2) {
                    cout << "¡Ambas víctimas rescatadas! Regresando a posición inicial..." << endl;
                    _goal_pos = _start_pos;
                    _state    = RETURN_TO_START;
                } else {
                    cout << "Buscando segunda víctima..." << endl;
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
    while (_theta >  (float)M_PI) _theta -= 2.0f * (float)M_PI;
    while (_theta < -(float)M_PI) _theta += 2.0f * (float)M_PI;

    // Correct heading drift using compass every ~5 s (78 steps × 64 ms)
    static int corr_count = 0;
    if (++corr_count >= 78) {
        corr_count = 0;
        float compass_theta = -(get_compass_heading() + (float)M_PI / 2.0f);
        while (compass_theta >  (float)M_PI) compass_theta -= 2.0f * (float)M_PI;
        while (compass_theta < -(float)M_PI) compass_theta += 2.0f * (float)M_PI;
        _theta = 0.8f * _theta + 0.2f * compass_theta;
    }
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
    return (_ds_front->getValue() > OBSTACLE_DIST_THRESHOLD);
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

            // Ignorar si apunta hacia una víctima ya registrada (radio <2.5 m en espacio mundo)
            bool is_new = true;
            for (const auto& vp : _victim_positions) {
                float d = sqrtf(powf(est_pos.x - vp.x, 2) + powf(est_pos.y - vp.y, 2));
                if (d < 2.5f) { is_new = false; break; }
            }
            if (is_new) {
                int n = (int)_victim_positions.size() + 1;
                cout << "\n¡VÍCTIMA " << n << "/2 DETECTADA! Aproximándome..." << endl;
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
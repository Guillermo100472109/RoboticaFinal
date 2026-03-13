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

    // FIX: _spin_steps solo se inicializa una vez
    _spin_steps = 0;
    _scan_steps = 0;

    // Wall Follower Adaptativo
    _follow_right_wall = false;  // Comenzar siguiendo muro izquierdo
    _last_theta = 0.0f;
    _clockwise_turns = 0;
    _counter_clockwise_turns = 0;

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

    // Cámara frontal
    _front_camera = getCamera("camera_f");
    _front_camera->enable(_time_step);

    // Brújula
    _compass = getCompass("compass");
    _compass->enable(_time_step);
}

MyRobot::~MyRobot() {
    _left_wheel_motor->setVelocity(0.0);
    _right_wheel_motor->setVelocity(0.0);
}

// ─────────────────────────────────────────────────────────────────────────────
void MyRobot::run() {
    step(_time_step); // Paso inicial para que los sensores tengan datos

    float initial_compass_deg = get_compass_heading() * 180.0f / M_PI;
    if (initial_compass_deg < 0.0f) initial_compass_deg += 360.0f;

    cout << "=================================================" << endl;
    cout << "[INFO] Angulo inicial (brujula): " << initial_compass_deg << "deg" << endl;
    cout << "[INFO] Objetivo de alineacion: 270deg" << endl;
    cout << "=================================================" << endl;
    cout << "Iniciando alineacion... Meta -> x: " << _goal_pos.x << " y: " << _goal_pos.y << endl;

    Point p = {_x, _y};
    _waypoints.push_back(p);

    // FIX: debug cada ~2 segundos en lugar de ~1 para reducir carga de consola
    int debug_steps = 0;
    while (step(_time_step) != -1) {
        compute_odometry();

        // Imprimir debug cada ~2 segundos (64ms * 30 ≈ 2s)
        if (debug_steps++ % 30 == 0) {
            float theta_deg = _theta * 180.0f / M_PI;
            while (theta_deg < 0.0f)    theta_deg += 360.0f;
            while (theta_deg >= 360.0f) theta_deg -= 360.0f;
            cout << "[DEBUG] Pos: (" << _x << ", " << _y
                 << ") | theta: " << theta_deg << "deg"
                 << " | Estado: " << _state << endl;
        }

        update_state();

        _left_wheel_motor->setVelocity(_left_speed);
        _right_wheel_motor->setVelocity(_right_speed);

        save_waypoint();

        if (_state == DONE) {
            _left_wheel_motor->setVelocity(0.0);
            _right_wheel_motor->setVelocity(0.0);
            cout << "Mision completada." << endl;
            break;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// FIX: Fusionado look_for_green_person() y get_green_ratio() en una sola
// función que recorre la imagen UNA sola vez. Evita doble iteración por frame.
// ─────────────────────────────────────────────────────────────────────────────
float MyRobot::get_green_ratio() {
    const unsigned char *image = _front_camera->getImage();
    if (!image) return 0.0f;

    int width  = _front_camera->getWidth();
    int height = _front_camera->getHeight();
    int green_pixels = 0;
    int total = width * height;

    for (int i = 0; i < total; i++) {
        // BGRA layout en Webots: índices 0=B, 1=G, 2=R, 3=A por píxel
        int b = image[i * 4 + 0];
        int g = image[i * 4 + 1];
        int r = image[i * 4 + 2];
        if (g > (r + 40) && g > (b + 40) && g > 80)
            green_pixels++;
    }
    return (float)green_pixels / (float)total;
}

bool MyRobot::look_for_green_person() {
    // FIX: Reutiliza get_green_ratio(), no recorre la imagen dos veces
    return get_green_ratio() > 0.02f;
}

// ─────────────────────────────────────────────────────────────────────────────
void MyRobot::update_state() {
    // ── Detección de víctimas (activa solo una vez cruzada la línea amarilla) ──
    if (_state != DONE && _state != APPROACH_VICTIM && _state != SPIN_360
        && _x >= GOAL_X) {

        float green = get_green_ratio(); // UNA sola llamada, resultado reutilizado
        if (green > 0.02f) {
            const float EST_DIST = 2.5f;
            Point est_pos = {_x + EST_DIST * cosf(_theta),
                             _y + EST_DIST * sinf(_theta)};

            bool is_new = true;
            for (const auto& vp : _victim_positions) {
                float dx = est_pos.x - vp.x;
                float dy = est_pos.y - vp.y;
                // FIX: comparar distancia^2 para evitar sqrt innecesario
                if ((dx*dx + dy*dy) < 4.0f) { is_new = false; break; }
            }
            if (is_new) {
                _victim_positions.push_back(est_pos);
                int n = (int)_victim_positions.size();
                cout << "\n¡VICTIMA " << n << "/2 DETECTADA! Robot en: ("
                     << _x << ", " << _y << ")\n"
                     << "Aproximandome..." << endl;
                _left_speed  = 0.0;
                _right_speed = 0.0;
                _state       = APPROACH_VICTIM;
                _spin_steps  = 0;
                return;
            }
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
    while (theta_err >  M_PI) theta_err -= 2.0f * M_PI;
    while (theta_err < -M_PI) theta_err += 2.0f * M_PI;

    float current_dist_to_goal = get_distance_to_goal(_x, _y);

    // ── Comprobación de llegada al objetivo ──────────────────────────────────
    if (current_dist_to_goal < 0.5f &&
        (_state == GO_TO_GOAL || _state == ALIGN_TO_GOAL || _state == FOLLOW_WALL)) {
        if ((int)_victim_positions.size() >= 2) {
            cout << "Regreso al inicio completado. Mision finalizada." << endl;
            _state = DONE;
        } else {
            cout << "Zona de busqueda alcanzada. Iniciando exploracion..." << endl;
            _state      = SCAN_FOR_MORE;
            _scan_steps = 0;
        }
        return;
    }

    // FIX: Condición de vuelta a GO_TO_GOAL más flexible:
    // Solo requiere que no hay obstáculo frontal
    if (_state == FOLLOW_WALL && !obs_front) {
        _state = GO_TO_GOAL;
        _clockwise_turns = 0;
        _counter_clockwise_turns = 0;
    }

    switch (_state) {
        // ── Alineación inicial con brújula ───────────────────────────────────
        case INITIAL_ALIGN: {
            float deg = get_compass_heading() * 180.0f / M_PI;
            if (deg < 0.0f) deg += 360.0f;

            if (fabsf(deg - 270.0f) > 5.0f) {
                _left_speed  =  2.0;
                _right_speed = -2.0;
            } else {
                _left_speed  = 0.0;
                _right_speed = 0.0;
                cout << "Alineacion inicial completada. Avanzando hacia el objetivo." << endl;
                // Resetear odometría en posición y ángulo conocidos
                _theta = 0.0f;
                _x     = 0.0f;
                _y     = 0.0f;
                _state = GO_TO_GOAL;
            }
            break;
        }

        // ── Realineación hacia el objetivo (usado en retorno) ────────────────
        case ALIGN_TO_GOAL:
            if (fabsf(theta_err) > 0.15f) {
                _left_speed  = (theta_err > 0) ? -MAX_SPEED * 0.5f :  MAX_SPEED * 0.5f;
                _right_speed = (theta_err > 0) ?  MAX_SPEED * 0.5f : -MAX_SPEED * 0.5f;
            } else {
                _state = GO_TO_GOAL;
            }
            break;

        // ── Navegación directa al objetivo ──────────────────────────────────
        case GO_TO_GOAL:
            if (obs_front) {
                _state = FOLLOW_WALL;
                _clockwise_turns = 0;
                _counter_clockwise_turns = 0;
                _last_theta = _theta;
            } else {
                // Control proporcional de orientación
                _left_speed  = MAX_SPEED - (theta_err * 2.0f);
                _right_speed = MAX_SPEED + (theta_err * 2.0f);
                // Clampear velocidades
                if (_left_speed  > MAX_SPEED)  _left_speed  = MAX_SPEED;
                if (_right_speed > MAX_SPEED)   _right_speed = MAX_SPEED;
                if (_left_speed  < -MAX_SPEED)  _left_speed  = -MAX_SPEED;
                if (_right_speed < -MAX_SPEED)  _right_speed = -MAX_SPEED;
            }
            break;

        // ── Seguimiento de pared ADAPTATIVO (izquierda o derecha) ─────────────
        case FOLLOW_WALL: {
            // Detectar giros usando el cambio de ángulo (derivada de theta)
            float d_theta = _theta - _last_theta;
            // Normalizar a [-PI, PI]
            while (d_theta >  M_PI) d_theta -= 2.0f * M_PI;
            while (d_theta < -M_PI) d_theta += 2.0f * M_PI;

            _last_theta = _theta;

            // Si d_theta > 0: girando contra-reloj (izquierda)
            // Si d_theta < 0: girando reloj (derecha)
            const float TURN_THRESHOLD = 0.03f;  // ~1.7° de giro

            if (d_theta > TURN_THRESHOLD) {
                // Girando izquierda (CCW)
                _counter_clockwise_turns++;
                _clockwise_turns = 0;
            } else if (d_theta < -TURN_THRESHOLD) {
                // Girando derecha (CW)
                _clockwise_turns++;
                _counter_clockwise_turns = 0;
            } else {
                // No gira significativamente → resetear contadores lentamente
                if (_clockwise_turns > 0) _clockwise_turns--;
                if (_counter_clockwise_turns > 0) _counter_clockwise_turns--;
            }

            // ATRAPAMIENTO DETECTADO: muchos giros sostenidos en la misma dirección
            if (_clockwise_turns > 15 && !_follow_right_wall) {
                _follow_right_wall = true;
                cout << "[WALL_FOLLOWER] ATRAPAMIENTO: cambiando a seguir muro DERECHO" << endl;
                _clockwise_turns = 0;
                _counter_clockwise_turns = 0;
            } else if (_counter_clockwise_turns > 15 && _follow_right_wall) {
                _follow_right_wall = false;
                cout << "[WALL_FOLLOWER] ATRAPAMIENTO: cambiando a seguir muro IZQUIERDO" << endl;
                _clockwise_turns = 0;
                _counter_clockwise_turns = 0;
            }

            // ─── SEGUIMIENTO DE MURO IZQUIERDO ───────────────────────────────
            if (!_follow_right_wall) {
                if (!obs_front && obs_left) {
                    // Mantener distancia al muro izquierdo
                    if (left > OBSTACLE_DIST_THRESHOLD * 1.5) {
                        // Demasiado lejos → girar suavemente a la izquierda
                        _left_speed  = MAX_SPEED * 0.6f;
                        _right_speed = MAX_SPEED * 0.8f;
                    } else {
                        // Distancia adecuada → avanzar recto
                        _left_speed  = MAX_SPEED * 0.8f;
                        _right_speed = MAX_SPEED * 0.8f;
                    }
                } else if (obs_front && obs_left) {
                    // Muro por delante y izquierda → girar a la derecha
                    _left_speed  =  MAX_SPEED * 0.6f;
                    _right_speed = -MAX_SPEED * 0.6f;
                } else if (!obs_front && !obs_left) {
                    // Sin muro a la izquierda → girar a la izquierda para buscarlo
                    _left_speed  = MAX_SPEED * 0.3f;
                    _right_speed = MAX_SPEED * 0.8f;
                } else if (obs_front && !obs_left) {
                    // Solo obstáculo frontal → girar a la derecha
                    _left_speed  =  MAX_SPEED * 0.6f;
                    _right_speed = -MAX_SPEED * 0.6f;
                }
            }
            // ─── SEGUIMIENTO DE MURO DERECHO ──────────────────────────────────
            else {
                if (!obs_front && obs_right) {
                    // Mantener distancia al muro derecho
                    if (right > OBSTACLE_DIST_THRESHOLD * 1.5) {
                        // Demasiado lejos → girar suavemente a la derecha
                        _left_speed  = MAX_SPEED * 0.8f;
                        _right_speed = MAX_SPEED * 0.6f;
                    } else {
                        // Distancia adecuada → avanzar recto
                        _left_speed  = MAX_SPEED * 0.8f;
                        _right_speed = MAX_SPEED * 0.8f;
                    }
                } else if (obs_front && obs_right) {
                    // Muro por delante y derecha → girar a la izquierda
                    _left_speed  = -MAX_SPEED * 0.6f;
                    _right_speed =  MAX_SPEED * 0.6f;
                } else if (!obs_front && !obs_right) {
                    // Sin muro a la derecha → girar a la derecha para buscarlo
                    _left_speed  = MAX_SPEED * 0.8f;
                    _right_speed = MAX_SPEED * 0.3f;
                } else if (obs_front && !obs_right) {
                    // Solo obstáculo frontal → girar a la izquierda
                    _left_speed  = -MAX_SPEED * 0.6f;
                    _right_speed =  MAX_SPEED * 0.6f;
                }
            }
            break;
        }

        // ── Aproximación a víctima detectada ────────────────────────────────
        case APPROACH_VICTIM:
            _spin_steps++;
            if (get_green_ratio() > 0.10f || front > VICTIM_APPROACH_THRESHOLD || _spin_steps > 200) {
                cout << "Posicion de inspeccion alcanzada. Iniciando giro 360..." << endl;
                _left_speed  = 0.0;
                _right_speed = 0.0;
                _state       = SPIN_360;
                _spin_steps  = 0;
            } else {
                _left_speed  = MAX_SPEED * 0.4f;
                _right_speed = MAX_SPEED * 0.4f;
            }
            break;

        // ── Giro 360° para señalar la víctima ───────────────────────────────
        case SPIN_360:
            _spin_steps++;
            // FIX: velocidad moderada (0.3) para un giro limpio de ~360°
            if (_spin_steps <= 155) {
                _left_speed  = -MAX_SPEED * 0.3f;
                _right_speed =  MAX_SPEED * 0.3f;
            } else {
                _left_speed  = 0.0;
                _right_speed = 0.0;
                if ((int)_victim_positions.size() >= 2) {
                    cout << "\n¡Ambas victimas inspeccionadas! Regresando al inicio..." << endl;
                    _goal_pos = _start_pos;
                    _state    = ALIGN_TO_GOAL;
                } else {
                    cout << "Giro completado. Buscando segunda victima..." << endl;
                    _state      = SCAN_FOR_MORE;
                    _scan_steps = 0;
                }
            }
            break;

        // ── Búsqueda activa de la segunda víctima ───────────────────────────
        case SCAN_FOR_MORE:
            _scan_steps++;
            // FIX: la detección de víctimas también opera en SCAN_FOR_MORE
            // (el bloque de detección al inicio de update_state() ya lo cubre
            //  porque el estado no es APPROACH_VICTIM ni SPIN_360 ni DONE)

            // Fase 1: girar ~360° a la izquierda para escanear el entorno
            if (_scan_steps <= 160) {
                _left_speed  =  MAX_SPEED * 0.3f;
                _right_speed = -MAX_SPEED * 0.3f;
            }
            // Fase 2: avanzar un poco para cambiar ángulo de visión
            else if (_scan_steps <= 220) {
                _left_speed  = MAX_SPEED * 0.5f;
                _right_speed = MAX_SPEED * 0.5f;
            }
            // Fase 3: girar ~360° a la derecha
            else if (_scan_steps <= 380) {
                _left_speed  = -MAX_SPEED * 0.3f;
                _right_speed =  MAX_SPEED * 0.3f;
            }
            // Búsqueda agotada: continuar navegando hacia el objetivo
            else {
                cout << "Busqueda agotada. Continuando navegacion." << endl;
                _state = GO_TO_GOAL;
            }
            break;

        case VICTIM_DETECTED: // fallback — no debería alcanzarse
            _left_speed  = 0.0;
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

    _x     += d_s * cosf(_theta + d_theta / 2.0f);
    _y     += d_s * sinf(_theta + d_theta / 2.0f);
    _theta += d_theta;
}

float MyRobot::encoder_tics_to_meters(float tics) {
    return tics / ENCODER_TICS_PER_RADIAN * WHEEL_RADIUS;
}

float MyRobot::get_distance_to_goal(float current_x, float current_y) {
    float dx = _goal_pos.x - current_x;
    float dy = _goal_pos.y - current_y;
    return sqrtf(dx*dx + dy*dy);
}

void MyRobot::save_waypoint() {
    float dx = _x - _last_saved_x;
    float dy = _y - _last_saved_y;
    // FIX: comparar distancia^2 para evitar sqrt en cada ciclo
    if ((dx*dx + dy*dy) > 0.25f && _state == GO_TO_GOAL) {
        Point p = {_x, _y};
        _waypoints.push_back(p);
        _last_saved_x = _x;
        _last_saved_y = _y;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
float MyRobot::get_compass_heading() {
    const double *values = _compass->getValues();
    // values[0] = X del norte en frame local
    // values[2] = Z del norte en frame local
    float heading = atan2f((float)values[0], (float)values[2]);
    return heading;
}
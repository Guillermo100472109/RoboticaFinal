#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

#include <iostream>
#include <vector>
#include <cmath>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp> // <- Añadimos la librería de la cámara
#include <webots/Compass.hpp> // <- Brújula para orientación inicial
#include <webots/GPS.hpp>


using namespace std;
using namespace webots;

// ── Constantes Físicas y de Navegación ────────────────────────────────────────
#define MAX_SPEED               5.0
#define WHEELS_DISTANCE         0.3606
#define WHEEL_RADIUS            0.0825
#define ENCODER_TICS_PER_RADIAN 1.0

#define OBSTACLE_DIST_THRESHOLD 150.0
#define VICTIM_APPROACH_THRESHOLD 750.0  // ds_front value to stop ~0.5 m from victim (tune if needed)
#define GOAL_X                  18.0
#define GOAL_Y                  0.0

struct Point {
    float x;
    float y;
};

// ── Estados de la FSM ─────────────────────────────────────────────────────────
typedef enum {
    INITIAL_ALIGN,
    ALIGN_TO_GOAL,
    GO_TO_GOAL,
    FOLLOW_WALL,
    APPROACH_VICTIM, // <- Avanzar hacia la víctima hasta ~1 m
    SPIN_360,        // <- Girar 360° en el sitio
    SCAN_FOR_MORE,   // <- Buscar la segunda víctima
    VICTIM_DETECTED, // <- Mantenido por compatibilidad
    RETURN_TO_START, // <- Nuevo estado para volver al inicio
    TURN_180,        // <- Girar 180° tras encontrar un callejón sin salida
    DONE
} State;

class MyRobot : public Robot {
public:
    MyRobot();
    ~MyRobot();

    void run();

private:
    int    _time_step;
    double _left_speed, _right_speed;

    // Localización
    float  _x, _y, _theta;
    float  _sr, _sl;
    
    // Variables de Meta
    Point  _start_pos;
    Point  _goal_pos;

    // Waypoints
    std::vector<Point> _waypoints;
    float _last_saved_x, _last_saved_y;

    State  _state;

    // Seguimiento de víctimas
    int                _spin_steps;
    int                _scan_steps;
    std::vector<Point> _victim_positions;

    // Odometría robusta
    float              _compass_at_reset; // heading de la brújula cuando se fijó _theta=0

    // Seguimiento dual de paredes
    bool               _follow_left_wall;
    bool               _prev_obs_front;
    int                _front_obstacle_count;
    int                _turn_180_steps;

    // Dispositivos
    PositionSensor *_left_wheel_sensor;
    PositionSensor *_right_wheel_sensor;
    Motor          *_left_wheel_motor;
    Motor          *_right_wheel_motor;
    DistanceSensor *_ds_front;
    DistanceSensor *_ds_front_right;
    DistanceSensor *_ds_left;           
    DistanceSensor *_ds_right;
    DistanceSensor *_ds_left_perp;
    DistanceSensor *_ds_right_perp;
    Camera         *_front_camera; // <- Dispositivo de cámara
    Compass        *_compass;      // <- Brújula para heading real
    GPS            *_gps;

    // Métodos internos
    void   compute_odometry();
    float  encoder_tics_to_meters(float tics);
    void   update_state();
    float  get_distance_to_goal(float current_x, float current_y);
    void   save_waypoint();
    bool   look_for_green_person();
    float  get_green_ratio();          // fracción [0,1] de píxeles verdes en la cámara
    float  get_compass_heading();      // heading real del robot en radianes

    // Refactorización
    Point  get_current_position();
    float  calculate_theta_error(Point current_pos);
    bool   is_obstacle_front();
    bool   is_obstacle_left();
    bool   is_obstacle_right();
    bool   is_obstacle_left_perp();
    bool   is_obstacle_right_perp();
    bool   process_victim_detection();
};

#endif /* MY_ROBOT_H_ */
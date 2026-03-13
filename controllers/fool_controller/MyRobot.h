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

using namespace std;
using namespace webots;

// ── Constantes Físicas y de Navegación ────────────────────────────────────────
#define MAX_SPEED               5.0
#define WHEELS_DISTANCE         0.3606
#define WHEEL_RADIUS            0.0825
#define ENCODER_TICS_PER_RADIAN 1.0

#define OBSTACLE_DIST_THRESHOLD 150.0
#define VICTIM_APPROACH_THRESHOLD 750.0  // ds_front value to stop ~0.5 m from victim (tune if needed)
#define GOAL_X                  16.0
#define GOAL_Y                  0.0

struct Point {
    float x;
    float y;
};

// ── Estados de la FSM ─────────────────────────────────────────────────────────
typedef enum {
    ALIGN_TO_GOAL,
    GO_TO_GOAL,
    FOLLOW_WALL,
    APPROACH_VICTIM, // <- Avanzar hacia la víctima hasta ~1 m
    SPIN_360,        // <- Girar 360° en el sitio
    SCAN_FOR_MORE,   // <- Buscar la segunda víctima
    VICTIM_DETECTED, // <- Mantenido por compatibilidad
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

    // Dispositivos
    PositionSensor *_left_wheel_sensor;
    PositionSensor *_right_wheel_sensor;
    Motor          *_left_wheel_motor;
    Motor          *_right_wheel_motor;
    DistanceSensor *_ds_front;
    DistanceSensor *_ds_left;           
    DistanceSensor *_ds_right;
    Camera         *_front_camera; // <- Dispositivo de cámara

    // Métodos internos
    void   compute_odometry();
    float  encoder_tics_to_meters(float tics);
    void   update_state();
    float  get_distance_to_goal(float current_x, float current_y);
    void   save_waypoint();
    bool   look_for_green_person();
    float  get_green_ratio();          // fracción [0,1] de píxeles verdes en la cámara
};

#endif /* MY_ROBOT_H_ */
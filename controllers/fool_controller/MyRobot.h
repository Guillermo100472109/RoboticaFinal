#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

#include <iostream>
#include <vector>
#include <cmath>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
<<<<<<< HEAD
#include <webots/Camera.hpp>
#include <webots/Compass.hpp>
=======
#include <webots/Camera.hpp> // <- Añadimos la librería de la cámara
>>>>>>> parent of 4ff7628 (Compás incial añadido)

using namespace std;
using namespace webots;

// ── Constantes Físicas y de Navegación ────────────────────────────────────────
#define MAX_SPEED               5.0
#define WHEELS_DISTANCE         0.3606
#define WHEEL_RADIUS            0.0825
#define ENCODER_TICS_PER_RADIAN 1.0

#define OBSTACLE_DIST_THRESHOLD   150.0
#define VICTIM_APPROACH_THRESHOLD 750.0
#define GOAL_X                    16.0
#define GOAL_Y                    0.0

struct Point {
    float x;
    float y;
};

// ── Estados de la FSM ─────────────────────────────────────────────────────────
typedef enum {
    ALIGN_TO_GOAL,
    GO_TO_GOAL,
    FOLLOW_WALL,
    APPROACH_VICTIM,
    SPIN_360,
    SCAN_FOR_MORE,
    VICTIM_DETECTED,
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

    // Wall Follower Adaptativo
    bool               _follow_right_wall;  // true = seguir muro derecho, false = seguir muro izquierdo
    float              _last_theta;         // Angulo anterior para detectar giros
    int                _clockwise_turns;    // Contador de giros CW (derecha) consecutivos
    int                _counter_clockwise_turns; // Contador de giros CCW (izquierda) consecutivos

    // Dispositivos
    PositionSensor *_left_wheel_sensor;
    PositionSensor *_right_wheel_sensor;
    Motor          *_left_wheel_motor;
    Motor          *_right_wheel_motor;
    DistanceSensor *_ds_front;
    DistanceSensor *_ds_left;
    DistanceSensor *_ds_right;
<<<<<<< HEAD
    Camera         *_front_camera;
    Compass        *_compass;
=======
    Camera         *_front_camera; // <- Dispositivo de cámara
>>>>>>> parent of 4ff7628 (Compás incial añadido)

    // Métodos internos
    void   compute_odometry();
    float  encoder_tics_to_meters(float tics);
    void   update_state();
    float  get_distance_to_goal(float current_x, float current_y);
    void   save_waypoint();
    bool   look_for_green_person();
<<<<<<< HEAD
    float  get_green_ratio();
    float  get_compass_heading();
=======
    float  get_green_ratio();          // fracción [0,1] de píxeles verdes en la cámara
>>>>>>> parent of 4ff7628 (Compás incial añadido)
};

#endif /* MY_ROBOT_H_ */
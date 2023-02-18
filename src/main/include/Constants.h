#pragma once

// #include <units/units.h>

// ################## CAN_ID ##################
//Drivetrain
#define CAN_ID_DRIVETRAIN_MOTOR_RIGHT 1
#define CAN_ID_DRIVETRAIN_MOTOR_RIGHT_FOLLOW 2
#define CAN_ID_DRIVETRAIN_MOTOR_LEFT 3
#define CAN_ID_DRIVETRAIN_MOTOR_LEFT_FOLLOW 4

// ################## JOYSTICK_ID ##################
#define DRIVER_JOYSTICK_RIGHT_ID 0
#define DRIVER_JOYSTICK_LEFT_ID 1

// ################## SMART_CURRENT_LIMIT ##################
// Drivetrain
#define SMART_CURRENT_LIMIT_DRIVETRAIN_MOTOR 40

// ################## RAMP_RATE ##################
// Drivetrain
#define RAMP_RATE_DRIVETRAIN_MOTOR 0.1

// ################## VOLTAGE_COMPENSATION ##################
// Drivetrain
#define VOLTAGE_COMPENSATION_DRIVETRAIN_MOTOR 11.0


#define TRACKWIDTH 0.61f
#define HALF_TRACKWIDTH (TRACKWIDTH / 2.0f)

#define AMAX 5.1 // Acceleration Max  au PIF .. à définir aux encodeurs
#define VMAX 3.4 // vitesse Max  théorique (3,395472 sur JVN-DT) .. à vérifier aux encodeurs
#define WMAX                     \
  (((2.0 * VMAX) / TRACKWIDTH) / \
   1.7) // vitesse angulaire Max theorique    .. à modifier avec Garice

#define NABS(a) (((a) < 0) ? -(a) : (a))     // VALEUR ABSOLUE
#define NMAX(a, b) (((a) > (b)) ? (a) : (b)) // Max
#define NMIN(a, b) (((a) < (b)) ? (a) : (b)) // Min

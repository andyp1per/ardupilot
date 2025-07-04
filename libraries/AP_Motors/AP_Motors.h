#pragma once

#include "AP_Motors_config.h"

#include "AP_Motors_Class.h"
#include "AP_MotorsMulticopter.h"
#include "AP_MotorsMatrix.h"
#if AP_MOTORS_TRI_ENABLED
#include "AP_MotorsTri.h"
#endif  // AP_MOTORS_TRI_ENABLED
#include "AP_MotorsHeli_Single.h"
#include "AP_MotorsHeli_Dual.h"
#include "AP_MotorsHeli_Quad.h"
#include "AP_MotorsSingle.h"
#include "AP_MotorsCoax.h"
#include "AP_MotorsTailsitter.h"
#include "AP_Motors6DOF.h"
#include "AP_MotorsMatrix_6DoF_Scripting.h"
#include "AP_MotorsMatrix_Scripting_Dynamic.h"

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  IMU-aided height above ground filter. It runs on each filter step at the fusion time horizon
  from the main filter's bias-corrected vertical acceleration and the downward range finder, and
  supplies the height above ground the flow fusion, the range finder height and getHAGL() use.
 */
#include "AP_NavEKF3.h"
#include "AP_NavEKF3_core.h"

#if EK3_FEATURE_AGL_KF

#include <AP_DAL/AP_DAL.h>

// 3-state AGL KF initialisation
// Start with generous uncertainty; the first valid RF measurement will hard-reset the state
void NavEKF3_core::AglKalmanFilter::InitialiseVariables(ftype initialHgt)
{
    hgt = initialHgt;    // assume sitting on ground at minimum range
    vel = 0.0f;
    bias = 0.0f;
    memset(cov, 0, sizeof(cov));
    cov[0][0] = 25.0f;   // 5 m initial std-dev in height
    cov[1][1] = 1.0f;    // 1 m/s initial std-dev in velocity
    cov[2][2] = 1.0f;    // 1 m/s/s initial std-dev in accel-Z bias (learn it quickly)
    valid = false;
    lastRngFuseTime_ms = 0;
}

// Cap one AGL KF variance by scaling its row and column rather than the diagonal
// alone: a diagonal-only clamp leaves the off-diagonals free to grow past
// sqrt(P[i][i]*P[j][j]), which drives the bias gain without bound.
static void aglKfCapVariance(ftype Pagl[3][3], uint8_t i, ftype maxVar)
{
    if (Pagl[i][i] <= maxVar) {
        return;
    }
    // a NaN variance also lands here; restart that state uncorrelated at the cap
    const bool isNaN = isnan(Pagl[i][i]);
    const ftype scale = isNaN ? 0.0f : sqrtF(maxVar / Pagl[i][i]);
    for (uint8_t j = 0; j < 3; j++) {
        Pagl[i][j] = isNaN ? 0.0f : Pagl[i][j] * scale;
        Pagl[j][i] = isNaN ? 0.0f : Pagl[j][i] * scale;
    }
    Pagl[i][i] = maxVar;
}

/*
 * 3-state IMU-aided AGL Kalman filter
 *
 * State:       x  = [h_agl (m), v_agl (m/s), b_az (m/s/s)]'   (AGL is "up")
 *              b_az is the residual accel-Z error left in velDotNED.z after the
 *              main filter's own bias estimate has been removed by
 *              correctDeltaVelocity(); estimating it here stops that residual
 *              accumulating into the AGL height.
 * Transition:  F  = [[1, imuDt, 0], [0, 1, imuDt], [0, 0, 1]]
 * Input:       u  = -velDotNED.z * imuDt        (main filter's accel bias estimate removed)
 * Noise:       Qvel = sq(EK3_ACC_P_NSE * imuDt)
 *              Qhgt = sq(EK3_TERR_GRAD) * horizDist^2
 *              Qbias = sq(EK3_AGL_ABIAS_P * imuDt)
 *
 * Prediction:  x(k+1) = F*x(k) + [0, u, 0]'
 *              P(k+1) = F*P*F' + Q
 *
 * Observation: z = rng * prevTnb.c.z,  H = [1, 0, 0],  R = sq(EK3_RNG_M_NSE)
 * Update:      innov = z - H*x,  innovVar = H*P*H' + R
 *              K = P*H' / innovVar
 *              x += K * innov
 *              P  = (I-KH)*P*(I-KH)' + K*R*K'   (b_az observable via successive RF updates)
 */
void NavEKF3_core::UpdateAglKf()
{
    const ftype imuDt = imuDataDelayed.delVelDT;
    const uint32_t aglKfRngTimeout_ms = 5000;   // mark filter invalid / hard-reset after this gap without RF fusion

    // h_agl(k+1) = h_agl(k) + v_agl(k)*imuDt
    aglKf.hgt += aglKf.vel * imuDt;

    // v_agl(k+1) = v_agl(k) + (aglKf.bias - velDotNED.z)*imuDt
    // velDotNED.z is NED-down acceleration (positive = downward, includes gravity).
    // Negate to get AGL (up) rate, and remove the estimated accel-Z bias aglKf.bias so the
    // error in the main filter's accel-bias estimate does not accumulate into the height.
    // aglKf.bias itself is a random walk with no deterministic prediction term.
    aglKf.vel += (aglKf.bias - velDotNED.z) * imuDt;

    // First-order decay of v_agl toward zero once the range finder has stopped
    // arriving (tau = 2 s).  Without range measurements v_agl is unobservable and
    // accumulated IMU bias error makes it drift, pulling h_agl to the floor during
    // subsequent climbs.  The test is on elapsed time since the last fusion rather
    // than on rangeDataToFuse: range data is capped at 20 Hz while the filter steps
    // at about 83 Hz, so decaying whenever no sample was due would run on most steps
    // and bias v_agl low by roughly half in a sustained climb or descent.
    if (imuSampleTime_ms - aglKf.lastRngFuseTime_ms > aglKfRngGapMax_ms) {
        const ftype tauV = 2.0f;
        aglKf.vel *= expf(-imuDt / tauV);
    }

    // the floor below is fused as a measurement rather than clamped, so a NaN restarts from it here
    if (isnan(aglKf.hgt) || isnan(aglKf.vel)) {
        aglKf.hgt = rngOnGnd;
        aglKf.vel = 0.0f;
    }

#if EK3_FEATURE_RANGEFINDER_MEASUREMENTS
    // A range finder below its minimum delivers no reading, so after a touchdown nothing corrects
    // the velocity the landing left and the height coasts up off the ground. With the sensor
    // reporting out of range low and no sample arriving, stop the coast. A main filter moving
    // vertically faster than a resting vehicle, up or down, is a liftoff or touchdown still below
    // the sensor minimum, which is left to the IMU
    const uint32_t rngLowTime_ms = rngOutOfRangeLowTime_ms[rangeDataDelayed.sensor_idx];
    if ((rngLowTime_ms != 0) && (imuSampleTime_ms - rngLowTime_ms < rngFreshTime_ms) &&
        (imuSampleTime_ms - rngValidMeaTime_ms > rngOnGndNoDataTime_ms) &&
        (fabsF(stateStruct.velocity.z) < aglKfStillVelZMax_m_s)) {
        aglKf.vel = 0.0f;
    }
    // That test trusts the main filter, which can believe it is climbing on the ground after a
    // hard flight. A range last fused near the floor that has had no reading for a while is on
    // the ground whatever the filter says, so a height well above that last reading is a coast:
    // hold it on the floor until the range finder reads again or stops reporting too low.
    // Known limit: a range finder that reads Good on the ground and then reports too low from
    // liftoff while in range (one returning zero on a lost return) pins the height to the floor
    // until the 5 s timeout
    const bool rngLowNearFloor = (rngLowTime_ms != 0) && (imuSampleTime_ms - rngLowTime_ms < rngFreshTime_ms) &&
                                 (imuSampleTime_ms - rngValidMeaTime_ms > aglKfFloorHoldNoRngTime_ms) &&
                                 (aglKfLastRngHgt < rngOnGnd + aglKfFloorHoldNearFloor_m);
    if (!rngLowNearFloor) {
        aglKfHeldOnFloor = false;
    } else if (aglKf.hgt > aglKfLastRngHgt + aglKfFloorHoldCoast_m) {
        aglKfHeldOnFloor = true;
    }
    if (aglKfHeldOnFloor) {
        aglKf.hgt = rngOnGnd;
        aglKf.vel = 0.0f;
    }
#endif

    // ----- Covariance prediction: P = F*P*F' + Q -----
    //
    // F = [[1, imuDt, 0    ],   state [h, v, b]; v gains imuDt*b from the bias state
    //      [0,  1,   imuDt ],
    //      [0,  0,   1     ]]
    //
    // Process noise Q:
    //
    // Qhgt - terrain-induced AGL uncertainty during forward flight.
    //   As the vehicle moves horizontally by dist over terrain with unknown gradient "g",
    //   the true AGL changes by ~g * dist.  Since "g" is unknown, we treat it as zero-mean
    //   with std-dev terrGradMax, giving variance: Qhgt = terrGradMax^2 * horizDist^2
    //   horizDist^2 = (vx^2+vy^2)*imuDt^2 is the squared horizontal distance travelled this step.
    //   Capped at 1 m^2 so a single large-velocity step can't blow up the covariance.
    //   The intended effect is that P[0][0] grows quickly during fast horizontal flight over rough terrain,
    //   allowing the RF measurement to pull h_agl back when the next reading arrives.
    //
    // Qvel - unmodelled vertical accelerations (IMU noise, vibration, model error).
    //   Uses the same accNoise parameter as CovariancePrediction (sq(imuDt*accNoise)),
    //   so the velocity uncertainty budget is consistent with the main EKF.
    //   The intended effect is that P[1][1] grows every step when RF is absent, reflecting accumulating IMU integration error in v_agl.
    //
    // Qbias - random walk of the accel-Z bias state, from EK3_AGL_ABIAS_P rather than the main
    //   filter's EK3_ABIAS_P_NSE; its parameter description gives the response it sets.
    //
    const ftype horizDistSq = MIN(sq(stateStruct.velocity.x * imuDt)
                                  + sq(stateStruct.velocity.y * imuDt), 1.0f);  // cap at 1 m^2
    const ftype Qvel = sq(frontend->_accNoise * imuDt);   // matches CovariancePrediction: sq(imuDt*accNoise)
    const ftype Qhgt = sq(frontend->_terrGradMax) * horizDistSq;
    const ftype Qbias = sq(constrain_ftype(frontend->_aglKfAccelBiasPnse.get(), 0.0f, 1.0f) * imuDt);

    // Capture before overwrite (P is symmetric)
    const ftype P00 = aglKf.cov[0][0];
    const ftype P01 = aglKf.cov[0][1];
    const ftype P02 = aglKf.cov[0][2];
    const ftype P11 = aglKf.cov[1][1];
    const ftype P12 = aglKf.cov[1][2];
    const ftype P22 = aglKf.cov[2][2];

    // Expanded F*P*F' + Q:
    aglKf.cov[0][0] = P00 + imuDt * (P01 + P01) + sq(imuDt) * P11 + Qhgt;
    aglKf.cov[0][1] = aglKf.cov[1][0] = P01 + imuDt * (P11 + P02) + sq(imuDt) * P12;
    aglKf.cov[0][2] = aglKf.cov[2][0] = P02 + imuDt * P12;
    aglKf.cov[1][1] = P11 + imuDt * (P12 + P12) + sq(imuDt) * P22 + Qvel;
    aglKf.cov[1][2] = aglKf.cov[2][1] = P12 + imuDt * P22;
    aglKf.cov[2][2] = P22 + Qbias;

    // Cap covariance to prevent runaway during prolonged RF absence
    aglKfCapVariance(aglKf.cov, 0, 100.0f);  // 10 m std-dev cap
    aglKfCapVariance(aglKf.cov, 1, 100.0f);  // 10 m/s std-dev cap
    aglKfCapVariance(aglKf.cov, 2, 25.0f);   // 5 m/s/s std-dev cap on bias

    // AGL cannot go below the on-ground sensor reading. Fuse that floor as a reading with the range
    // finder's noise, so a downward velocity resting on it is corrected through the covariance.
    // Overwriting the height and clearing the velocity instead counted a touchdown twice: once at the
    // clear, and again as the impact decelerated the vehicle, which lifted the height off the ground.
    // Not on a step with a fresh range sample, which is floored already, and not into the bias, as
    // for any reading at the floor
    if ((aglKf.hgt < rngOnGnd) && (!rangeDataToFuse || posVelFusionDelayed)) {
        FuseAglKfHeight(rngOnGnd - aglKf.hgt, sq(frontend->_rngNoise), false);
    }

    // mark invalid if RF has been absent too long. On a step SelectVelPosFusion() delayed for
    // magnetometer fusion, rangeDataToFuse still holds the previous step's sample, already used
    if (!rangeDataToFuse || posVelFusionDelayed) {
        if (imuSampleTime_ms - aglKf.lastRngFuseTime_ms > aglKfRngTimeout_ms) {
            aglKf.valid = false;
        }
        return;
    }

    // Only fuse when vehicle tilt is within acceptable limits
    if (prevTnb.c.z < frontend->DCM33FlowMin) {
        return;
    }

    // After the timeout of IMU-only propagation, vertical velocity drift makes the
    // prediction unreliable.  Re-initialise directly from the rangefinder.
    if (imuSampleTime_ms - aglKf.lastRngFuseTime_ms > aglKfRngTimeout_ms) {
        // Tilt-corrected AGL directly from rangefinder reading
        aglKf.hgt = MAX(rangeDataDelayed.rng * prevTnb.c.z, rngOnGnd);
        aglKf.vel = 0.0f;                          // assume stationary on reset
        // keep the learned aglKf.bias (it is slowly varying) but re-open its uncertainty
        // and drop the cross-covariances
        memset(aglKf.cov, 0, sizeof(aglKf.cov));
        aglKf.cov[0][0] = sq(frontend->_rngNoise); // initialise h uncertainty to RF noise
        aglKf.cov[1][1] = 1.0f;                    // 1 m/s velocity uncertainty after reset
        aglKf.cov[2][2] = 1.0f;                    // 1 m/s/s bias uncertainty after reset
        aglKf.lastRngFuseTime_ms = imuSampleTime_ms;
        aglKfLastHgtInnov = 0.0f;
        aglKfLastRngHgt = aglKf.hgt;
        aglKf.valid = true;
        return;  // skip measurement update this cycle (just used the reading for reset)
    }

    // Measurement update - fuse tilt-corrected rangefinder reading
    //
    // Observation model: z = h_agl,  H = [1, 0, 0]
    //   z_meas = rng * cos(tilt) = rng * prevTnb.c.z
    //
    const ftype hgtMeas = MAX(rangeDataDelayed.rng * prevTnb.c.z, rngOnGnd);

    // Measurement noise variance R (reuse EK3_RNG_M_NSE)
    const ftype measNoiseVar = sq(frontend->_rngNoise);

    // Innovation and innovation covariance
    // hgtInnov = hgtMeas - H*x = hgtMeas - h_agl
    // innovVar = H*P*H' + R    = P[0][0] + R
    const ftype hgtInnov = hgtMeas - aglKf.hgt;
    const ftype innovVar = aglKf.cov[0][0] + measNoiseVar;

    // reject outliers (RF glitches, specular reflections, etc.)
    // gate is expressed as a multiplier on the 1-sigma bound.
    const ftype innovGate = MAX(0.01f * (ftype)frontend->_rngInnovGate, 1.0f);
    if (sq(hgtInnov) > sq(innovGate) * innovVar) {
        // Innovation too large, likely a glitch.  Inflate the state uncertainties so the
        // next valid reading can correct them more aggressively.  Only the diagonals:
        // leaving the off-diagonals alone weakens the correlations, which is what keeps
        // the first accepted reading after a glitch out of the bias state.
        aglKf.cov[0][0] = MIN(aglKf.cov[0][0] * 2.0f, 100.0f);
        aglKf.cov[1][1] = MIN(aglKf.cov[1][1] * 2.0f, 100.0f);
        aglKf.cov[2][2] = MIN(aglKf.cov[2][2] * 2.0f, 25.0f);
        return;
    }

    // the AGL KF reads a step in the ground as vertical motion, and with R from EK3_RNG_M_NSE
    // its velocity takes seconds to recover, so note the step for the velD fusion. The prediction
    // carries the IMU, so a lagging velocity or an accel bias moves the innovation slowly and a
    // step moves it in one sample, however long since the last one. 25 ms of jitter in when the
    // samples were taken is allowed for. A jump only counts from a filter that agreed with the
    // range: one already well off it may be its own velocity error showing, as after an accel
    // clip, and holding the velocity out then leaves that error uncorrected. After a gap in the
    // range the last innovation says nothing about the filter now, so any jump counts
    const bool innovSettled = (imuSampleTime_ms - aglKf.lastRngFuseTime_ms > aglKfRngGapMax_ms) ||
                              (fabsF(aglKfLastHgtInnov) < aglKfStepFromMax);
    if (innovSettled &&
        (fabsF(hgtInnov - aglKfLastHgtInnov) > aglKfStepMin + 0.025f * fabsF(aglKf.vel))) {
        aglKfStepTime_ms = imuSampleTime_ms;
    }

    // A measurement at the on-ground floor carries nothing about the bias; fusing it would only
    // shrink the bias variance and lock in whatever the start-up transient left there. The reading
    // is tested before the position offset correction, which can move it off the floor, and the
    // corrected height as well, which a tilted reading reaches only after the tilt correction
    const bool onFloor = rangeDataDelayed.onFloor || ((aglKf.hgt <= rngOnGnd) && (hgtMeas <= rngOnGnd));
    FuseAglKfHeight(hgtInnov, measNoiseVar, !onFloor);

    aglKf.lastRngFuseTime_ms = imuSampleTime_ms;
    aglKfLastHgtInnov = hgtInnov;
    aglKfLastRngHgt = hgtMeas;
    aglKf.valid = true;
}

// FuseAglKfHeight - fuse one height observation into the AGL KF, given its innovation (observed
// height less the estimate, m) and its noise variance (m^2). With learnBias false the bias takes
// no part, for an observation that carries nothing about it
void NavEKF3_core::FuseAglKfHeight(ftype hgtInnov, ftype measNoiseVar, bool learnBias)
{
    const ftype innovVar = aglKf.cov[0][0] + measNoiseVar;

    // Kalman gain:  K = P*H' / innovVar = [P[0][0], P[1][0], P[2][0]]' / innovVar
    // (H = [1, 0, 0], so P*H' is the first column of P)
    const ftype Kh = aglKf.cov[0][0] / innovVar;
    const ftype Kv = aglKf.cov[1][0] / innovVar;
    const ftype Kb = learnBias ? aglKf.cov[2][0] / innovVar : 0.0f;

    // State update:  x += K * hgtInnov
    aglKf.hgt += Kh * hgtInnov;
    aglKf.vel += Kv * hgtInnov;
    aglKf.bias += Kb * hgtInnov;
    // the bias is an accelerometer error like the main filter's, so it gets the same limit
    // (in m/s/s here, so without the dtEkfAvg the delta-velocity states need)
    aglKf.bias  = constrain_ftype(aglKf.bias, -frontend->_accBiasLim, frontend->_accBiasLim);

    // Covariance update P = (I-KH)*P*(I-KH)' + K*R*K'  (Joseph form).
    // With H = [1, 0, 0], (I-KH) = [[1-Kh, 0, 0], [-Kv, 1, 0], [-Kb, 0, 1]]:
    //   P[0][0] = (1-Kh)^2*Phh + Kh^2*R
    //   P[0][1] = (1-Kh)*(Phv - Kv*Phh) + Kh*Kv*R
    //   P[0][2] = (1-Kh)*(Phb - Kb*Phh) + Kh*Kb*R
    //   P[1][1] = Pvv - 2*Kv*Phv + Kv^2*innovVar
    //   P[1][2] = Pvb - Kb*Phv - Kv*Phb + Kv*Kb*innovVar
    //   P[2][2] = Pbb - 2*Kb*Phb + Kb^2*innovVar
    // The h/v terms reduce to the original 2-state expressions when the bias terms are 0.
    const ftype oneMinusKh = 1.0f - Kh;
    const ftype Phh = aglKf.cov[0][0];
    const ftype Phv = aglKf.cov[0][1];
    const ftype Phb = aglKf.cov[0][2];
    const ftype Pvv = aglKf.cov[1][1];
    const ftype Pvb = aglKf.cov[1][2];
    const ftype Pbb = aglKf.cov[2][2];

    aglKf.cov[0][0] = MAX(sq(oneMinusKh) * Phh + sq(Kh) * measNoiseVar, 0.0f);
    aglKf.cov[0][1] = aglKf.cov[1][0] = oneMinusKh * (Phv - Kv * Phh) + Kh * Kv * measNoiseVar;
    aglKf.cov[0][2] = aglKf.cov[2][0] = oneMinusKh * (Phb - Kb * Phh) + Kh * Kb * measNoiseVar;
    aglKf.cov[1][1] = MAX(Pvv - 2.0f * Kv * Phv + sq(Kv) * innovVar, 0.0f);
    aglKf.cov[1][2] = aglKf.cov[2][1] = Pvb - Kb * Phv - Kv * Phb + Kv * Kb * innovVar;
    aglKf.cov[2][2] = MAX(Pbb - 2.0f * Kb * Phb + sq(Kb) * innovVar, 0.0f);
}

#endif  // EK3_FEATURE_AGL_KF

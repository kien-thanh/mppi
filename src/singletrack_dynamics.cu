#include <singletrack_dynamics.cuh>
#include <mppi/utils/math_utils.h>

SingletrackDynamics::SingletrackDynamics(float wheelbase, cudaStream_t stream) : Dynamics<SingletrackDynamics, SingletrackDynamicsParams>(stream)
{
  this->params_ = SingletrackDynamicsParams(wheelbase);
}

bool SingletrackDynamics::computeGrad(const Eigen::Ref<const state_array> &state, const Eigen::Ref<const control_array> &control, Eigen::Ref<dfdx> A, Eigen::Ref<dfdu> B)
{
  return true;
}

void SingletrackDynamics::computeDynamics(const Eigen::Ref<const state_array> &state, const Eigen::Ref<const control_array> &control, Eigen::Ref<state_array> state_der)
{
}

__device__ void SingletrackDynamics::computeDynamics(float *state, float *control, float *state_der, float *theta_s)
{
  float steer = angle_utils::normalizeAngle(state[S_INDEX(STEER)]);
  float yaw = angle_utils::normalizeAngle(state[S_INDEX(YAW)]);

  const float sin_steer = __sinf(steer);
  const float cos_steer = __cosf(steer);
  const float sin_yaw = __sinf(yaw);
  const float cos_yaw = __cosf(yaw);

  const float l_wb = this->params_.wheelbase;

  state_der[S_INDEX(X_POS)] = state[S_INDEX(VEL)] * cos_yaw;
  state_der[S_INDEX(Y_POS)] = state[S_INDEX(VEL)] * sin_yaw;
  state_der[S_INDEX(STEER)] = control[C_INDEX(STEER_SPEED)];
  state_der[S_INDEX(VEL)] = control[C_INDEX(ACCELERATION)];
  state_der[S_INDEX(YAW)] = state[S_INDEX(VEL)] / (l_wb * __tanf(state[S_INDEX(STEER)]));
}

void SingletrackDynamics::printState(const Eigen::Ref<const state_array>& state)
{
  printf("Position: x=%f, y=%f; Steering angle: %f; Velocity: %f; Yaw: %f \n", state(0), state(1), state(2), state(3), state(4));
}

void SingletrackDynamics::printState(float *state)
{
  printf("Position: x=%f, y=%f; Steering angle: %f; Velocity: %f; Yaw: %f \n", state[0], state[1], state[2], state[3], state[4]);
}

Dynamics<SingletrackDynamics, SingletrackDynamicsParams>::state_array
SingletrackDynamics::stateFromMap(const std::map<std::string, float> &map)
{
  state_array s;
  s(S_INDEX(X_POS)) = map.at("X_POS");
  s(S_INDEX(Y_POS)) = map.at("Y_POS");
  s(S_INDEX(STEER)) = map.at("STEER");
  s(S_INDEX(VEL)) = map.at("VEL");
  s(S_INDEX(YAW)) = map.at("YAW");

  return s;
}
#include <optional>
#include <tuple>

#pragma once

class PIDController {
  private:
    float kp;
    float ki;
    float kd;

    float stable_error;

    bool stablized = false;

    std::optional<long> last_timestamp;

    float previous_error = 0.0;

    std::optional<std::tuple<float, float>> output_limits;
    std::optional<std::tuple<float, float>> integral_bounds;

  public:
    PIDController(float kp, float ki, float kd, float stable_error);
    float update(float error);
    void reset() {
        last_timestamp.reset();
        previous_error = 0.0f;
        integral_sum = 0.0f;
        stablized = false;
    }
    float integral_sum = 0.0;

    /**
     * Runs a stability check using a given error.
     *
     * Used to revalidate stability during update, or to check whether calling update() with a given
     * error will result in stability
     */
    bool check_stability(float error);

    /**
     * Checks if the controller has stabilized during a previous update() or check_stability() call
     */
    [[nodiscard]] bool is_stable() const;

    /**
      Specifies the maximum and minimum values which can be output by the
      controller
     */
    PIDController with_output_limits(float min, float max);
    /**
      Specifies the lower and upper bounds of the error where the integral term
      will be active
     */
    PIDController with_integral_bounds(float lower, float upper);
};
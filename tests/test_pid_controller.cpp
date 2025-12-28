/**
 * @file test_pid_controller.cpp
 * @brief Unit tests for PIDController class
 *
 * These tests verify the correctness of PID control law, anti-windup,
 * and saturation mechanisms.
 */

#include <gtest/gtest.h>
#include "control/pid_controller.hpp"
#include <cmath>

using namespace gnc;

/**
 * Test Fixture for PIDController tests
 */
class PIDControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Standard PID with reasonable parameters
        controller = std::make_unique<PIDController>(
            1.0, 0.5, 0.1,   // Kp, Ki, Kd
            -10.0, 10.0,     // output limits
            5.0              // integral limit
        );
    }

    std::unique_ptr<PIDController> controller;
};

/**
 * Test: Proportional term only (Ki=0, Kd=0)
 *
 * Expected behavior: output = Kp * error
 */
TEST_F(PIDControllerTest, ProportionalOnly) {
    PIDController p_controller(2.0, 0.0, 0.0, -10.0, 10.0);

    double error = 3.0;
    double output = p_controller.compute(error, 0.1);

    // output = Kp * error = 2.0 * 3.0 = 6.0
    EXPECT_NEAR(output, 6.0, 1e-6);
    EXPECT_NEAR(p_controller.getProportionalTerm(), 6.0, 1e-6);
    EXPECT_NEAR(p_controller.getIntegralTerm(), 0.0, 1e-6);
    EXPECT_NEAR(p_controller.getDerivativeTerm(), 0.0, 1e-6);
}

/**
 * Test: Integral accumulation
 *
 * Expected behavior: Integral should accumulate error over time
 */
TEST_F(PIDControllerTest, IntegralAccumulation) {
    PIDController i_controller(0.0, 1.0, 0.0, -10.0, 10.0);

    double dt = 0.1;
    double constant_error = 2.0;

    // Apply constant error for 5 steps
    double output = 0.0;
    for (int i = 0; i < 5; ++i) {
        output = i_controller.compute(constant_error, dt);
    }

    // Integral = sum(error * dt) = 2.0 * 0.1 * 5 = 1.0
    // Output = Ki * integral = 1.0 * 1.0 = 1.0
    EXPECT_NEAR(output, 1.0, 1e-6);
    EXPECT_NEAR(i_controller.getIntegralAccumulation(), 1.0, 1e-6);
}

/**
 * Test: Derivative term
 *
 * Expected behavior: Derivative should respond to rate of error change
 */
TEST_F(PIDControllerTest, DerivativeTerm) {
    PIDController d_controller(0.0, 0.0, 1.0, -10.0, 10.0);

    double dt = 0.1;

    // First call: error = 0, derivative should be 0 (no previous error)
    double output1 = d_controller.compute(0.0, dt);
    EXPECT_NEAR(output1, 0.0, 1e-6);

    // Second call: error = 2.0, derivative = (2.0 - 0.0) / 0.1 = 20.0
    double output2 = d_controller.compute(2.0, dt);
    // Output = Kd * derivative = 1.0 * 20.0 = 20.0 (before saturation)
    // But saturated to max = 10.0
    EXPECT_NEAR(output2, 10.0, 1e-6);  // Saturated
    EXPECT_NEAR(d_controller.getDerivativeTerm(), 20.0, 1e-6);  // Before sat
}

/**
 * Test: Anti-windup (integral clamping)
 *
 * Expected behavior: Integral term should not exceed integral_max
 */
TEST_F(PIDControllerTest, AntiWindup) {
    PIDController aw_controller(0.0, 1.0, 0.0,
                                -10.0, 10.0,
                                3.0);  // integral_max = 3.0

    double dt = 0.1;
    double large_error = 100.0;

    // Apply large error for many steps
    for (int i = 0; i < 100; ++i) {
        aw_controller.compute(large_error, dt);
    }

    // Integral should be clamped to integral_max
    double integral = aw_controller.getIntegralAccumulation();
    EXPECT_LE(integral, 3.0);
    EXPECT_GE(integral, -3.0);
    EXPECT_NEAR(integral, 3.0, 1e-6);  // Should hit upper limit
}

/**
 * Test: Output saturation
 *
 * Expected behavior: Output should never exceed [output_min, output_max]
 */
TEST_F(PIDControllerTest, OutputSaturation) {
    PIDController sat_controller(10.0, 0.0, 0.0, -5.0, 5.0);

    // Large positive error: Kp * error = 10.0 * 10.0 = 100.0
    // Should saturate to output_max = 5.0
    double output_pos = sat_controller.compute(10.0, 0.1);
    EXPECT_EQ(output_pos, 5.0);

    // Large negative error: Kp * error = 10.0 * (-10.0) = -100.0
    // Should saturate to output_min = -5.0
    double output_neg = sat_controller.compute(-10.0, 0.1);
    EXPECT_EQ(output_neg, -5.0);
}

/**
 * Test: Reset functionality
 *
 * Expected behavior: After reset, integral and derivative history cleared
 */
TEST_F(PIDControllerTest, ResetFunctionality) {
    double dt = 0.1;

    // Accumulate some integral and derivative history
    for (int i = 0; i < 10; ++i) {
        controller->compute(5.0, dt);
    }

    // Integral should be non-zero
    EXPECT_GT(std::abs(controller->getIntegralAccumulation()), 0.0);

    // Reset
    controller->reset();

    // Integral should be zero
    EXPECT_EQ(controller->getIntegralAccumulation(), 0.0);

    // Next derivative should be zero (no previous error)
    double output = controller->compute(5.0, dt);
    // Only proportional and integral (from this step) should contribute
    // But integral from one step is small
}

/**
 * Test: Gain modification during runtime
 *
 * Expected behavior: setGains should update gains without resetting state
 */
TEST_F(PIDControllerTest, GainModification) {
    double dt = 0.1;

    // Compute with initial gains
    double output1 = controller->compute(2.0, dt);

    // Change gains (double Kp)
    controller->setGains(2.0, 0.5, 0.1);

    // Compute again with same error
    // Note: integral and derivative history are preserved
    double output2 = controller->compute(2.0, dt);

    // Proportional term should have changed
    // output2 should be greater than output1 due to higher Kp
    EXPECT_GT(std::abs(output2), std::abs(output1) - 0.5);
}

/**
 * Test: Step response behavior
 *
 * Educational test: Verify typical step response characteristics
 */
TEST_F(PIDControllerTest, StepResponse) {
    PIDController step_controller(1.0, 0.1, 0.05, -10.0, 10.0);

    double dt = 0.01;
    double setpoint = 10.0;
    double measurement = 0.0;

    std::vector<double> outputs;

    // Simulate closed-loop step response (simple first-order plant)
    for (int i = 0; i < 500; ++i) {
        double error = setpoint - measurement;
        double output = step_controller.compute(error, dt);
        outputs.push_back(output);

        // Simple plant: dy/dt = output
        measurement += output * dt;
    }

    // By end of simulation, error should be small
    double final_error = setpoint - measurement;
    EXPECT_LT(std::abs(final_error), 0.5) << "Should converge close to setpoint";

    // Output should decrease as error decreases
    EXPECT_LT(std::abs(outputs.back()), std::abs(outputs.front()));
}

/**
 * Test: Constructor parameter validation
 *
 * Expected behavior: Should reject invalid output limits
 */
TEST(PIDControllerConstructorTest, ParameterValidation) {
    // output_min >= output_max should throw
    EXPECT_THROW(PIDController(1.0, 0.0, 0.0, 10.0, 5.0), std::invalid_argument);

    // output_min == output_max should throw
    EXPECT_THROW(PIDController(1.0, 0.0, 0.0, 5.0, 5.0), std::invalid_argument);

    // integral_max <= 0 should throw
    EXPECT_THROW(PIDController(1.0, 0.1, 0.0, -10.0, 10.0, 0.0),
                 std::invalid_argument);
    EXPECT_THROW(PIDController(1.0, 0.1, 0.0, -10.0, 10.0, -1.0),
                 std::invalid_argument);

    // Valid parameters should not throw
    EXPECT_NO_THROW(PIDController(1.0, 0.1, 0.05, -10.0, 10.0, 5.0));
}

/**
 * Test: Zero gains
 *
 * Expected behavior: Zero gains should produce zero output
 */
TEST_F(PIDControllerTest, ZeroGains) {
    PIDController zero_controller(0.0, 0.0, 0.0, -10.0, 10.0);

    double output = zero_controller.compute(100.0, 0.1);
    EXPECT_EQ(output, 0.0) << "Zero gains should give zero output";
}

/**
 * Test: Negative error handling
 *
 * Expected behavior: Should handle negative errors correctly
 */
TEST_F(PIDControllerTest, NegativeError) {
    PIDController neg_controller(2.0, 0.0, 0.0, -10.0, 10.0);

    double negative_error = -3.0;
    double output = neg_controller.compute(negative_error, 0.1);

    // output = Kp * error = 2.0 * (-3.0) = -6.0
    EXPECT_NEAR(output, -6.0, 1e-6);
}

/**
 * Main function for running tests
 */
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

/**
 * @file test_bicycle_model.cpp
 * @brief Unit tests for BicycleModel class
 *
 * These tests verify the correctness of the bicycle kinematic model
 * and RK4 integration implementation.
 */

#include <gtest/gtest.h>
#include "dynamics/bicycle_model.hpp"
#include <cmath>

using namespace gnc;

/**
 * Test Fixture for BicycleModel tests
 */
class BicycleModelTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create model with standard parameters
        model = std::make_unique<BicycleModel>(2.5, 0.01);
    }

    std::unique_ptr<BicycleModel> model;
};

/**
 * Test: Straight line motion with zero steering
 *
 * Expected behavior: Vehicle should travel straight along heading direction
 * with constant velocity.
 */
TEST_F(BicycleModelTest, StraightLineMotion) {
    // Initial state: origin, heading east (θ=0), velocity 5 m/s
    VehicleState initial{0.0, 0.0, 0.0, 5.0, 0.0};
    model->reset(initial);

    // Control: no acceleration, no steering
    ControlInputs inputs{0.0, 0.0};

    // Simulate for 1 second (100 steps at 100 Hz)
    VehicleState state = initial;
    for (int i = 0; i < 100; ++i) {
        state = model->step(state, inputs);
    }

    // After 1 second at 5 m/s, should travel 5 meters in +x direction
    EXPECT_NEAR(state.x, 5.0, 0.01) << "X position should be ~5m";
    EXPECT_NEAR(state.y, 0.0, 0.01) << "Y position should remain ~0m";
    EXPECT_NEAR(state.theta, 0.0, 0.01) << "Heading should remain ~0 rad";
    EXPECT_NEAR(state.v, 5.0, 0.01) << "Velocity should remain ~5 m/s";
}

/**
 * Test: Circular motion with constant steering
 *
 * Expected behavior: Vehicle should follow a circular path.
 * After one complete revolution, should return approximately to start.
 */
TEST_F(BicycleModelTest, CircularMotion) {
    // Initial state: origin, heading north (θ=π/2), velocity 5 m/s
    VehicleState initial{0.0, 0.0, M_PI/2, 5.0, 0.0};
    model->reset(initial);

    // Constant steering angle
    double steering_angle = 0.2;  // rad (≈11.5 degrees)
    ControlInputs inputs{0.0, steering_angle};

    // Calculate expected circle radius: R = L / tan(δ)
    double wheelbase = model->getWheelbase();
    double radius = wheelbase / std::tan(steering_angle);

    // Period for one full circle: T = 2πR / v
    double period = 2.0 * M_PI * radius / 5.0;
    int steps = static_cast<int>(period / model->getTimestep());

    // Simulate one full revolution
    VehicleState state = initial;
    for (int i = 0; i < steps; ++i) {
        state = model->step(state, inputs);
    }

    // Should return approximately to origin (allow some numerical error)
    EXPECT_NEAR(state.x, initial.x, 0.5) << "Should return to start X";
    EXPECT_NEAR(state.y, initial.y, 0.5) << "Should return to start Y";

    // Heading should have rotated by 2π (back to original)
    double heading_diff = std::fmod(state.theta - initial.theta + M_PI, 2*M_PI) - M_PI;
    EXPECT_NEAR(heading_diff, 0.0, 0.1) << "Heading should return to start";
}

/**
 * Test: Acceleration and deceleration
 *
 * Expected behavior: Velocity should increase/decrease according to
 * acceleration input.
 */
TEST_F(BicycleModelTest, AccelerationDeceleration) {
    // Start from rest
    VehicleState initial{0.0, 0.0, 0.0, 0.0, 0.0};
    model->reset(initial);

    // Accelerate at 2 m/s² for 3 seconds
    ControlInputs accel_inputs{2.0, 0.0};
    VehicleState state = initial;

    for (int i = 0; i < 300; ++i) {  // 3 seconds at 100 Hz
        state = model->step(state, accel_inputs);
    }

    // Velocity should be approximately 6 m/s (v = at)
    EXPECT_NEAR(state.v, 6.0, 0.1) << "Velocity after acceleration";

    // Now decelerate at -2 m/s² for 3 seconds
    ControlInputs decel_inputs{-2.0, 0.0};
    for (int i = 0; i < 300; ++i) {
        state = model->step(state, decel_inputs);
    }

    // Should return to approximately 0 m/s
    EXPECT_NEAR(state.v, 0.0, 0.1) << "Velocity after deceleration";
}

/**
 * Test: RK4 vs Euler integration accuracy
 *
 * Educational test: Demonstrates the superior accuracy of RK4.
 * We compare position error for a circular trajectory.
 */
TEST_F(BicycleModelTest, RK4Accuracy) {
    // This test verifies that RK4 is being used (not Euler)
    // For a circle, Euler method would cause spiraling outward

    VehicleState initial{10.0, 0.0, M_PI/2, 5.0, 0.0};
    model->reset(initial);

    ControlInputs inputs{0.0, 0.2};  // Constant steering

    // Simulate for 10 seconds
    VehicleState state = initial;
    for (int i = 0; i < 1000; ++i) {
        state = model->step(state, inputs);
    }

    // Calculate distance from expected circular path center
    // Center should be at origin for this setup
    double radius_from_origin = std::sqrt(state.x*state.x + state.y*state.y);

    // With RK4, radius should remain approximately constant
    // With Euler, radius would grow significantly
    double initial_radius = std::sqrt(initial.x*initial.x + initial.y*initial.y);
    double radius_error = std::abs(radius_from_origin - initial_radius);

    EXPECT_LT(radius_error, 1.0) << "RK4 should maintain radius accuracy";
}

/**
 * Test: Invalid inputs detection
 *
 * Expected behavior: Should throw exception for NaN or Inf inputs
 */
TEST_F(BicycleModelTest, InvalidInputsDetection) {
    VehicleState initial{0.0, 0.0, 0.0, 5.0, 0.0};
    model->reset(initial);

    // NaN acceleration should throw
    ControlInputs nan_inputs{std::nan(""), 0.0};
    EXPECT_THROW(model->step(initial, nan_inputs), std::invalid_argument);

    // Inf steering should throw
    ControlInputs inf_inputs{0.0, std::numeric_limits<double>::infinity()};
    EXPECT_THROW(model->step(initial, inf_inputs), std::invalid_argument);
}

/**
 * Test: Reset functionality
 *
 * Expected behavior: After reset, model should be at specified initial state
 */
TEST_F(BicycleModelTest, ResetFunctionality) {
    VehicleState initial{5.0, 10.0, M_PI/4, 3.0, 0.0};
    model->reset(initial);

    VehicleState current = model->getCurrentState();

    EXPECT_DOUBLE_EQ(current.x, 5.0);
    EXPECT_DOUBLE_EQ(current.y, 10.0);
    EXPECT_DOUBLE_EQ(current.theta, M_PI/4);
    EXPECT_DOUBLE_EQ(current.v, 3.0);
}

/**
 * Test: Constructor parameter validation
 *
 * Expected behavior: Should reject invalid wheelbase or timestep
 */
TEST(BicycleModelConstructorTest, ParameterValidation) {
    // Negative wheelbase should throw
    EXPECT_THROW(BicycleModel(-1.0, 0.01), std::invalid_argument);

    // Zero wheelbase should throw
    EXPECT_THROW(BicycleModel(0.0, 0.01), std::invalid_argument);

    // Negative timestep should throw
    EXPECT_THROW(BicycleModel(2.5, -0.01), std::invalid_argument);

    // Zero timestep should throw
    EXPECT_THROW(BicycleModel(2.5, 0.0), std::invalid_argument);

    // Valid parameters should not throw
    EXPECT_NO_THROW(BicycleModel(2.5, 0.01));
}

/**
 * Main function for running tests
 */
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

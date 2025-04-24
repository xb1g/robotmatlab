# Analysis of Forward and Inverse Kinematics for a 3-DOF Spherical Robot (RRP)

I'll walk you through how to analyze the forward and inverse kinematics and Jacobian motion of a 3-DOF spherical robot (RRP - Revolute, Revolute, Prismatic). This is a systematic approach you can follow to complete this task.

## 1. Kinematic Modeling of the Spherical Robot

### Robot Configuration
The Spherical (RRP) robot consists of:
- First joint: Revolute (rotation about vertical axis) - θ₁
- Second joint: Revolute (rotation in vertical plane) - θ₂
- Third joint: Prismatic (extension/retraction) - d₃

### DH Parameters
First, establish the Denavit-Hartenberg parameters:

| Link | α(i-1) | a(i-1) | d(i) | θ(i) |
|------|--------|--------|------|------|
| 1    | 0      | 0      | 0    | θ₁   |
| 2    | 90°    | 0      | 0    | θ₂   |
| 3    | 0      | 0      | d₃   | 0    |

## 2. Forward Kinematics Analysis

### Step 1: Set up transformation matrices
For each link, calculate the transformation matrix using DH parameters:

T(i-1,i) = [
  cos(θᵢ)  -sin(θᵢ)cos(αᵢ₋₁)  sin(θᵢ)sin(αᵢ₋₁)  aᵢ₋₁cos(θᵢ);
  sin(θᵢ)  cos(θᵢ)cos(αᵢ₋₁)   -cos(θᵢ)sin(αᵢ₋₁)  aᵢ₋₁sin(θᵢ);
  0        sin(αᵢ₋₁)          cos(αᵢ₋₁)           dᵢ;
  0        0                   0                   1
]

### Step 2: Calculate the end-effector position
Multiply the transformation matrices to get the final position:
T₀₃ = T₀₁ × T₁₂ × T₂₃

The position coordinates (x, y, z) can be extracted from T₀₃:
- x = d₃cos(θ₁)cos(θ₂)
- y = d₃sin(θ₁)cos(θ₂)
- z = d₃sin(θ₂)

## 3. Inverse Kinematics Analysis

Given the end-effector position (x, y, z), calculate the joint variables:

### Step 1: Calculate θ₁
θ₁ = atan2(y, x)

### Step 2: Calculate θ₂
r = √(x² + y²)
θ₂ = atan2(z, r)

### Step 3: Calculate d₃
d₃ = √(x² + y² + z²)

## 4. Jacobian Analysis

### Step 1: Form the Jacobian matrix
The Jacobian matrix J relates joint velocities to end-effector velocities:
v = J × q̇

Where:
- v is the end-effector velocity vector [vₓ, vᵧ, vᵣ, ωₓ, ωᵧ, ωᵣ]ᵀ
- q̇ is the joint velocity vector [θ̇₁, θ̇₂, ḋ₃]ᵀ

For the spherical robot, calculate partial derivatives:
J = [
  ∂x/∂θ₁  ∂x/∂θ₂  ∂x/∂d₃;
  ∂y/∂θ₁  ∂y/∂θ₂  ∂y/∂d₃;
  ∂z/∂θ₁  ∂z/∂θ₂  ∂z/∂d₃;
  ...     ...     ...
]

## 5. Verification with MATLAB

Here's how to implement this in MATLAB:

```matlab
% Define symbolic variables
syms theta1 theta2 d3 real

% Forward Kinematics
x = d3*cos(theta1)*cos(theta2);
y = d3*sin(theta1)*cos(theta2);
z = d3*sin(theta2);

% Display forward kinematics equations
disp('Forward Kinematics:');
disp(['x = ', char(x)]);
disp(['y = ', char(y)]);
disp(['z = ', char(z)]);

% Jacobian Matrix (position part only)
J = jacobian([x; y; z], [theta1; theta2; d3]);
disp('Jacobian Matrix:');
disp(J);

% Verification with random sets
disp('Verification with random sets:');
results = zeros(16, 6); % To store results

for i = 1:16
    % Generate random joint variables
    th1 = rand * 2*pi;  % 0 to 2π
    th2 = rand * pi - pi/2;  % -π/2 to π/2
    d_3 = rand * 10;  % 0 to 10 (adjust range as needed)
    
    % Calculate forward kinematics
    x_val = double(subs(x, {theta1, theta2, d3}, {th1, th2, d_3}));
    y_val = double(subs(y, {theta1, theta2, d3}, {th1, th2, d_3}));
    z_val = double(subs(z, {theta1, theta2, d3}, {th1, th2, d_3}));
    
    % Calculate inverse kinematics
    th1_inv = atan2(y_val, x_val);
    r = sqrt(x_val^2 + y_val^2);
    th2_inv = atan2(z_val, r);
    d3_inv = sqrt(x_val^2 + y_val^2 + z_val^2);
    
    % Store results for comparison
    results(i, :) = [th1, th2, d_3, th1_inv, th2_inv, d3_inv];
    
    % Display for verification
    fprintf('Set %d: Original [%.4f, %.4f, %.4f], Inverse [%.4f, %.4f, %.4f]\n', ...
        i, th1, th2, d_3, th1_inv, th2_inv, d3_inv);
end

% Calculate errors
errors = results(:, 1:3) - results(:, 4:6);
mean_abs_error = mean(abs(errors));
disp('Mean Absolute Error:');
disp(mean_abs_error);

% Compare Jacobian motion for different step sizes
% Case 1: Change position once
delta_pos = [0.1; 0.1; 0.1]; % Small change in x, y, z

% Calculate required joint changes using inverse Jacobian
J_eval = double(subs(J, {theta1, theta2, d3}, {th1, th2, d_3}));
delta_q_single = pinv(J_eval) * delta_pos;

% Case 2: Change position ten times with smaller steps
delta_pos_small = [0.01; 0.01; 0.01]; % 1/10 of the original change
delta_q_multiple = zeros(3, 1);

for j = 1:10
    % Recalculate Jacobian at each step for accuracy
    curr_th1 = th1 + delta_q_multiple(1);
    curr_th2 = th2 + delta_q_multiple(2);
    curr_d3 = d_3 + delta_q_multiple(3);
    
    J_curr = double(subs(J, {theta1, theta2, d3}, {curr_th1, curr_th2, curr_d3}));
    delta_q_step = pinv(J_curr) * delta_pos_small;
    delta_q_multiple = delta_q_multiple + delta_q_step;
end

% Compare results
disp('Jacobian Motion Comparison:');
disp('Single step:');
disp(delta_q_single);
disp('Ten small steps:');
disp(delta_q_multiple);
disp('Difference:');
disp(delta_q_single - delta_q_multiple);
```

## 6. Visualization and Analysis

For a more visual approach, you can add this code:

```matlab
% Create a figure for visualization
figure;
hold on;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Spherical Robot Workspace Sample Points');

% Plot the tested points
scatter3(results(:,1), results(:,2), results(:,3), 'filled');

% Visualize single vs multiple step difference
figure;
bar([delta_q_single, delta_q_multiple]);
legend('Single Step', 'Ten Small Steps');
title('Comparison of Joint Changes for Single vs Multiple Steps');
xlabel('Joint Number');
ylabel('Joint Change');
```

## 7. Expected Findings and Analysis

When comparing the Jacobian motion with one step versus ten steps:

1. In a perfectly linear system, both approaches would yield identical results.
2. Due to the nonlinearity of the robot kinematics, the multi-step approach typically provides more accurate results.
3. The difference will be more pronounced when:
   - The robot is near singularities
   - The movements are large
   - The robot is in configurations where the Jacobian changes rapidly

The analysis should highlight:
- The accuracy of inverse kinematics solutions
- The errors introduced by linearization in the Jacobian
- The importance of step size in trajectory planning

This approach should provide a comprehensive analysis of the spherical robot's kinematics and allow you to effectively compare the different motion strategies.


# Documentation: Droplet Evolution Simulation with Level Set Method

## Table of Contents
- [Introduction](#introduction)
- [Background](#background)
  - [What Is the Level Set Method?](#what-is-the-level-set-method)
  - [What Are the Navier-Stokes Equations?](#what-are-the-navier-stokes-equations)
  - [Why Simulate Droplet Dynamics?](#why-simulate-droplet-dynamics)
- [Physics and Governing Equations](#physics-and-governing-equations)
  - [Interface Dynamics with the Level Set Method](#1-interface-dynamics-with-the-level-set-method)
  - [Fluid Dynamics with the Navier-Stokes Equations](#2-fluid-dynamics-with-the-navier-stokes-equations)
  - [Interface Dynamics: Curvature, Surface Tension, and Smoothing Functions](#3-interface-dynamics-curvature-surface-tension-and-smoothing-functions)
    - [Curvature and Surface Tension](#31-curvature-and-surface-tension)
    - [Heaviside and Delta Functions](#32-heaviside-and-delta-functions)
- [Simulation Setup](#simulation-setup)
  - [Domain and Grid](#1-domain-and-grid)
  - [Initial Conditions](#2-initial-conditions)
  - [Physical Parameters](#3-physical-parameters)
  - [Numerical Parameters](#4-numerical-parameters)
- [Code Structure and Files](#code-structure-and-files)
  - [solver.py](#1-solverpy)
  - [bc.py](#2-bcpy)
  - [geo.py](#3-geopy)
  - [initial.py](#4-initialpy)
  - [post.py](#5-postpy)
- [How to Use the Simulation](#how-to-use-the-simulation)
  - [Prerequisites](#1-prerequisites)
  - [File Structure](#2-file-structure)
  - [Running the Simulation](#3-running-the-simulation)
  - [Outputs](#4-outputs)
- [Tuning the Simulation](#tuning-the-simulation)
  - [Physical Parameters](#1-physical-parameters)
  - [Numerical Parameters](#2-numerical-parameters)
  - [Force Toggles](#3-force-toggles)
- [Testing the Simulation](#testing-the-simulation)
  - [Test Cases](#1-test-cases)
  - [Metrics to Monitor](#2-metrics-to-monitor)
  - [Test Procedure](#3-test-procedure)
- [Updating the Simulation](#updating-the-simulation)
  - [Adding New Forces](#1-adding-new-forces)
  - [Improving Numerical Stability](#2-improving-numerical-stability)
  - [Enhancing Visualization](#3-enhancing-visualization)
  - [Performance Optimization](#4-performance-optimization)
- [Known Issues and Limitations](#known-issues-and-limitations)
  - [Common Issues](#1-common-issues)
  - [Troubleshooting](#2-troubleshooting)
- [Conclusion](#conclusion)
- [References](#references)
- [Version History](#version-history)

## Introduction

This documentation provides a comprehensive guide to a 2D droplet evolution simulation that models the dynamics of a droplet pinned on a surface. The simulation uses the **Level Set Method** to track the droplet interface and solves the **Navier-Stokes equations** to compute the velocity and pressure fields. Physical forces such as surface tension, gravity, inertia, pressure, and viscosity are modeled to achieve a realistic equilibrium shape for the droplet while ensuring numerical stability.

The simulation is implemented in Python and leverages several libraries for numerical computation, performance optimization, and visualization:
- **NumPy**: For efficient array operations and numerical computations.
- **Numba**: For Just-In-Time (JIT) compilation to accelerate performance-critical functions.
- **Matplotlib**: For plotting the droplet interface and diagnostic metrics.
- **scikit-fmm**: For reinitializing the level set function as a signed distance function.

The codebase is modular, consisting of five main Python files:
- **`solver.py`**: Core simulation logic, including force computation, Navier-Stokes solver, and level set evolution.
- **`bc.py`**: Utilities for handling boundary conditions, such as upwind gradients.
- **`geo.py`**: Geometric computations, including curvature and interface-related functions.
- **`initial.py`**: Initialization of the simulation domain, fields, and initial conditions.
- **`post.py`**: Post-processing and visualization utilities.

This documentation is intended for researchers, students, and engineers interested in computational fluid dynamics (CFD), interface tracking, and droplet dynamics. It provides detailed explanations of the physics, numerical methods, code structure, usage instructions, tuning tips, and troubleshooting guidance.

---

## Background

### What Is the Level Set Method?
The **Level Set Method** is a numerical technique for tracking interfaces and shapes in a computational domain. It represents the interface implicitly as the zero level set of a higher-dimensional function, $\phi(x, y, t)$, called the level set function. In this simulation:
- $\phi(x, y, t) = 0$ defines the droplet boundary (interface).
- $\phi(x, y, t) < 0$ inside the droplet.
- $\phi(x, y, t) > 0$ outside the droplet.

The Level Set Method is particularly useful for handling topological changes (e.g., merging or breaking of interfaces) and computing geometric properties like curvature. However, it requires periodic reinitialization to maintain $\phi$ as a signed distance function ($|\nabla \phi| = 1$), which ensures accurate curvature calculations.

### What Are the Navier-Stokes Equations?
The **Navier-Stokes equations** describe the motion of incompressible fluids, accounting for forces such as pressure, viscosity, and external forces (e.g., gravity, surface tension). They consist of:
- **Momentum Equation**: Balances forces acting on the fluid.
- **Continuity Equation**: Ensures the fluid is incompressible (i.e., $\nabla \cdot \mathbf{u} = 0$).

In this simulation, the Navier-Stokes equations are solved numerically to compute the velocity field $\mathbf{u} = (u, v)$ and pressure $p$, which drive the evolution of the droplet interface.

### Why Simulate Droplet Dynamics?
Droplet dynamics are critical in many applications, including inkjet printing, microfluidics, coating processes, and biological systems (e.g., cell membrane dynamics). This simulation focuses on a pinned droplet on a surface, where forces like surface tension (which minimizes surface area) and gravity (which flattens the droplet) compete to determine the equilibrium shape. Understanding these dynamics helps in designing better materials, optimizing industrial processes, and studying natural phenomena.

---

## Physics and Governing Equations

### 1. Interface Dynamics with the Level Set Method

#### Purpose
The Level Set Method tracks the droplet interface implicitly, allowing for accurate computation of geometric properties (e.g., curvature) and handling of complex interface motion.

#### Equations
The level set function $\phi(x, y, t)$ evolves according to the advection equation:
$$
\frac{\partial \phi}{\partial t} + \mathbf{u} \cdot \nabla \phi = 0
$$
where:
- $\mathbf{u} = (u, v)$: Velocity field, computed from the Navier-Stokes equations.
- $\nabla \phi$: Gradient of the level set function, representing the direction of interface motion.

To maintain $\phi$ as a signed distance function ($|\nabla \phi| = 1$), it is periodically reinitialized using the `distance` function from the `scikit-fmm` library. This ensures numerical stability and accurate curvature calculations.

#### Implementation Details
- The advection equation is solved using a first-order upwind scheme in the `advect_level_set` function.
- Reinitialization occurs every 10 time steps to prevent $\phi$ from becoming too distorted.

#### Usage Notes
- The Level Set Method can introduce numerical diffusion, leading to area loss. This is mitigated by monitoring area errors and applying corrections if needed.
- Reinitialization frequency can be adjusted based on the simulation’s stability requirements.

### 2. Fluid Dynamics with the Navier-Stokes Equations

#### Purpose
The Navier-Stokes equations govern the motion of the fluid, determining the velocity field $\mathbf{u} = (u, v)$ and pressure $p$ that drive the droplet’s evolution.

#### Equations
The incompressible Navier-Stokes equations are:
- **Momentum Equation**:
  $$
  \rho \left( \frac{\partial \mathbf{u}}{\partial t} + (\mathbf{u} \cdot \nabla) \mathbf{u} \right) = -\nabla p + \mu \nabla^2 \mathbf{u} + \mathbf{F}
  $$
- **Continuity Equation**:
  $$
  \nabla \cdot \mathbf{u} = 0
  $$
where:
- $\rho$: Density (kg/m³).
- $\mu$: Dynamic viscosity (Pa·s).
- $\mathbf{F}$: External forces per unit volume (N/m³), including gravity, surface tension, inertia, pressure, and viscous forces.

The forces $\mathbf{F} = (F_x, F_y)$ are computed as:
- **Gravity**: $F_y = -\rho g$, where $g = 9.81 \, \text{m/s}^2$.
- **Surface Tension**: $F_x = -\sigma (\kappa - \bar{\kappa}) \frac{\partial \phi}{\partial x}$, $F_y = -\sigma (\kappa - \bar{\kappa}) \frac{\partial \phi}{\partial y}$, where $\sigma$ is the surface tension coefficient, $\kappa$ is the local curvature, and $\bar{\kappa}$ is the average curvature.
- **Inertia**: $F_x = -\rho \left( u \frac{\partial u}{\partial x} + v \frac{\partial u}{\partial y} \right)$, $F_y = -\rho \left( u \frac{\partial v}{\partial x} + v \frac{\partial v}{\partial y} \right)$.
- **Pressure**: $F_x = -\frac{\partial p}{\partial x}$, $F_y = -\frac{\partial p}{\partial y}$.
- **Viscous**: $F_x = \mu \left( \frac{\partial^2 u}{\partial x^2} + \frac{\partial^2 u}{\partial y^2} \right)$, $F_y = \mu \left( \frac{\partial^2 v}{\partial x^2} + \frac{\partial^2 v}{\partial y^2} \right)$.

#### Implementation Details
- The Navier-Stokes equations are solved using a projection method in `solve_naviers_stokes`:
  1. Compute intermediate velocities using forces and viscous terms.
  2. Apply a pressure projection (10 iterations) to enforce incompressibility.
  3. Update velocities with damping to reduce oscillations.
- Forces are clipped to $[-10, 10]$ to prevent numerical overflow.

#### Usage Notes
- The pressure projection may require more iterations for highly dynamic flows.
- Damping can be adjusted to control velocity growth, especially in early time steps.

### 3. Interface Dynamics: Curvature, Surface Tension, and Smoothing Functions

#### 3.1 Curvature and Surface Tension

##### Purpose
The curvature $\kappa$ of the droplet interface drives surface tension forces, which minimize the surface area of the droplet. The average curvature $\bar{\kappa}$ is used to ensure no net translation due to surface tension.

##### Equations
- **Local Curvature**:
  $$
  \kappa = \nabla \cdot \left( \frac{\nabla \phi}{|\nabla \phi|} \right)
  $$
- **Average Curvature**:
  $$
  \bar{\kappa} = \frac{\int \kappa \delta(\phi) \, dA}{\int \delta(\phi) \, dA}
  $$
- **Surface Tension Force**:
  - $F_x = -\sigma (\kappa - \bar{\kappa}) \frac{\partial \phi}{\partial x}$
  - $F_y = -\sigma (\kappa - \bar{\kappa}) \frac{\partial \phi}{\partial y}$

##### Implementation Details
- Curvature is computed using central differences in `compute_curvature`.
- The average curvature is calculated in `compute_average_curvature` using the Dirac delta function to weight the interface.
- Surface tension forces are computed in `compute_ns_forces`.

##### Usage Notes
- Low grid resolution can lead to inaccurate curvature estimates. Increase $N_x$ and $N_y$ for better accuracy.
- The surface tension coefficient $\sigma$ can be adjusted to control the strength of the force.

#### 3.2 Heaviside and Delta Functions

##### Purpose
The Heaviside and Dirac delta functions are used to compute properties of the droplet (e.g., area, average curvature) by weighting quantities at the interface.

##### Equations
- **Heaviside Function**:
  $$
  H(\phi) = \begin{cases} 
  0 & \text{if } \phi < -\epsilon \\ 
  \frac{1}{2} \left( 1 + \frac{\phi}{\epsilon} + \frac{1}{\pi} \sin\left(\frac{\pi \phi}{\epsilon}\right) \right) & \text{if } |\phi| \leq \epsilon \\ 
  1 & \text{if } \phi > \epsilon 
  \end{cases}
  $$
  where $\epsilon = 1.5 \times dx$.
- **Dirac Delta Function**:
  $$
  \delta(\phi) = \begin{cases} 
  \frac{1}{2\epsilon} \left( 1 + \cos\left(\frac{\pi \phi}{\epsilon}\right) \right) & \text{if } |\phi| \leq \epsilon \\ 
  0 & \text{otherwise}
  \end{cases}
  $$

##### Implementation Details
- The Heaviside function is implemented in `compute_heaviside` using a smoothed approximation to avoid discontinuities.
- The Dirac delta function is implemented in `compute_delta_function` as the derivative of the Heaviside function.

##### Usage Notes
- The smoothing parameter $\epsilon$ controls the width of the transition region. Adjust it based on grid resolution (default: $1.5 \times dx$).
- These functions are critical for area conservation and curvature averaging.

---

## Simulation Setup

### 1. Domain and Grid

#### Purpose
The simulation domain defines the physical space where the droplet evolves, and the grid discretizes this space for numerical computation.

#### Equations
- **Domain**: $[-0.6, 0.6] \times [0, 0.12]$ meters (width 1.2 m, height 0.12 m).
- **Grid Size**: $N_x \times N_y = 300 \times 300$ (default).
- **Grid Spacing**:
  $$
  dx = \frac{1.2}{N_x}, \quad dy = \frac{0.12}{N_y}
  $$

#### Implementation Details
- The domain and grid are initialized in `initialize_field` in `initial.py`.
- Grid coordinates $(X, Y)$ are created using NumPy’s `meshgrid`.

#### Usage Notes
- Increase $N_x$ and $N_y$ (e.g., to 400x400) for better resolution of the interface, at the cost of increased computation time.
- Ensure $N_x$ and $N_y$ are proportional to the domain dimensions to maintain square grid cells ($dx \approx dy$).

### 2. Initial Conditions

#### Purpose
Initial conditions define the starting state of the simulation, including the droplet shape, velocity, and pressure.

#### Details
- **Level Set Function ($\phi$)**: Initialized as a semicircular droplet pinned at the bottom center of the domain.
- **Velocity ($(u, v)$) and Pressure ($p$)**: Initialized to zero.
- **Pinning Mask**: Fixes the contact line at the bottom boundary to simulate a pinned droplet.

#### Implementation Details
- `initialize_field` in `initial.py` sets up $\phi$ using a semicircular shape.
- `initialize_velocity_pressure` in `initial.py` sets $(u, v)$ and $p$ to zero.

#### Usage Notes
- The initial droplet radius can be adjusted in `initialize_field` to simulate different droplet sizes.
- The pinning mask ensures the contact line remains fixed, mimicking a no-slip boundary condition at the surface.

### 3. Physical Parameters

#### Purpose
Physical parameters define the material properties and external forces acting on the droplet.

#### Details
- **Density ($\rho$)**: $100 \, \text{kg/m}^3$ (lighter fluid to balance gravity).
- **Viscosity ($\mu$)**: $0.05 , \text{Pa} \cdot \text{s}$ (increased for damping).
- **Gravity ($g$)**: $9.81 \, \text{m/s}^2$.
- **Surface Tension ($\sigma$)**: $0.072 \, \text{N/m}$ (typical for a water-air interface).

#### Implementation Details
- These parameters are passed to `simple_sdf_evolution` in `solver.py` and used in `compute_ns_forces`.

#### Usage Notes
- Adjust $\rho$ and $\mu$ to control the balance between gravity and viscous effects.
- $\sigma$ can be increased to make the droplet more rounded (stronger surface tension).

### 4. Numerical Parameters

#### Purpose
Numerical parameters control the stability and accuracy of the simulation.

#### Details
- **Time Step**: Computed dynamically using the CFL condition, with a `max_speed` cap of 2.0.
- **Reinitialization Frequency**: Every 10 steps.
- **Force Clipping Range**: $[-10, 10]$.
- **Damping Factor**: Applied to velocity updates to reduce oscillations.
- **Pressure Projection Iterations**: 10 iterations to enforce incompressibility.

#### Implementation Details
- Time step computation is handled in `compute_max_stable_dt`.
- Reinitialization and force clipping occur in the main simulation loop in `simple_sdf_evolution`.

#### Usage Notes
- Adjust the `max_speed` cap to control the time step size (smaller cap = smaller $dt$, more stable but slower).
- Increase pressure projection iterations for better incompressibility in dynamic flows.

---

## Code Structure and Files

### 1. `solver.py`

#### Purpose
The `solver.py` file contains the core simulation logic, orchestrating the evolution of the droplet by computing forces, solving the Navier-Stokes equations, and advecting the level set function.

#### Key Functions

- **`compute_upwind_hamiltonian(phi, F, dx, dy)`**:
  - **Purpose**: Computes the upwind Hamiltonian for surface tension-driven evolution, used when only surface tension is active.
  - **Equation**:
    $$
    H = \max(F, 0) \sqrt{(\max(D_x^-, 0))^2 + (\min(D_x^+, 0))^2 + (\max(D_y^-, 0))^2 + (\min(D_y^+, 0))^2} + \min(F, 0) \sqrt{(\min(D_x^-, 0))^2 + (\max(D_x^+, 0))^2 + (\min(D_y^-, 0))^2 + (\max(D_y^+, 0))^2}
    $$
  - **Inputs**:
    - `phi`: Level set function ($\phi$).
    - `F`: Forcing term.
    - `dx, dy`: Grid spacings.
  - **Outputs**: Hamiltonian $H$, used to evolve $\phi$.
  - **Usage Notes**: This function is critical for surface tension-only simulations, ensuring the interface evolves to minimize surface area.

- **`compute_max_stable_dt(phi, F, dx, dy)`**:
  - **Purpose**: Computes a stable time step using the CFL condition to ensure numerical stability.
  - **Equation**:
    $$
    dt = \frac{\min(dx, dy)}{\text{max speed}}
    $$
  - **Inputs**:
    - `phi`: Level set function.
    - `F`: Forcing term.
    - `dx, dy`: Grid spacings.
  - **Outputs**: Time step $dt$.
  - **Usage Notes**: The `max_speed` cap (default 2.0) prevents excessively small time steps, but can be adjusted for stability vs. speed trade-offs.

- **`compute_ns_forces(phi, u, v, p, dx, dy, rho, mu, g, surface_tension, ...)`**:
  - **Purpose**: Computes all forces acting on the fluid: surface tension, inertia, gravity, pressure, and viscous forces.
  - **Inputs**:
    - `phi, u, v, p`: Level set function, velocity components, and pressure.
    - `dx, dy`: Grid spacings.
    - `rho`: Density ($100 \, \text{kg/m}^3$).
    - `mu:` Viscosity ($0.05 , \text{Pa} \cdot \text{s}$).
    - `g`: Gravity ($9.81 \, \text{m/s}^2$).
    - `surface_tension`: Surface tension coefficient ($0.072 \, \text{N/m}$).
  - **Outputs**: Force components $F_x, F_y$.
  - **Usage Notes**: Forces are clipped to $[-10, 10]$ to prevent numerical overflow. Adjust clipping range if needed.

- **`solve_naviers_stokes(u, v, p, force_x, force_y, dx, dy, dt, rho)`**:
  - **Purpose**: Updates the velocity field $(u, v)$ and pressure $p$ by solving the Navier-Stokes equations.
  - **Inputs**:
    - `u, v, p`: Current velocity and pressure fields.
    - `force_x, force_y`: Force components.
    - `dx, dy, dt, rho`: Grid spacings, time step, and density.
  - **Outputs**: Updated $u, v, p$.
  - **Usage Notes**: Includes damping and a pressure projection (10 iterations) to enforce $\nabla \cdot \mathbf{u} = 0$.

- **`advect_level_set(phi, u, v, dx, dy, dt)`**:
  - **Purpose**: Evolves the level set function $\phi$ using the advection equation.
  - **Equation**:
    $$
    \phi^{n+1} = \phi^n - dt (\mathbf{u} \cdot \nabla \phi)
    $$
  - **Inputs**:
    - `phi, u, v`: Level set function and velocity field.
    - `dx, dy, dt`: Grid spacings and time step.
  - **Outputs**: Updated $\phi$.
  - **Usage Notes**: Uses a first-order upwind scheme, which may introduce numerical diffusion. Consider higher-order schemes for better accuracy.

- **`simple_sdf_evolution(...)`**:
  - **Purpose**: Main simulation loop, orchestrating the entire evolution process.
  - **Inputs**:
    - `amplitude, width_guess`: For theoretical shape plotting.
    - `num_steps`: Number of time steps (default 500).
    - `plot_interval`: Frequency of plotting (default 50).
    - `Nx, Ny`: Grid size (default 300x300).
    - `rho, mu, g, surface_tension`: Physical parameters.
    - `use_surface_tension, use_inertia, use_gravity, use_pressure, use_viscous`: Toggle forces.
  - **Outputs**:
    - Final $\phi$.
    - Lists of residuals, area errors, and simulation times.
  - **Usage Notes**: This is the entry point for running the simulation. Adjust `num_steps` and `plot_interval` based on desired runtime and visualization frequency.

#### Simulation Loop
1. Initialize $\phi$, $(u, v)$, $p$, and the pinning mask.
2. For each time step:
   - Compute forces (or Hamiltonian if surface tension only).
   - Compute $dt$ using the CFL condition.
   - Update $(u, v)$ and $p$ using the Navier-Stokes solver.
   - Advect $\phi$.
   - Reinitialize $\phi$ every 10 steps.
   - Compute residuals and area errors.
   - Plot the interface every `plot_interval` steps.

### 2. `bc.py`

#### Purpose
The `bc.py` file provides utilities for handling boundary conditions, particularly for computing upwind gradients used in surface tension evolution.

#### Key Functions

- **`compute_upwind_gradient(phi, dx, dy)`**:
  - **Purpose**: Computes upwind gradients of $\phi$ in the $x$ and $y$ directions, which are used in the upwind Hamiltonian for surface tension evolution.
  - **Equations**:
    $$
    D_x^- = \frac{\phi_{i,j} - \phi_{i-1,j}}{dx}, \quad D_x^+ = \frac{\phi_{i+1,j} - \phi_{i,j}}{dx}
    $$
    $$
    D_y^- = \frac{\phi_{i,j} - \phi_{i,j-1}}{dy}, \quad D_y^+ = \frac{\phi_{i,j+1} - \phi_{i,j}}{dy}
    $$
  - **Inputs**:
    - `phi`: Level set function.
    - `dx, dy`: Grid spacings.
  - **Outputs**: Upwind gradients $D_x^-, D_x^+, D_y^-, D_y^+$.
  - **Usage Notes**: Used in `compute_upwind_hamiltonian` to ensure stable evolution of the interface under surface tension.

### 3. `geo.py`

#### Purpose
The `geo.py` file handles geometric computations related to the level set function, such as gradients, curvature, and interface smoothing functions.

#### Key Functions

- **`compute_gradient(phi, dx, dy)`**:
  - **Purpose**: Computes the gradient of $\phi$ using central differences, used for curvature and surface tension calculations.
  - **Equation**:
    $$
    \frac{\partial \phi}{\partial x} \approx \frac{\phi_{i+1,j} - \phi_{i-1,j}}{2 \, dx}, \quad \frac{\partial \phi}{\partial y} \approx \frac{\phi_{i,j+1} - \phi_{i,j-1}}{2 \, dy}
    $$
  - **Inputs**:
    - `phi`: Level set function.
    - `dx, dy`: Grid spacings.
  - **Outputs**: Gradient components $\frac{\partial \phi}{\partial x}, \frac{\partial \phi}{\partial y}$.
  - **Usage Notes**: Central differences provide second-order accuracy but may introduce oscillations near sharp interfaces.

- **`compute_curvature(phi, dx, dy)`**:
  - **Purpose**: Computes the local curvature $\kappa$ of the interface, which drives surface tension forces.
  - **Equation**:
    $$
    \kappa = \nabla \cdot \left( \frac{\nabla \phi}{|\nabla \phi|} \right)
    $$
  - **Inputs**:
    - `phi`: Level set function.
    - `dx, dy`: Grid spacings.
  - **Outputs**: Curvature $\kappa$.
  - **Usage Notes**: Uses central differences to approximate derivatives, which may require a fine grid for accuracy.

- **`compute_average_curvature(phi, kappa, dx, dy)`**:
  - **Purpose**: Computes the average curvature $\bar{\kappa}$ over the interface to ensure no net translation due to surface tension.
  - **Equation**:
    $$
    \bar{\kappa} = \frac{\int \kappa \delta(\phi) \, dA}{\int \delta(\phi) \, dA}
    $$
  - **Inputs**:
    - `phi`: Level set function.
    - `kappa`: Local curvature.
    - `dx, dy`: Grid spacings.
  - **Outputs**: Average curvature $\bar{\kappa}$.
  - **Usage Notes**: The Dirac delta function $\delta(\phi)$ weights the interface, ensuring accurate averaging.

- **`compute_heaviside(phi, dx)`**:
  - **Purpose**: Approximates the Heaviside function to identify regions inside and outside the droplet.
  - **Equation**:
    $$
    H(\phi) = \begin{cases} 
    0 & \text{if } \phi < -\epsilon \\ 
    \frac{1}{2} \left( 1 + \frac{\phi}{\epsilon} + \frac{1}{\pi} \sin\left(\frac{\pi \phi}{\epsilon}\right) \right) & \text{if } |\phi| \leq \epsilon \\ 
    1 & \text{if } \phi > \epsilon 
    \end{cases}
    $$
  - **Inputs**:
    - `phi`: Level set function.
    - `dx`: Grid spacing (used to compute $\epsilon$).
  - **Outputs**: Heaviside function $H(\phi)$.
  - **Usage Notes**: $\epsilon = 1.5 \times dx$ controls the smoothing width. Adjust $\epsilon$ for finer grids.

- **`compute_delta_function(phi, dx)`**:
  - **Purpose**: Approximates the Dirac delta function, used for weighting quantities at the interface.
  - **Equation**:
    $$
    \delta(\phi) = \begin{cases} 
    \frac{1}{2\epsilon} \left( 1 + \cos\left(\frac{\pi \phi}{\epsilon}\right) \right) & \text{if } |\phi| \leq \epsilon \\ 
    0 & \text{otherwise}
    \end{cases}
    $$
  - **Inputs**:
    - `phi`: Level set function.
    - `dx`: Grid spacing (used to compute $\epsilon$).
  - **Outputs**: Dirac delta function $\delta(\phi)$.
  - **Usage Notes**: Used in area calculations and curvature averaging. Ensure $\epsilon$ matches the grid resolution.

### 4. `initial.py`

#### Purpose
The `initial.py` file handles the initialization of the simulation domain, fields, and initial conditions.

#### Key Functions

- **`initialize_field(Nx, Ny)`**:
  - **Purpose**: Initializes the level set function $\phi$, grid coordinates $(X, Y)$, and pinning mask.
  - **Inputs**:
    - `Nx, Ny`: Grid dimensions (default 300x300).
  - **Outputs**:
    - `phi`: Level set function (semicircular droplet).
    - `X, Y`: Grid coordinates.
    - `x, y`: 1D coordinate arrays.
    - `pinning_mask`: Binary mask to fix the contact line.
    - `initial_area`: Initial droplet area.
  - **Usage Notes**: The semicircular shape is centered at the bottom of the domain. Adjust the radius for different droplet sizes.

- **`initialize_velocity_pressure(Nx, Ny)`**:
  - **Purpose**: Initializes the velocity $(u, v)$ and pressure $p$ fields to zero.
  - **Inputs**:
    - `Nx, Ny`: Grid dimensions.
  - **Outputs**:
    - `u, v`: Velocity components.
    - `p`: Pressure field.
  - **Usage Notes**: Zero initialization assumes the fluid starts at rest. Modify for non-zero initial velocities if needed.

### 5. `post.py`

#### Purpose
The `post.py` file provides utilities for post-processing and visualization of simulation results.

#### Key Functions

- **`plot_theoretical_shape(X, Y, initial_area, width_guess, ax)`**:
  - **Purpose**: Plots a theoretical semicircular shape for comparison with the simulated droplet.
  - **Inputs**:
    - `X, Y`: Grid coordinates.
    - `initial_area`: Initial droplet area.
    - `width_guess`: Estimated width of the droplet.
    - `ax`: Matplotlib axis for plotting.
  - **Outputs**: None (plots directly to the axis).
  - **Usage Notes**: The theoretical shape assumes a perfect semicircle, which may differ from the simulated shape due to gravity and other forces.

- **`plot_residuals_and_area(residuals, area_errors, times)`**:
  - **Purpose**: Plots residuals and area errors over time to assess convergence and area conservation.
  - **Inputs**:
    - `residuals`: List of residuals over time.
    - `area_errors`: List of area errors (%).
    - `times`: List of simulation times.
  - **Outputs**: None (generates plots).
  - **Usage Notes**: Monitor these plots to ensure the simulation converges (residuals decrease) and conserves area (error stabilizes).

---

## How to Use the Simulation

### 1. Prerequisites

#### Dependencies
Install the required Python libraries with the following versions:
- **Python**: 3.8 or higher.
- **NumPy**: 1.21.0 or higher (`pip install numpy`).
- **Numba**: 0.53.0 or higher (`pip install numba`).
- **Matplotlib**: 3.4.0 or higher (`pip install matplotlib`).
- **scikit-fmm**: 2021.1.0 or higher (`pip install scikit-fmm`).

Install all dependencies in one command:
```bash
pip install numpy numba matplotlib scikit-fmm
```

#### System Requirements
- **Operating System**: Windows, macOS, or Linux.
- **Memory**: At least 4 GB RAM (8 GB recommended for larger grids).
- **Disk Space**: Minimal (code and outputs are small, < 10 MB).

### 2. File Structure
Ensure all Python files are in the same directory:
- `solver.py`
- `bc.py`
- `geo.py`
- `initial.py`
- `post.py`
- `main.py`

### 3. Running the Simulation

#### Main Script
open the script (e.g., `main.py`) to run the simulation:
```python
from solver import simple_sdf_evolution
from post import plot_residuals_and_area

# Run the simulation with default parameters
phi, residuals, area_errors, times = simple_sdf_evolution(
    num_steps=200000,
    plot_interval=1000,
    Nx=300,
    Ny=300,
    rho=100.0,
    mu=0.05,
    g=9.81,
    surface_tension=0.072,
    use_surface_tension=True,
    use_inertia=True,
    use_gravity=True,
    use_pressure=True,
    use_viscous=True
)

# Plot residuals and area errors
plot_residuals_and_area(residuals, area_errors, times)
```

#### Steps to Run
1. Save the script as `main.py` in the same directory as the other files (if it isn't) .
2. Run the script:
   ```bash
   python main.py
   ```
3. Monitor the output:
   - Plots will appear every `plot_interval` steps (default 1000).
   - Progress output will print every 50,000 steps.

### 4. Outputs

#### Plots
- **Interface Plot**: Shows the droplet interface (blue) and initial theoretical shape (green dashed line) every `plot_interval` steps.
- **Residuals and Area Errors**: Plots of residuals and area errors over time, generated at the end of the simulation.

#### Progress Output
Every 50,000 steps, the simulation prints:
- Step number.
- Simulation time (seconds).
- Residual (measure of convergence).
- Area error (%).

#### Returned Values
- `phi`: Final level set function.
- `residuals`: List of residuals over time.
- `area_errors`: List of area errors (%).
- `times`: List of simulation times.

---

## Tuning the Simulation

### 1. Physical Parameters

#### Density ($\rho$)
- **Default**: $100 \, \text{kg/m}^3$.
- **Purpose**: Controls the effect of gravity and inertia.
- **Tuning**:
  - **Lower ($50 \, \text{kg/m}^3$)**: Reduces gravity’s effect, preventing flattening.
  - **Higher ($200 \, \text{kg/m}^3$)**: Increases gravity’s effect, leading to more flattening.
- **Example**: If the droplet flattens too much, set `rho=50.0` in `simple_sdf_evolution`.

#### Viscosity ($\mu$)
- **Default**: $0.05 \, \text{Pa·s}$.
- **Purpose**: Controls damping of velocity oscillations.
- **Tuning**:
 -  **Lower ($0.01 , \text{Pa} \cdot \text{s}$)**: Less damping, more dynamic motion.
-  **Higher ($0.1 , \text{Pa} \cdot \text{s}$)**: More damping, reduces oscillations but may slow convergence.convergence.
- **Example**: For a more stable simulation, set `mu=0.1`.

#### Surface Tension ($\sigma$)
- **Default**: $0.072 \, \text{N/m}$ (water-air interface).
- **Purpose**: Controls the strength of surface tension, which minimizes surface area.
- **Tuning**:
  - **Lower ($0.05 \, \text{N/m}$)**: Weaker surface tension, droplet spreads more.
  - **Higher ($0.1 \, \text{N/m}$)**: Stronger surface tension, droplet becomes more rounded.
- **Example**: To make the droplet more rounded, set `surface_tension=0.1`.

#### Gravity ($g$)
- **Default**: $9.81 \, \text{m/s}^2$.
- **Purpose**: Simulates gravitational force.
- **Tuning**: Typically fixed at $9.81 \, \text{m/s}^2$ for realism. Adjust $\rho$ instead to control gravity’s effect.

### 2. Numerical Parameters

#### Grid Size ($N_x, N_y$)
- **Default**: $300 \times 300$.
- **Purpose**: Controls spatial resolution.
- **Tuning**:
  - **Increase ($400 \times 400$)**: Better curvature resolution, but slower.
  - **Decrease ($150 \times 150$)**: Faster, but less accurate.
- **Example**: For better accuracy, set `Nx=400, Ny=400`.

#### Time Step
- **Default**: Computed dynamically with `max_speed=2.0`.
- **Purpose**: Ensures numerical stability via the CFL condition.
- **Tuning**:
  - **Lower `max_speed` ($1.0$)**: Smaller $dt$, more stable but slower.
  - **Higher `max_speed` ($3.0$)**: Larger $dt$, faster but less stable.
- **Example**: For more stability, set `max_speed=1.0` in `compute_max_stable_dt`.

#### Force Clipping
- **Default**: $[-10, 10]$.
- **Purpose**: Prevents numerical overflow due to large forces.
- **Tuning**:
  - **Tighter ($[-5, 5]$)**: More conservative, reduces risk of overflow.
  - **Looser ($[-20, 20]$)**: Allows larger forces, but riskier.
- **Example**: If overflow occurs, set the clipping range to $[-5, 5]$ in `compute_ns_forces`.

#### Damping Factor
- **Default**: Applied in `solve_naviers_stokes`.
- **Purpose**: Reduces velocity oscillations.
- **Tuning**:
  - **Increase ($2.0$)**: More damping, reduces oscillations.
  - **Decrease ($0.5$)**: Less damping, allows more dynamic motion.
- **Example**: For less oscillation, set the damping factor to 2.0.

#### Pressure Projection Iterations
- **Default**: 10 iterations.
- **Purpose**: Enforces incompressibility ($\nabla \cdot \mathbf{u} = 0$).
- **Tuning**:
  - **Increase ($20$)**: Better incompressibility, but slower.
  - **Decrease ($5$)**: Faster, but less accurate.
- **Example**: For better accuracy, set iterations to 20 in `solve_naviers_stokes`.

### 3. Force Toggles
- **Purpose**: Isolate specific forces for debugging or analysis.
- **Toggles**:
  - `use_surface_tension=True, use_inertia=False, use_gravity=False, use_pressure=False, use_viscous=False`: Surface tension only.
  - `use_gravity=True, use_surface_tension=False`: Gravity only.
- **Example**: To study surface tension effects, set only `use_surface_tension=True`.

---

## Testing the Simulation

### 1. Test Cases

#### Surface Tension Only
- **Setup**: Set `use_surface_tension=True`, all other forces to `False`.
- **Expected Result**: The droplet should evolve into a semicircular shape, as surface tension minimizes surface area.
- **Metrics**:
  - Area error: Should be < 4%.
  - Residual: Should decrease to < 1.

#### All Forces
- **Setup**: Use default settings (all forces enabled).
- **Expected Result**: The droplet should deform under gravity but stabilize into a rounded shape pinned at the bottom.
- **Metrics**:
  - Area error: Should stabilize between 2-4%.
  - Residual: Should decrease to < 1.

### 2. Metrics to Monitor
- **Area Error**: Measures area conservation. Should stabilize between 2-4%.
- **Residual**: Measures convergence. Should decrease over time (e.g., below 0.01).
- **Droplet Shape**: Should be rounded, not flattened, and pinned at the bottom.

### 3. Test Procedure
1. Run the simulation with `num_steps=1000000`, `plot_interval=1000`.
2. Monitor progress output at steps 50,000, 100,000, 150,000, 200,000,etc..:
   - Check simulation time, residual, and area error.
3. Inspect the final plot at step 1,000,000:
   - Verify the droplet shape is realistic and matches the theoretical shape (green dashed line).

---

## Updating the Simulation

### 1. Adding New Forces
- **Steps**:
  1. Modify `compute_ns_forces` to include the new force term.
  2. Add a toggle (`use_new_force`) in the function signature.
- **Example**: Add a body force $F_{\text{body}} = (F_x, F_y)$:
  ```python
  if use_body_force:
      body_x = some_function(i, j)
      body_y = some_function(i, j)
  else:
      body_x, body_y = 0.0, 0.0
  force_x[i,j] += body_x
  force_y[i,j] += body_y
  ```

### 2. Improving Numerical Stability
- **Reinitialization Frequency**: Decrease to every 5 steps if $\phi$ deviates from a signed distance function.
- **Advection Scheme**: Implement a higher-order scheme (e.g., WENO) in `advect_level_set` for better accuracy.

### 3. Enhancing Visualization
- **Velocity Field**: Add a quiver plot in `simple_sdf_evolution`:
  ```python
  ax.quiver(X[::10, ::10], Y[::10, ::10], u[::10, ::10], v[::10, ::10], color='r')
  ```
- **Curvature Plot**: Plot the curvature field using `compute_curvature` to visualize interface properties.

### 4. Performance Optimization
- **Profiling**: Use `cProfile` to identify bottlenecks:
  ```bash
  python -m cProfile -o profile.out main.py
  ```
- **Parallelization**: Use Numba’s `@njit(parallel=True)` for more functions.
- **Grid Size**: Reduce to 150x150 for faster testing.

---

## Known Issues and Limitations

### 1. Common Issues
- **Flattening**: If $\rho$ is too high, gravity dominates, causing the droplet to flatten.
- **Numerical Instability**: Large $dt$ or unclipped forces can cause overflow.
- **Area Conservation**: Area error may exceed 4% due to numerical diffusion.
- **Curvature Accuracy**: Low grid resolution can underestimate $\kappa$.

### 2. Troubleshooting
- **Flattening**:
  - **Solution**: Reduce $\rho$ (e.g., to $50 \, \text{kg/m}^3$) or increase $\mu$ (e.g., to $0.1 \, \text{Pa·s}$).
- **Numerical Instability**:
  - **Solution**: Lower the `max_speed` cap (e.g., to 1.0) and tighten force clipping (e.g., to $[-5, 5]$).
- **Area Conservation**:
  - **Solution**: Add a mass correction term in `simple_sdf_evolution`:
    ```python
    H = compute_heaviside(phi, dx)
    current_area = np.sum(H) * dx * dy
    area_error = (current_area - initial_area) / initial_area
    phi = phi - 0.1 * area_error * dx
    ```
- **Curvature Accuracy**:
  - **Solution**: Increase grid resolution (e.g., $N_x = N_y = 400$).

---

## Conclusion

This simulation provides a robust and flexible framework for modeling droplet dynamics using the Level Set Method and Navier-Stokes equations. It balances physical accuracy with numerical stability, making it suitable for studying droplet behavior under various forces. By tuning parameters and following the usage guidelines, users can achieve realistic results for a pinned droplet on a surface. Future improvements could include higher-order numerical schemes, advanced pressure solvers, and support for more complex force models.

---

## References

- **Level Set Method**:
  - Osher, S., & Sethian, J. A. (1988). "Fronts propagating with curvature-dependent speed: Algorithms based on Hamilton-Jacobi formulations." *Journal of Computational Physics*, 77(1), 12-49.
- **Navier-Stokes Equations**:
  - Chorin, A. J. (1968). "Numerical solution of the Navier-Stokes equations." *Mathematics of Computation*, 22(104), 745-762.
- **Droplet Dynamics**:
  - De Gennes, P. G., Brochard-Wyart, F., & Quéré, D. (2004). *Capillarity and Wetting Phenomena: Drops, Bubbles, Pearls, Waves*. Springer.
- **scikit-fmm**:
  - Official documentation: [https://pythonhosted.org/scikit-fmm/](https://pythonhosted.org/scikit-fmm/).

---

## Version History

- **Version 1.0 (April 2025)**:
  - Initial release of the simulation and documentation.
  - Supports surface tension, gravity, inertia, pressure, and viscous forces.
  - Uses first-order upwind scheme for advection.
- **Version 1.1 (Future)**:
  - Planned: Higher-order advection schemes (e.g., WENO).
  - Planned: Conjugate gradient pressure solver.

# Position Based Dynamics

#### Overview

Dynamics simulation featuring:
- Cloth simulation
- Rigid body simulation
- Collisions with static objects
- Two way interactions between dynamic objects

Hosted on Github here: https://github.com/EmperorJack/PositionBasedDynamics

#### Paper

Matthias Müller, Bruno Heidelberger, Marcus Hennix, and John Ratcliff. 2007. Position based dynamics.
J. Vis. Comun. Image Represent. 18, 2 (April 2007), 109-118

#### Installation

Cmake files provided. Tested on:
- Windows (MinGW, Visual Studio 15 2017)
- Linux
- Mac (LLVM - Clang)

#### Running the program

To ensure that the resources are loaded correctly the executable needs to be run from a `/build` directory e.g:
`<PathToPositionBasedDynamics/build> .\PositionBasedDynamics.exe`

#### Interactions

To rotate the camera simply click and drag across the screen, it's not very intuitive right now but it works.

To reset the current scene configuration press `space`.

To move the attachment positions in scene configuration C press the arrow keys.

#### Interface

The user interface allows dynamic configuration of the simulation parameters including:
- Which scene configuration is currently enabled
- Number of solver iterations
- Time-step
- Gravity
- Wind Speed
- Strech Factor
- Bend Factor
- Wireframe drawing

#### Recommended Parameters

The solver iterations should not be set below 2 otherwise the solver will not converge.

- Scene Configuration A - Cloth Simulation
    - Keep time-step at 0.03
    - Adjust gravity, wind and bend factor as desired
    - Keep stretch factor between 0.5 and 1
- Scene Configuration B - Cloth Collision Detection
    - Lower the time-step to observe the collision detection in slow motion
    - Keep gravity at around 1
    - Adjust stretch and bend factor as desired
- Scene Configuration C - Two Way Interaction
    - Keep time-step at 0.03
    - Adjust gravity as desired
    - Keep wind between 0 and 0.5
    - Keep stretch between 0.5 and 1
    - Keep bend factor between 0 and 0.5
- Scene Configuration D - Rigid Bodies
    - Keep time-step at 0.03
    - Adjust gravity and wind as desired

# Mobile base with panda example

The mobile base with a panda arm is controlled in two modes: arm-driven and base-driven control. In the arm-driven approach, the base task is in the nullspace of the arm task, where commanding a arm pose will potentially move the base. In the base-driven approach, the arm task is in the nullspace of the base task, where commanding a base pose will potentially move the arm. 

When commanding the base to go to a specific location, the base-driven control is used with a joint task to hold the posture of the arm stiff.  When commanding the base to perform a task with the arm in a stationary spot, the base-driven control is used to fix the base in place, and the arm task is used to perform manipulation tasks. 

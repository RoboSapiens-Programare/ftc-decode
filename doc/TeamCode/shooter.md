# Technical Documentation: Shooter Subsystem

The `Shooter` class manages a dual-motor flywheel turret system. It includes automated target tracking based on the robot's field position, velocity control using a custom PIDF controller, and integration with the PedroPathing library for heading alignment.

---

## **1. Hardware Configuration**
The subsystem controls two high-speed motors linked to a flywheel mechanism.

| Component | Hardware Name | Direction |
| :--- | :--- | :--- |
| **Left Motor** | `"starDestroyer"` | Forward |
| **Right Motor** | `"natasha"` | Reverse |

The motors operate in `RUN_WITHOUT_ENCODER` mode because the control logic is handled by a custom `PIDFController` targeting velocity rather than position.

---

## **2. Velocity Control (PIDF)**
The shooter uses a PIDF algorithm specifically tuned for velocity maintenance. The documentation explicitly recommends the **Ziegler-Nichols Method** for tuning these constants.

* **Target Velocity:** Controlled by `targetVelocity` (ticks/second).
* **Power Distribution:** The PID output is split equally between the left and right motors (`pidOutput / 2`).
* **Feedforward (Kf):** Essential for maintaining high-speed flywheel rotation by compensating for aerodynamic drag and friction.



---

## **3. Automated Tracking & Targeting**
The subsystem calculates the necessary robot heading to face the "Obelisk" (target) based on the current alliance color.

### **Target Coordinates**
* **Red Alliance:** `(133, 135)`
* **Blue Alliance:** `(12, 135)`

### **Logic Flow**
1. **Distance Calculation:** `computeDistance()` uses the current robot Pose from the `follower` to determine the hypotenuse to the target.
2. **Angle Calculation:** `getAngle()` uses `Math.atan(dy / dx)` to calculate the precise heading required for the robot to face the target.
3. **Pathing Integration:** The `track()` method generates a short `BezierLine` and utilizes `setConstantHeadingInterpolation` to force the robot's drivetrain to face the goal while moving or idling.

---

## **4. API Reference**

### **Control Methods**
* **`track()`**: Initiates the auto-aim sequence by overriding the current drivetrain path to align the robot's heading with the goal.
* **`update()`**: The main loop. It calculates PID output if `shooting` is true and refreshes tracking paths if `isTracking` is enabled.
* **`reset()`**: Resets the tracking state, allowing for new path generation.

### **Status Methods**
* **`isShootReady()`**: Returns `true` if the flywheel velocity is within the defined tolerance (20 ticks/sec) of the target.
* **`computeDistance()`**: Returns the distance (in inches/units) to the alliance-specific goal.

---

## **5. Example Implementation**

```java
// Initialization
Shooter shooter = new Shooter(hardwareMap);

// In Teleop / Auto
if (gamepad1.right_trigger > 0.1) {
    shooter.shooting = true;
    shooter.track(); // Align robot to goal
}

// When speed is reached
if (shooter.isShootReady()) {
    // Logic to trigger the Spindexer/Feeder
}

// Crucial: Must be in the main loop
shooter.update();

# Technical Documentation: Spindexer Subsystem

The `Spindexer` class is a high-level subsystem designed to manage a 3-slot revolving storage mechanism. It handles inventory tracking by color, precise positioning using a custom PIDF controller, and automated homing using a hardware limit switch.

---

## **1. Physical Configuration**
The spindexer is logically divided into 3 equal slots. The movement is calculated based on the encoder resolution provided in the configuration.

| Constant | Default Value | Description |
| :--- | :--- | :--- |
| `ticksPerRevolution` | `8192` | The number of encoder ticks for one 360° rotation. |
| `tolerance` | `200` | The allowable error in ticks before the target is considered "reached." |
| `shootDirection` | `-1` | Defines the rotational polarity for firing sequences. |



---

## **2. Control Logic (PIDF)**
Instead of using the built-in FTC `RUN_TO_POSITION`, this subsystem utilizes a custom **PIDFController** (`Kp`, `Ki`, `Kd`, `Kf`). This allows for more granular control over acceleration and holding power.

* **Setpoint Management:** The `targetPosition` variable tracks the absolute encoder target.
* **Power Multiplier:** The final output is scaled by `uV.revolverPowerMultiplier` for global speed tuning.

---

## **3. Inventory Tracking**
The subsystem maintains a virtual map of the physical slots using the `slotColors` array.

* **`slotColors`**: An array of size 3 containing `ColorEnum` values (`GREEN`, `PURPLE`, etc.).
* **`getFreeSlot()`**: Returns the index of the first empty (`UNDEFINED`) slot.
* **`getBallCount()`**: Returns the total number of non-empty slots.
* **`shoot()`**: Clears the current slot's color data and rotates to the next slot automatically.

---

## **4. Homing & Calibration**
To ensure the slots align with the intake/outtake, the subsystem must be calibrated via the `home()` method.

1.  **Search:** The motor spins at `-0.3` power until the `limitSwitch` is pressed.
2.  **Zeroing:** The encoder is reset to `0`.
3.  **Offsetting:** The motor moves to a specific position defined by `uV.homingOffset` to align Slot 0 with the exit port.
4.  **Completion:** The `homing` flag is set to `false`, and the system enters normal PID operation.

---

## **5. Motif Sorting**
The `Spindexer` supports "Motif" logic, which aligns the revolver based on a specific color pattern (e.g., ensuring a Green ball is in the correct firing position).

* **`greenMotifPosition`**: The intended slot index for the green ball in a sequence.
* **`motifGoToStart()`**: Calculates the required rotation by finding the current `greenSlot` and applying a modulo operation to find the shortest path to the start pose.

---

## **6. API Reference**

### **Movement Methods**
* `goToSlot(int slot)`: Rotates specifically to slot 0, 1, or 2.
* `shoot()`: Advances the spindexer by 1/3 rotation and marks the slot empty.
* `isReady()`: Checks if the motor has finished moving and homing is complete.

### **Status Methods**
* `getSlotColor(int i)`: Returns the `ColorEnum` stored in a specific slot.
* `isSlotFull(int slot)`: Boolean check if a slot contains an object.
* `update()`: **Crucial.** This must be called in every loop of the OpMode to process PID calculations and homing logic.

---

## **Example Implementation**

```java
// Inside your OpMode
Spindexer spindexer = new Spindexer(hardwareMap);

// In the initialization phase
spindexer.home();

// In the run loop
spindexer.update(); // Keep the PID running

if (gamepad1.a && spindexer.isReady()) {
    spindexer.shoot();
}

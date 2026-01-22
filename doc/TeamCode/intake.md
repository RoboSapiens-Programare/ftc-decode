# Technical Documentation: Intake Subsystem

The `Intake` subsystem is responsible for the collection and identification of game elements. It integrates mechanical intake hardware with a computer vision pipeline to automate inventory management within the `Spindexer`.

---

## **1. Hardware Architecture**
The intake assembly uses a mix of high-speed rotation and sensory feedback to ensure reliable collection.

| Component | Hardware Name | Type | Note |
| :--- | :--- | :--- | :--- |
| **Intake Motor** | `"intake"` | `DcMotorEx` | Primary intake drive; reversed to pull objects in. |
| **Left Roller** | `"rollerLeft"` | `CRServo` | Continuous rotation for top-side grip. |
| **Right Roller** | `"rollerRight"` | `CRServo` | Reversed; works in tandem with the left roller. |
| **Intake Sensor** | `"intakeSensor"` | `TouchSensor` | Physical switch that detects when a ball is fully inside. |
| **Intake Cam** | `"IntakeCam"` | `WebcamName` | 320x240 camera used for real-time color analysis. |



---

## **2. Computer Vision Pipeline**
The system utilizes the `PredominantColorProcessor` (via the FTC VisionPortal) to classify game elements by color.

* **Region of Interest (ROI):** The processor is constrained to a unity center coordinate box `(-0.1, 0.1, 0.1, -0.1)` to ignore field background noise and focus only on the object passing through the intake.
* **Color Space:** Analysis is performed using **HSV** (Hue, Saturation, Value) for higher reliability under varying light conditions.
* **Stream Debugging:** The camera feed is pushed to `FtcDashboard` at 30 FPS for live calibration.

---

## **3. Logic & Classification Thresholds**
When the `intakeSensor` is triggered, the subsystem evaluates the current CV analysis.

### **Classification Rules**
| Artifact Color | Hue Range | Saturation | Action |
| :--- | :--- | :--- | :--- |
| **Green** | $70 \le H \le 95$ | $> 90$ | `spindexer.setSlotColor(slot, GREEN)` |
| **Purple** | $120 \le H \le 170$ | $> 90$ | `spindexer.setSlotColor(slot, PURPLE)` |

### **Cooldown Mechanism**
To prevent a single ball from being registered multiple times as it bounces or settles, an `ElapsedTime` cooldown of **200ms** is enforced between successful color captures.

---

## **4. API Reference**

### **`update()`**
The primary logic loop. It monitors the `intakeSensor` and `colorSensor`. If a ball is detected and the `Spindexer` is ready, it assigns the identified color to the `Spindexer's` current target slot.

### **`setPower(double power, boolean roller)`**
Controls the intake speed. 
* **Motor:** Receives full `power`.
* **Rollers:** The right roller is scaled to $37.5\%$ of the left roller's power to optimize grip/friction ratios during intake.

### **`setRollerPower(double left, double right)`**
Provides manual overrides for the CR Servos, useful for clearing jams or specialized autonomous maneuvers.

---

## **5. Example Usage**

```java
// Inside your OpMode loop
if (gamepad1.left_bumper) {
    // Intake balls with rollers active
    intake.setPower(1.0, true);
} else if (gamepad1.right_bumper) {
    // Reverse intake to clear a jam
    intake.setPower(-1.0, true);
} else {
    intake.setPower(0, false);
}

// Crucial: Must be called to process color detection and Spindexer updates
intake.update();

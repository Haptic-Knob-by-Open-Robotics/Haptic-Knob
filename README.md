# 🎛️ Haptic Knob: Feel an RLC Circuit With Your Hands

A haptic interface that lets you *physically feel* what is happening in basic electrical elements and circuits (R, L, C, RLC, and a diode) by turning a motorized knob.

The core idea is simple: we measure how you turn the knob (position, speed, acceleration), then we command the motor to push back with a torque that behaves like the voltage of a circuit element.

---

## What this project is trying to achieve

Normally, circuits are invisible. You measure them with probes and plots.

Here, you **feel** them.

- In **resistor mode**, the faster you turn, the more it pushes back.
- In **capacitor mode**, the more you “wind it up”, the stronger it wants to return.
- In **inductor mode**, sudden changes in speed get resisted (it fights acceleration).
- In **RLC mode**, you can feel damping and oscillation depending on R, L, C parameters.
- In **diode mode**, you can turn one way, but the other direction feels “blocked”.

---

## Electrical to rotational-mechanical analogy

We map circuit variables to knob variables:

| Electrical | Knob / Mechanical |
|----------:|--------------------|
| Voltage (V) | Torque (T) |
| Current (I) | Angular velocity (ω) |
| Charge (Q) | Angular displacement (θ) |
| dI/dt | Angular acceleration (α) |

This gives us a clean “translation layer” between circuit and mechanical equations into haptic behavior.

---

## How the haptic loop works

1. You turn the knob.
2. The encoder measures:
   - Angular position `θ`
   - Angular velocity `ω`
   - Angular acceleration `α`
3. We compute a target torque `T_cmd` using a selected haptic model.
4. The motor controller applies torque back to your hand.

In other words, the motor is acting like a real-time “voltage source” that outputs torque.

---

## Haptic Feedback Model Descriptions (Physics)

### Resistor Model
- **Encoder inputs:** Angular velocity `ω`
- **User parameter:** Resistance `R`
- **Haptic equation:** `T = R * ω`
- **Key behavior:**
  - At a given `R`, torque scales with turning speed.
  - At a given speed, higher `R` feels “heavier”.
- **Derivation:** Ohm’s Law `V = I * R`

---

### Capacitor Model
- **Encoder inputs:** Angular position `θ`
- **User parameter:** Capacitance `C`
- **Haptic equation:** `T = (1 / C) * θ`
- **Key behavior:**
  - The more you rotate away from zero, the more it pushes back.
  - Lower `C` feels stiffer (since `1/C` is larger).
- **Derivation:** `V = Q / C`

---

### Inductor Model
- **Encoder inputs:** Angular acceleration `α`
- **User parameter:** Inductance `L`
- **Haptic equation:** `T = L * α`
- **Key behavior:**
  - Smooth constant speed feels light.
  - Sudden speed changes feel resisted (back-EMF sensation).
- **Derivation:** `V = L * (di/dt)`

---

### RLC Model (Series)
- **Encoder inputs:** `ω`, `θ`, `α`
- **User parameters:** `R`, `L`, `C`
- **Haptic equation:** `T = R * ω + L * α + θ / C`
- **Key behavior:**
  - **Underdamped (low R):** you feel oscillation, like energy sloshing between L and C.
  - **Critically damped / overdamped (higher R):** oscillations fade fast or disappear.
- **Derivation idea:** `V_out = R i + L di/dt + Vc`, converted to discrete-time equivalents.

---

### Diode Model
- **Encoder inputs:** Angular velocity direction (CW vs CCW)
- **User parameter:** `Vd = 0.7 V` (typical diode drop)
- **Haptic behavior:**
  - Turning **CW** (assume CW is positive current direction): constant feedback torque corresponding to `Vd`.
  - Turning **CCW**: “infinite” opposing torque (practically implemented as a very large torque limit), so the knob feels blocked.
- **Note:** In real hardware we clamp torque for safety, but the intention is “one-way motion”.

---

## What does it feel like to turn the knob, and why?

### Resistor
A resistor resists current flow. Since current maps to angular velocity, turning faster increases “current”, so the knob pushes back harder.  
Higher resistance means more pushback for the same turning speed.

### Capacitor
A capacitor stores charge. Since charge maps to angular displacement, the further you rotate, the more “charge” you build up, and the stronger the restoring torque gets (`V = Q/C`).  
If you let go, it wants to spring back toward the original position, like twisting a spring.

### Inductor
An inductor resists changes in current. Since current maps to angular velocity, `di/dt` maps to angular acceleration.  
So it mainly fights *how quickly* you change speed. Sudden acceleration feels like you hit invisible inertia.

### LC Series Circuit
An LC circuit behaves like a pendulum: energy swaps between electric storage (C) and magnetic storage (L).  
That exchange shows up as oscillation in the knob’s torque and motion.

Equation form:
`V = L di/dt + Q/C`  
So torque depends on both acceleration (L term) and displacement (C term).

### RLC Series Circuit
The resistor acts like a damper. You still get oscillation in some settings, but it shrinks over time.  
- **Underdamped:** keeps oscillating but fades.
- **Critically damped:** returns quickly with minimal/no overshoot.
- **Overdamped:** returns slowly, no oscillation.

### Diode
A diode enforces one direction of current. Since direction maps to rotation direction, one direction feels allowed, the other feels blocked.  
The “0.7 V” drop becomes a constant “threshold-like” torque sensation in the forward direction.

---

## Parameters you can tune

These are the knobs behind the knob.

| Mode | Parameters | What it changes in your hand |
|------|------------|------------------------------|
| R | `R` | Heaviness vs speed |
| C | `C` | Spring stiffness vs displacement |
| L | `L` | “Inertia” vs acceleration changes |
| RLC | `R, L, C` | Damping and oscillation character |
| Diode | `Vd`, torque limit | Forward threshold and reverse blocking strength |

---

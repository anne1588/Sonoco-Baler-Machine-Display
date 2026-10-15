Baler pressure-controlled cycle demonstration (LVGL v8.3, 320x480 portrait)

- Single Arduino sketch; add your EXISTING working display/touch/tick setup in setup().
- ONE current process panel plus pressure, stroke and hopper readouts.
- START CONVEYOR: simulate hopper filling (2.5s), then auto-stop.
- Fixed compression stroke limit: strokeLimit=75%.
- PLC-calculated pressure is REPRESENTED by a simulated rise of 20 units per stroke;
  pressureLimit=80. Replace simulated pressure with confirmed PLC data later.
- After reaching pressure setpoint, plunger MUST retract to 0/home before
  the door-open limit-switch signal can advance the sequence.
- DOOR OPEN LIMIT SWITCH (SIMULATE): becomes enabled at plunger home.
- While the door remains open, START initiates more material feed, a fixed
  compression stroke, and retraction. This can repeat as often as needed.
- DOOR CLOSED LIMIT SWITCH (SIMULATE): available ONLY between strokes while
  plunger is home; signals completed discharge and resets for a new bale.
- There is NO bale-exit sensor and NO bale-exit confirmation button.
- No actual motors, valves, conveyor or safety circuits are controlled here.

IMPORTANT: The operator decides when discharge is complete and physically
closes the door. The closed limit switch verifies only door closure, not that
the bale exited. Real controller logic needs pressure, plunger-home, stroke
limit, hopper-high and both door limit switches, with independent safety
interlocks and safeguarding. Display messages are informational only.

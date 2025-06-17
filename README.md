# Pre-released Control Pragram

![1750137852319](images/README/1750137852319.png)

JPCM_node.cpp completes suction force compensated model predictive control based on remote controller or trajectory command (Wall_in_proximity_run.cpp).

All factors, especially the dynamic factors of suction compensation, are realized in GTSAMIntegration. 

## Supporting:

### Estimator:

#### (1) FGO-based FakeGPS + IMU (pose, velocity, bias, gravity rotation)

### Controller:

#### (1) Differential-Flatness-Based Control (PID)

#### (2) FGO-based MPC

#### (3) Uncerntainty-aware MPC

#### (4) Joined Positioning and Control Model (testing)

### Dependence:

Gtsam-4.0.3

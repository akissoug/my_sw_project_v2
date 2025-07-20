# SEAWINGS Simulation Guide

## Installation

### 1. Install QGroundControl
```bash
# Download QGroundControl
cd ~/Downloads
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x QGroundControl.AppImage
sudo mv QGroundControl.AppImage /usr/local/bin/

```

### 2. Install micro XRCE-DDS Agent (if not already installed)
```bash
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```


## Running Simulation - Method 1: Using Launch File

```bash
# Terminal 1: Launch everything
cd ~/sw_ws
source install/setup.bash
ros2 launch seawings_mission_management seawings_mission.launch.py

# This will open multiple terminals automatically:
# - XRCE-DDS Agent
# - PX4 SITL with Gazebo
# - PowerMonitor node
# - FaultDetector node
# - MissionSupervisor node

# Terminal 2: Start QGroundControl
QGroundControl.AppImage
```

## Running Simulation - Method 2: Manual (Separate Terminals)

### Terminal 1: micro XRCE-DDS Agent
```bash
MicroXRCEAgent udp4 -p 8888
```

### Terminal 2: PX4 SITL
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo

# Wait for these messages:
# INFO  [init] PX4_SIM_HOSTNAME: localhost
# INFO  [simulator] Waiting for simulator to accept connection on TCP port 4560
# INFO  [simulator] Simulator connected on TCP port 4560.
# INFO  [commander] Ready for takeoff!
```

### Terminal 3: QGroundControl
```bash
QGroundControl.AppImage

# QGC will automatically connect to PX4
# You should see:
# - Green "Ready To Fly" status
# - GPS lock (simulated)
# - Battery status
# - Telemetry data
```

### Terminal 4: PowerMonitor
```bash
cd ~/sw_ws
source install/setup.bash
ros2 run seawings_mission_management power_monitor
```

### Terminal 5: FaultDetector  
```bash
cd ~/sw_ws
source install/setup.bash
ros2 run seawings_mission_management fault_detector
```

### Terminal 6: MissionSupervisor
```bash
cd ~/sw_ws
source install/setup.bash
ros2 run seawings_mission_management mission_supervisor
```

## QGroundControl Basic Operations

### 1. Initial Setup
- **Wait for connection**: Green status bar saying "Ready To Fly"
- **Check telemetry**: Battery, GPS, altitude should show data
- **Safety check**: Ensure area is clear in simulation

### 2. Creating a Mission
1. Click "Plan" tab (left side)
2. Click on map to add waypoints
3. Set altitude for each waypoint
4. Add "Return to Launch" at end
5. Click "Upload" button

### 3. Arming the Vehicle
Arm the vehicle from px4 terminal by hitting:
commander arm
1. Go back to "Fly" tab
2. Arm the vehicle from px4 terminal by hitting:
commander arm
Now the drone should start the mission.


### 6. Testing Nodes

#### Test Battery Monitor (Terminal 7):
```bash
# Monitor battery status
ros2 topic echo /fmu/out/battery_status

# Watch PowerMonitor logs - it will trigger RTL at 25% battery
```

#### Test Fault Detection:
```bash
# In PX4 console (Terminal 2), inject GPS failure:
failure gps off

# Watch FaultDetector trigger emergency landing
```

#### Test Mission Supervisor:
```bash
# Monitor mission status
ros2 topic echo /seawings/mission_status

# Should show: DISARMED -> ARMED -> TAKEOFF -> MISSION -> RTL -> LANDING
```

## Monitoring Everything

### Terminal 7: Monitor All Topics
```bash
# List all topics
ros2 topic list

# Monitor specific topics
ros2 topic echo /fmu/out/vehicle_status
ros2 topic echo /fmu/out/battery_status  
ros2 topic echo /fmu/out/vehicle_gps_position
ros2 topic echo /seawings/mission_status
```


## Testing Scenarios

### Scenario 1: Normal Mission
1. Arm in QGC
2. Upload simple square mission (4 waypoints)
3. Execute mission
4. Watch nodes monitor the mission
5. Should complete and RTL normally

### Scenario 2: Low Battery RTL
1. Start mission
2. In Terminal 2 (PX4), simulate battery drain:
   ```
   param set BAT1_V_EMPTY 3.5
   param set BAT1_V_CHARGED 4.0
   ```
3. PowerMonitor should trigger RTL

### Scenario 3: GPS Failure
1. During mission, in PX4 console:
   ```
   failure gps off
   ```
2. FaultDetector should trigger emergency landing

### Scenario 4: Mission Timeout
1. Set short timeout in config
2. Start long mission
3. MissionSupervisor should trigger RTL

### Extras:

In the PX4 terminal, run these commands: (also modify battery thresholds in QGC)

```bash
#### Enable battery simulation
param set SIM_BAT_ENABLE 1

#### Set battery drain rate (current in Amps)
param set SIM_BAT_DRAIN 15.0    # 15A current draw

#### Optional: Set battery parameters
param set BAT1_CAPACITY 5000    # 5000 mAh battery
param set BAT1_N_CELLS 4        # 4S battery
param set BAT1_V_CHARGED 4.2    # Voltage per cell when full
param set BAT1_V_EMPTY 3.5      # Voltage per cell when empty

#### Set thresholds
param set BAT1_LOW_THR 0.3      # Low battery at 30%
param set BAT1_CRIT_THR 0.2     # Critical at 20%
param set BAT1_EMER_THR 0.1     # Emergency at 10%

#### Save parameters
param save
```


## Quick Reference Commands

```bash
# Launch everything
ros2 launch seawings_mission_management seawings_mission.launch.py

# Monitor all
ros2 topic echo /seawings/mission_status

# Emergency stop (in PX4 console)
commander disarm -f

```

# ARS408_ros
## Radar command
```bash
# Setup (can_setup.sh)
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# Show
candump can0

# Send
cansend can0 <id>#<msg>

# Example: Object
cansend can0 200#0800000008000000
```

# ros_colorsens_apds9960
A ROS 2 driver for the adafruit apds9960 color sensor.

## Running the Node:

Using the CLI: </br>
`ros2 run ros_colorsens_apds9960 apds9960` </br>

Python launch file:
```python
color_sensor = Node(
    package="ros_colorsens_9960",
    executable='apds9960',
    screen='on'
)
```
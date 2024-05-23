"""
Use this controller to fly the drone in the Webots simulator using the 
barrier function controller. It is a simple example of how to use the
crazyflie's motors and sensors in Webots.
It is limited to a fixed height and a fixed forward velocity.
"""

from helper_functions.flight_controller import FlightController

if __name__ == '__main__':
    flight_controller = FlightController()
    flight_controller.run()
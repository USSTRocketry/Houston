# Houston
Avionics Ground station

## Install Instructions
1. You need Python3, so install that if you dont have it,<br> verify you have it by typing `python3 --version` in the terminal<br>(Ctrl + X then select Terminal (Admin))
2. You are going to need a few more libraries for this to run properly,<br> specifically Proto, Numpy, Protobuf, PyOpenGL and PyQt6.
3. Install them all using:<br>**pip install proto numpy protobuf pyopengl pyqt6 pyqtgraph**

**PLEASE NOTE:** display_main.py is currently adjusted to listen to the mock_telemetry_server.py. Adjust the incoming IP/port on display_main.py before running so it connects to the proper ESP32 or whatever you are using


### Documentation and tutorials

1. https://www.pythonguis.com/pyqt6-tutorial/
2. https://stackabuse.com/brief-introduction-to-opengl-in-python-with-pyopengl/
3. https://numpy.org/learn/

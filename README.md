# Aircraft-Design
This repository contains a wing and propulsion system sizing program, which generates a matching diagram for certain aircraft parameters, requirements, and constraints, which can be edited by the user either directly in the code, or on a slider in a streamlit applet.

In order to use the program download the run the program, download the MDapp.py file and type "streamlit run MDapp.py" in the terminal.

To customise for different aircraft, or for iterative design changes, individual parameters or requirements can be changed in the "AircraftData" dataclass or select parameters can be changed in the interactive app. note there is limited precision in the app

Dependencies:
numpy
matplotlib.pyplot
dataclasses
streamlit

Future development:
1. restructure "Aerodata" to include nested dataclasses, for example for different flap/landing gear configurations
2. develop and an algorithm for a class 1 weight estimation, and/or drag polar estimation which can be integrated into the existing program (values such as zero lift drag or oswald factor can be fed straight into the wing sizing algorithm). 
3. develop seperate but integrated algorithms for further more detailed wing design, such as wing planform or airfoil design, perhaps through training a surrogate neural network on airfoil data, and performing a search space optimisation on a range of parameters to get an "optimal" starting point for a wing design

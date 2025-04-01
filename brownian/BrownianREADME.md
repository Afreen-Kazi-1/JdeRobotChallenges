# Brownian Motion Robot Simulation

## Overview
This project simulates Brownian Motion using a simple robot model in a bounded arena. The robot moves in random directions, bouncing off walls upon collision. The simulation is visualized using **Matplotlib**.

## Features
- Simulates Brownian motion in a 2D bounded space.
- Uses **Matplotlib** for visualization.
- Implements collision handling with random angle adjustments.
- Modular Python design for reuse and expansion.

## Installation
Ensure you have Python **3.x** installed. Then, install the required dependencies:
```bash
pip install numpy matplotlib
```

## Usage
Run the simulation using the following command:
```bash
python app.py
```
### **Structure**
- `brownianmotion.py` → Main module containing the BrownianRobot class.
- `app.py` → A script demonstrating the running of the module.
- `BrownianREADME.md` → Project documentation.

## Demo
A sample GIF or video demonstrating the simulation can be found in the `output/` directory.

## Contributing
1. Fork the repository.
2. Clone your fork:
   ```bash
   git clone https://github.com/yourusername/brownian-motion-robot.git
   ```
3. Create a new branch:
   ```bash
   git checkout -b feature-name
   ```
4. Commit your changes and push:
   ```bash
   git add .
   git commit -m "Add feature-name"
   git push origin feature-name
   ```
5. Open a Pull Request.

## Contact
For any questions or suggestions, feel free to open an issue.


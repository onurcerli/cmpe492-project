# Generation of Organic Simulation Environment for Robot Testing

This project generates organic simulation environments for robot navigation testing in MuJoCo. Instead of using traditional grid-based maps, it creates organic layouts composed of irregular cells, producing more diverse environments.

## Getting Started

1. Clone the repository:

   ```
   git clone https://github.com/onurcerli/cmpe492-project
   ```

2. Change directory:

   ```
   cd cmpe492-project
   ```

3. Create a virtual environment:

   ```
   python -m venv env
   ```

4. Activate the virtual environment:

   - Linux/macOS
     ```
     source env/bin/activate
     ```
   - Windows
     ```
     .\env\Scripts\activate
     ```
     -

5. Install the required packages:

   ```
   pip install -r requirements.txt
   ```

6. Running the Project
   - Run the code to generate the simulation environment:
     ```
     python project.py
     ```
   - Now you can observe the 2d version of the environment on the opening matplotlib window and also you can explore the 3d environment by launching mujoco viewer with the output file:
     ```
     python -m mujoco.viewer --mjcf=mujoco_simulation_environment.xml
     ```

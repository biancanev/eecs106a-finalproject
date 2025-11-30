# Installing Dependencies for Standalone Simulation

## Quick Install

```bash
pip3 install numpy matplotlib cvxpy
```

Or if using a virtual environment:

```bash
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install numpy matplotlib cvxpy
```

## Detailed Installation

### Option 1: Using pip (Recommended)

```bash
pip3 install numpy matplotlib cvxpy osqp
```

### Option 2: Using apt (Ubuntu/Debian)

```bash
sudo apt-get update
sudo apt-get install python3-numpy python3-matplotlib python3-cvxpy
```

### Option 3: Using conda

```bash
conda install numpy matplotlib
pip install cvxpy osqp
```

## Verify Installation

```bash
python3 -c "import numpy; import matplotlib; import cvxpy; print('All dependencies OK!')"
```

## Running After Installation

```bash
cd turtlebot_interceptor
python3 run_standalone_sim.py
```

## Troubleshooting

### "No module named 'numpy'"
- Install: `pip3 install numpy`
- Or: `sudo apt-get install python3-numpy`

### "No module named 'cvxpy'"
- Install: `pip3 install cvxpy osqp`
- Note: cvxpy requires a solver (OSQP is recommended)

### "No module named 'matplotlib'"
- Install: `pip3 install matplotlib`
- Or: `sudo apt-get install python3-matplotlib`

### Import Errors
- Make sure you're using Python 3: `python3 --version`
- Check if packages are installed: `pip3 list | grep numpy`


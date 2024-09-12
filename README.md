# Py Bridge Designer
A Python implementation of the [Bridge Designer 2016](https://bridgedesigner.org/) engineering simulator by Stephen J. Ressler and Gene K. Ressler.

Implementation reference was the C code created for a Ruby plugin for the Bridge Designer and Contest Rails Code - availible here: https://sourceforge.net/p/wpbdc/rails/ci/master/tree/

## IEEE Conference Paper
This code is part of the published conference paper, [Designing Architecture in a Bridge Engineering Simulation](https://ieeexplore.ieee.org/document/10639984) presented at the [2024 IEEE 7th International Conference on Industrial Cyber-Physical Systems (ICPS)](https://ieeexplore.ieee.org/xpl/conhome/10639954/proceeding)

### Requirements
`pip install -r requirements.txt`
* note: developed typing annotations requiring >= Python 3.6

### Gym Env Wrapper
Can be used with or without the bridge_env.py file.

If you wish to use or modify the bridge_env.py for Reinforcement Learning as a [Gym](https://gymnasium.farama.org/) enviroment, then make sure to install the gymnasium requirements:
`pip install gymnasium`

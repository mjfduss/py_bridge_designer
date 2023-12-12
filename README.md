# Py Bridge Designer
A Python implementation of the [Bridge Designer 2016](https://bridgedesigner.org/) engineering simulator by Stephen J. Ressler and Gene K. Ressler.

Implementation reference was the C code created for a Ruby plugin for the Bridge Designer and Contest Rails Code - availible here: https://sourceforge.net/p/wpbdc/rails/ci/master/tree/

### Requirements
`pip install -r requirements.txt`
* note: developed typing anotations requiring >= Python 3.6

### Gym Env Wrapper
If you wish to use this for Reinforcement Learning as a [Gym](https://gymnasium.farama.org/) enviroment, then make sure to install the gymnasium requirements:
`pip install gymnasium`
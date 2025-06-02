#!/usr/bin/env python3
from ray import rllib
print("RLlib available:", hasattr(rllib, 'agents'))

from __future__ import print_function
from sdfbuilder.math import Vector3
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')

from revolve.build import util
util.size_scale_factor = 10

from revolve.convert.yaml import yaml_to_robot
from tol.spec import body_spec, brain_spec
from tol.config import Config
from tol.build import get_builder, get_simulation_robot

bot_yaml = '''
---
body:
  id          : Core
  type        : ActiveCardan
'''

conf = Config()
bot = yaml_to_robot(body_spec, brain_spec, bot_yaml)
builder = get_builder(conf)
sdf = get_simulation_robot(bot, "test_bot", builder, conf)
sdf.elements[0].translate(Vector3(0, 0, 0.5))
print(str(sdf))
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
__author__ = 'Niklas Kroeger'
__email__ = "niklas.kroeger@imr.uni-hannover.de"
__status__ = "Development"

import warnings

warnings.simplefilter("always")
warnings.warn("Due to the migration to the pypylon library provided by basler, "
              "the classes from this file have been moved to "
              "pyCameras.cameraBasler\n"
              "Pleas adjust your import accordingly. This file will be deleted "
              "shortly!",
              DeprecationWarning)

from pyCameras.cameraBasler import *

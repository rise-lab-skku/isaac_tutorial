import os
import toml

##
# Register Gym environments.
##

#from .utils import import_packages
from omni.isaac.lab_tasks.utils import import_packages

# The blacklist is used to prevent importing configs from sub-packages
_BLACKLIST_PKGS = ["utils"]
# Import all configs in this package
import_packages(__name__, _BLACKLIST_PKGS)

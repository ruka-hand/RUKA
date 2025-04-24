import os
from importlib import import_module


def get_repo_root():
    """Returns the absolute path to the root directory of the RUKA repository.

    Returns:
        str: Absolute path to the repository root directory
    """
    # Get the directory containing this file
    current_dir = os.path.dirname(os.path.abspath(__file__))

    # Go up 2 levels from ruka_hand/utils to reach repo root
    repo_root = os.path.dirname(os.path.dirname(current_dir))

    return repo_root


def load_function(func_name: str):
    """Load a function dynamically from a module."""
    module_name, function_name = func_name.rsplit(".", 1)
    module = import_module(module_name)
    return getattr(module, function_name)


if __name__ == "__main__":
    print(get_repo_root())

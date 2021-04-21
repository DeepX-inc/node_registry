#!/usr/bin/env python3

from .xnode_builder import node_init


rosnode = node_init()


def register(globals_params: dict = None):
    """
    Spins the rosnode on the executor.

    Args:
    ----
    globals_params (dict): Dictonary of module globals()

    """
    if globals_params is not None:
        assert isinstance(
            globals_params, dict
        ), f'Expected {dict} but received {type(globals_params)}'

        __keys__: list = ['__name__', 'rosnode']
        for key in __keys__:
            if key not in globals_params.keys():
                raise KeyError(f'Key {key} is required')

        name = globals_params['__name__']
        if name not in ['__main__']:
            print(
                '\033[33m__main__ not found in the globals\033[0m'
            )
            return

    if rosnode.node is None:
        raise RuntimeError('Please initialize the node')
    rosnode.spin()


def register_node(func_or_dict):

    def _register(func):
        assert callable(func)
        func()
        register(globals_params)
        return func

    if isinstance(func_or_dict, dict):
        globals_params = func_or_dict
        return _register
    else:
        globals_params = None
        return _register(func_or_dict)

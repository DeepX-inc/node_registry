#!/usr/bin/env python3
# Copyright (c) 2021, DeepX-inc
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# @author Krishneel Chaudhary
"""Python Xnode Decorators."""
from .xnode_builder import node_init

rosnode = node_init()


def register(globals_params: dict = None):
    """
    Spin the rosnode on the executor.

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
    """Registor Node in XNode."""
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

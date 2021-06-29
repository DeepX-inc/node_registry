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

import os

import rclpy
from rclpy.node import Node as N


class XNode(N):

    def __init__(self, node_name: str, **kwargs: dict):
        """
        Create a XNode.

        Args:
            node_name (str): A name to give to this node.
        """
        super(XNode, self).__init__(
            node_name,
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
            **kwargs
        )

        self._name = f'{self.get_namespace()}/{self.get_name()}'
        self.logger.info(
            '\n\033[32m' + '{s:{c}^{n}}'.format(s='', n=80, c='-') +
            f'\nNode started >>> {self._name}' +
            '\n{s:{c}^{n}}'.format(s='', n=80, c='-') + '\033[0m'
        )

        if os.environ.get('USE_SIM_TIME', '').lower() in ['true', '1']:
            self.logger.info(f'Node {self.get_name()} is using SIM_TIME')
            param = rclpy.parameter.Parameter(
                'use_sim_time',
                rclpy.Parameter.Type.BOOL,
                True,
            )
            self.set_parameters([param])

    @property
    def logger(self):
        return self.get_logger()

    def get_parameter_value(self, name: str, default_value: object):
        """
        Get a parameter by name or set it with default value if not exists.

        Args:
            name (str): Fully-qualified name of the parameter, including its
            namespace.
            default_value (object): A default value for the parameter.

        Returns
        -------
        The parameter value.

        """
        param = self.get_parameter(name)
        if param.type_ == param.Type.NOT_SET:
            param = self.declare_parameter(name, default_value)
            self.logger.warn(
                f'Could not get param {name} setting default {default_value}'
            )
        return param.value

    def __del__(self):
        print(
            f'\033[34mNode shutdown <<< {self._name} \033[0m'
        )
        try:
            self.destroy_node()
        except rclpy.handle.InvalidHandle:
            pass

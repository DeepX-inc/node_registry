#!/usr/bin/env python3

import os

import rclpy
from rclpy.node import Node as N


class XNode(N):

    def __init__(self, node_name: str, **kwargs: dict):
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

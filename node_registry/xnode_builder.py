#!/usr/bin/env python3
"""
Authors: Krishneel Chaudhary, Dmitry Neverov, Rachel Chan, Lu Chenyue.

DeepX, Inc.
"""

from dataclasses import dataclass
from threading import Lock

from rcl_interfaces.srv import GetParameters

import rclpy
from rclpy.exceptions import (
    ParameterAlreadyDeclaredException,
    ParameterNotDeclaredException,
)

from .xnode import XNode


@dataclass
class XNodeBuilder(object):

    _REGISTRY = {}
    _PUB_REGISTRY = {}
    _node_name: str = None
    _lock = Lock()

    def __call__(self, argument: str):
        if callable(argument):
            func = argument
            self.__create_node(func, func())
            return func
        else:
            return self.register_node(node_name=argument)

    def __create_node(self, func, node_name: str):
        self._node_name = node_name
        node = XNode(node_name)
        self._REGISTRY[node_name] = node

    def register_node(self, node_name: str):
        assert isinstance(node_name, str)

        def _register(func_or_cls):
            self._node_name = node_name
            node = XNode(node_name)
            self._REGISTRY[node_name] = node
            return func_or_cls

        return _register

    def subscribe(self, msg_type, topic_name: str, qos_profile: int = 1):
        assert isinstance(topic_name, str)
        assert callable(msg_type)
        assert self._node_name is not None

        def _register(func):
            assert callable(func)
            assert isinstance(self.node, XNode)
            sub = self.node.create_subscription(
                msg_type, topic_name, func, qos_profile
            )
            assert sub
            return func

        return _register

    def approx_time_sync(
        self, arguments: list, queue: int = 5, slop: float = 0.1
    ):
        assert isinstance(arguments, (list, tuple))

        def _register(func):
            assert callable(func)
            from message_filters import ApproximateTimeSynchronizer
            from message_filters import Subscriber

            subscribers = [
                Subscriber(self.node, msg_type, topic_name)
                for msg_type, topic_name in arguments
            ]
            ats = ApproximateTimeSynchronizer(subscribers, queue, slop)
            ats.registerCallback(func)
            return func

        return _register

    def service(self, srv_type, srv_name: str):
        assert isinstance(srv_name, str)
        assert callable(srv_type)

        def _register(func):
            assert callable(func)
            assert isinstance(self.node, XNode)
            # TODO(Krishneel): register multiple srvs
            srv = self.node.create_service(srv_type, srv_name, func)
            assert srv
            return func

        return _register

    def client(self, srv_type, srv_name: str, timeout: float = 5.0):
        assert isinstance(srv_name, str)
        assert callable(srv_type)

        def _register(func):
            assert callable(func)
            # TODO(Krishneel): register multiple clients
            self.client = self.node.create_client(srv_type, srv_name)
            while not self.client.wait_for_service(timeout_sec=timeout):
                self.logger.warn(
                    f'service {srv_name} not available, waiting again...'
                )
            return func

        return _register

    def publisher(self, msg_type, topic_name: str, qos_profile: int = 1):
        assert isinstance(topic_name, str)
        assert callable(msg_type)

        def _register(func):
            assert isinstance(self.node, XNode)
            pub = self.node.create_publisher(msg_type, topic_name, qos_profile)
            assert pub
            self._PUB_REGISTRY[topic_name] = pub

        return _register

    def connection_based(self, pub_id):
        if type(pub_id) == str:
            pub_ids = [pub_id]
        else:
            assert isinstance(pub_id, (list, tuple))
            pub_ids = pub_id

        def _register(func):
            assert callable(func)

            for pub_id in pub_ids:
                pub = self.get_publisher(pub_id=pub_id)
                if pub.get_subscription_count() > 0:
                    with self._lock:
                        ret_val = func()
                        if ret_val is not None:
                            assert isinstance(ret_val, dict)
                            for key, item in ret_val.items():
                                if item is not None:
                                    self.get_publisher(key).publish(item)
                    break

            return func

        return _register

    def parameter(self, param_name: str, value: object):
        assert isinstance(param_name, str)

        def _register(func):
            try:
                self.__dict__[param_name] = self.node.get_parameter_value(
                    param_name, value
                )
            except ParameterAlreadyDeclaredException as e:
                self.logger.error(f'{e}')
                raise e
            return func

        return _register

    def parameter_list(self, parameters: list):
        assert len(parameters) > 0, 'Cannot create parameter with empty list'

        def _register(func):
            try:
                for param_name, value in parameters:
                    self.__dict__[param_name] = self.node.get_parameter_value(
                        param_name, value
                    )
            except ParameterAlreadyDeclaredException as e:
                self.logger.error(f'{e}')
                raise e
            return func

        return _register

    def timer(self, timer_period: float):
        assert isinstance(timer_period, (int, float))

        def _register(func):
            assert callable(func)
            assert self.node.create_timer(timer_period, func)
            return func

        return _register

    def inject(self, args):
        def update_vars(func, key: str = None):
            ret_val = func()
            if isinstance(ret_val, dict):
                self.__dict__ = {**self.__dict__, **ret_val}
            else:
                key = func.__name__ if key is None else key
                self.__dict__[key] = ret_val

        if callable(args):
            update_vars(args, args.__name__)
            return args
        else:
            assert isinstance(args, str)

            def _wrapper(func):
                update_vars(func, args)
                return func

            return _wrapper

    def timeit(self, func, once: bool = False, throttle: int = None):

        # TODO: Change to ROS time
        import time

        def _wrapper(*args, **kwargs):
            start = time.time()
            result = func(*args, **kwargs)
            self.logger.info(
                f'{func.__name__} Processing time: {time.time() - start}',
                once=once,
                throttle_duration_sec=throttle,
            )
            return result

        return _wrapper

    def spin(self):
        try:
            while rclpy.ok():
                rclpy.spin(self.node)
        except KeyboardInterrupt:
            rclpy.shutdown()

    def spin_until_future_complete(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        return future

    def get_publisher(self, pub_id: str):
        return self._PUB_REGISTRY.get(pub_id, None)

    def get_parameter(self, name: str):
        try:
            return self.node.get_parameter(name)
        except ParameterNotDeclaredException as e:
            self.logger.warn(f'Parameter {name} not found: {e}')

    def call_get_parameters(
        self,
        node_name: str,
        parameter_names: list,
        timeout_sec: float = 5.0,
    ):
        client = self.node.create_client(
            GetParameters, f'{node_name}/get_parameters'
        )

        ready = client.wait_for_service(timeout_sec=timeout_sec)
        if not ready:
            raise RuntimeError(
                f'Wait for service timed out for {node_name}/get_parameters'
            )

        request = GetParameters.Request()
        request.names = parameter_names
        future = self.spin_until_future_complete(client=client, request=request)

        response = future.result()
        if response is None:
            e = future.exception()
            raise RuntimeError(
                f"Exception while calling service of '{node_name}': {e}"
            )
        return response

    @property
    def use_sim_time(self):
        try:
            return self.get_parameter('use_sim_time').value
        except AttributeError:
            pass
        return False

    @property
    def logger(self):
        return self.node.get_logger()

    @property
    def clock_now(self):
        return self.node.get_clock().now()

    @property
    def node(self):
        return self._REGISTRY.get(self._node_name, None)

    """
    def __del__(self):
        try:
            self.node.destroy_node()
        except AttributeError:
            pass"""


def node_init(args=None):
    rclpy.init(args=args)
    return XNodeBuilder()

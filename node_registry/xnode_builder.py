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
"""Xnode Builder."""
from dataclasses import dataclass
from threading import Lock

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.exceptions import (ParameterAlreadyDeclaredException,
                              ParameterNotDeclaredException)

from .xnode import XNode


@dataclass
class XNodeBuilder(object):
    """Xnode Builder Class."""

    _REGISTRY = {}
    _PUB_REGISTRY = {}
    _node_name: str = None
    _lock = Lock()

    def __call__(self, argument: str):
        """Xnode Call."""
        if callable(argument):
            func = argument
            self.__create_node(func, func())
            return func
        else:
            return self.register_node(node_name=argument)

    def __create_node(self, func, node_name: str):
        """
        Create a New ROS node.

        Args:
            func ([Callable]):
            node_name (str): The name of the Ros Node.
        """
        self._node_name = node_name
        node = XNode(node_name)
        self._REGISTRY[node_name] = node

    def register_node(self, node_name: str):
        """
        Registrer a new node in the node_registry.

        Args:
            node_name (str): The name of the node.
        """
        assert isinstance(node_name, str)

        def _register(func_or_cls):
            self._node_name = node_name
            node = XNode(node_name)
            self._REGISTRY[node_name] = node
            return func_or_cls

        return _register

    def subscribe(self, msg_type, topic_name: str, qos_profile: int = 1):
        """
        Create a new subscription.

        Args:
            msg_type : The type of ROS messages the subscription
                     will subscribe to.
            topic_name (str): The name of the topic the subscription
                     will subscribe to.
            qos_profile (int, optional): A QoSProfile or a history depth
                to apply to the subscription.
                In the case that a history depth is provided, the QoS history
                is set to RMW_QOS_POLICY_HISTORY_KEEP_LAST, the QoS history
                depth is set to the value of the parameter, and all other
                QoS settings are set to their default values.
                Defaults to 1.
        """
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
        """
        Approximately synchronizes messages by their timestamps.

        Args:
            arguments (list): List of (msg_type, topic_name) of
                                different Subscriber.
            queue (int, optional): Size of the queue. Defaults to 5.
            slop (float, optional): It defines the delay (in seconds)
                                    with which messages can be synchronized.
                                    Defaults to 0.1.
        """
        assert isinstance(arguments, (list, tuple))

        def _register(func):
            assert callable(func)
            from message_filters import ApproximateTimeSynchronizer, Subscriber

            subscribers = [
                Subscriber(self.node, msg_type, topic_name)
                for msg_type, topic_name in arguments
            ]
            ats = ApproximateTimeSynchronizer(subscribers, queue, slop)
            ats.registerCallback(func)
            return func

        return _register

    def service(self, srv_type, srv_name: str):
        """
        Create a new service server.

        Args:
            srv_type : The service type.
            srv_name (str): The name of the service.

        Returns
        -------
            [Callable]: Return the callback function.

        """
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
        """
        Create a new service client.

        Args:
            srv_type : The service type.
            srv_name (str): The name of the service.
            timeout (float, optional): Seconds to wait. If ``None``, then
            wait forever.
            Defaults to 5.0.
        """
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
        """
        Create a new Publisher.

        Args:
            msg_type ([type]): The type of ROS messages the publisher will
                                publish.
            topic_name (str): The name of the topic the publisher will
                                publish to.
            qos_profile (int, optional): A QoSProfile or a history depth
                                        to apply to the publisher.
                                        Defaults to 1.
        """
        assert isinstance(topic_name, str)
        assert callable(msg_type)

        def _register(func):
            assert isinstance(self.node, XNode)
            pub = self.node.create_publisher(msg_type, topic_name, qos_profile)
            assert pub
            self._PUB_REGISTRY[topic_name] = pub

        return _register

    def connection_based(self, pub_id):
        """
        Call a function only if there are active subscribers for.

        the given publish topic.

        Args:
            pub_id : A list or tuple of publishers' IDS.
        """
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
        """
        Create the parameter specified in the argument.

        Args:
            param_name (str): The name of the parameter
            value (object): The value that need to be assigned to the
                            given parameter.

        Raises
        ------
            e: ParameterAlreadyDeclaredException if the parameter
             is already declared.

        """
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
        """
        Create a list of parameters.

        Args:
            parameters (list): List of name & value pair.

        Raises
        ------
            e: ParameterAlreadyDeclaredException if the parameter
             is already declared.

        """
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
        """
        Create a new timer.

        Args:
            timer_period (float): The period in second of the timer.

        Returns
        -------
            callback_group: The callback group for the timer.

        """
        assert isinstance(timer_period, (int, float))

        def _register(func):
            assert callable(func)
            assert self.node.create_timer(timer_period, func)
            return func

        return _register

    def inject(self, args):
        """
        Inject a function into the Xnode class.

        Args:
            args: Function which needs to be injected.
        """
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
        """
        Calculate the Processing time of the given function.

        Args:
            func : The function whose processing time is to be calculated
            once (bool, optional): [description]. Defaults to False.
            throttle (int, optional): [description]. Defaults to None.
        """
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
        """
        Execute work and block until the context associated with the executor.

        is shutdown.

        Args:
            None.
        """
        try:
            while rclpy.ok():
                rclpy.spin(self.node)
        except KeyboardInterrupt:
            rclpy.shutdown()

    def spin_until_future_complete(self, client, request):
        """
        Execute work until the future is complete.

        Args:
            client : Service Client.
            request : The service request.

        Returns
        -------
            Future [Object]: The future object to wait on.

        """
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        return future

    def get_publisher(self, pub_id: str):
        """
        Retrieve a publisher.

        Args:
            pub_id (str): The publisher id.

        Returns
        -------
            [Object]: return the publisher_object corresponding to the pub_id.

        """
        return self._PUB_REGISTRY.get(pub_id, None)

    def get_parameter(self, name: str):
        """
        Get a parameter value if it exists.

        Args:
            name (str): Name of the parameter.

        Returns
        -------
            [type]: The value of the parameter.

        """
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
        """
        Get all the parameter associated with the node.

        Args:
            node_name (str): THe name of the node.
            parameter_names (list): List of parameter names.
            timeout_sec (float, optional): Seconds to wait. If ``None``, then
            wait forever. Defaults to 5.0.
        """
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
        future = self.spin_until_future_complete(
            client=client, request=request)

        response = future.result()
        if response is None:
            e = future.exception()
            raise RuntimeError(
                f"Exception while calling service of '{node_name}': {e}"
            )
        return response

    @property
    def use_sim_time(self):
        """Get use_sim_time Value."""
        try:
            return self.get_parameter('use_sim_time').value
        except AttributeError:
            pass
        return False

    @property
    def logger(self):
        """Get Logger."""
        return self.node.get_logger()

    @property
    def clock_now(self):
        """Get current time."""
        return self.node.get_clock().now()

    @property
    def node(self):
        """Get Xnode object."""
        return self._REGISTRY.get(self._node_name, None)

    """
    def __del__(self):
        try:
            self.node.destroy_node()
        except AttributeError:
            pass"""


def node_init(args=None):
    """Initialize of Xnode."""
    rclpy.init(args=args)
    return XNodeBuilder()

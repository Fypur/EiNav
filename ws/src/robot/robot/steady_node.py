from typing import List

from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter

from rclpy.clock import Clock, ClockType


#this is a class to make everything work in wsl2
#it changes the node's clock types to be steady, else after sometime the timers just stop working in wsl2
class SteadyNode(Node):

    def __init__(self,
                 node_name: str,
                 *,
                 context: Context | None = None,
                 cli_args: List[str] | None = None,
                 namespace: str | None = None,
                 use_global_arguments: bool = True,
                 enable_rosout: bool = True,
                 start_parameter_services: bool = True,
                 parameter_overrides: List[Parameter] | None = None,
                 allow_undeclared_parameters: bool = False,
                 automatically_declare_parameters_from_overrides: bool = False,
                 enable_logger_service: bool = False) -> None:
        super().__init__(
            node_name,
            context=context,
            cli_args=cli_args,
            namespace=namespace,
            use_global_arguments=use_global_arguments,
            enable_rosout=enable_rosout,
            start_parameter_services=start_parameter_services,
            parameter_overrides=parameter_overrides,
            allow_undeclared_parameters=allow_undeclared_parameters,
            automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides,
            enable_logger_service=enable_logger_service)

        self._clock = Clock(clock_type=ClockType.STEADY_TIME)  #IMPORTANT FOR THIS TO WORK ON WSL2 !!!!!!!

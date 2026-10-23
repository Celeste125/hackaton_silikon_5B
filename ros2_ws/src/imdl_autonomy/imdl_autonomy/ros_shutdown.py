"""Shared, signal-tolerant teardown for ROS 2 Python autonomy nodes."""

import time

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor


_TRANSIENT_TAKE_ERROR = 'Unable to convert call argument'


class DiagnosticSingleThreadedExecutor(SingleThreadedExecutor):
    """Attach the offending subscription contract to rclpy take errors."""

    def _take_subscription(self, subscription):
        try:
            return super()._take_subscription(subscription)
        except RuntimeError as error:
            topic = getattr(subscription, 'topic_name', '<unknown>')
            message_type = getattr(subscription, 'msg_type', '<unknown>')
            raise RuntimeError(
                f'{error}; topic={topic}; type={message_type}'
            ) from error


def spin_node_resilient(node, executor=None) -> None:
    """Spin a node while tolerating isolated Humble DDS take failures."""
    owned_executor = executor is None
    if owned_executor:
        executor = DiagnosticSingleThreadedExecutor(context=node.context)
        executor.add_node(node)

    consecutive_take_errors = 0
    try:
        while rclpy.ok(context=node.context):
            try:
                executor.spin_once(timeout_sec=0.2)
                consecutive_take_errors = 0
            except RuntimeError as error:
                if _TRANSIENT_TAKE_ERROR not in str(error):
                    raise
                # SIGINT can invalidate the context while a subscription is
                # being taken. In that case teardown has already started and
                # rebuilding an executor would create entities on a dead
                # context.
                if not rclpy.ok(context=node.context):
                    break
                consecutive_take_errors += 1
                if (
                    consecutive_take_errors == 1
                    or consecutive_take_errors % 10 == 0
                ):
                    node.get_logger().warning(
                        'discarding transient DDS conversion failure '
                        f'(consecutive={consecutive_take_errors}): {error}'
                    )
                if consecutive_take_errors >= 100:
                    raise RuntimeError(
                        'DDS message conversion failed 100 times in a row'
                    ) from error
                if owned_executor:
                    executor.remove_node(node)
                    executor.shutdown(timeout_sec=0.5)
                    executor = DiagnosticSingleThreadedExecutor(
                        context=node.context
                    )
                    executor.add_node(node)
                time.sleep(0.01)
    except ExternalShutdownException:
        pass
    finally:
        if owned_executor:
            try:
                executor.shutdown()
            except ExternalShutdownException:
                pass


def shutdown_node(node, executor=None) -> None:
    if executor is not None:
        try:
            executor.shutdown()
        except KeyboardInterrupt:
            pass
    try:
        node.destroy_node()
    except KeyboardInterrupt:
        # ros2 launch can deliver SIGINT while an entity is being destroyed.
        pass
    if rclpy.ok():
        try:
            rclpy.shutdown()
        except KeyboardInterrupt:
            pass

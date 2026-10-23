"""Non-blocking access to Create3-specific runtime parameters."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.parameter import Parameter


class Create3SafetyState(str, Enum):
    IDLE = 'IDLE'
    ACTIVE = 'ACTIVE'
    SUCCEEDED = 'SUCCEEDED'
    FAILED = 'FAILED'


@dataclass(frozen=True)
class Create3SafetySnapshot:
    state: Create3SafetyState
    operation: str
    message: str

    @property
    def terminal(self) -> bool:
        return self.state in {
            Create3SafetyState.SUCCEEDED,
            Create3SafetyState.FAILED,
        }


class Create3SafetyAdapter:
    """Temporarily disable only Create3's finite backup-distance limit."""

    def __init__(self, node, remote_node: str):
        self.node = node
        self.remote_node = remote_node.rstrip('/')
        self.get_client = node.create_client(
            GetParameters,
            f'{self.remote_node}/get_parameters',
        )
        self.set_client = node.create_client(
            SetParameters,
            f'{self.remote_node}/set_parameters',
        )
        self.state = Create3SafetyState.IDLE
        self.operation = ''
        self.message = ''
        self.original_mode = None
        self.changed_by_us = False
        self._generation = 0

    def snapshot(self) -> Create3SafetySnapshot:
        return Create3SafetySnapshot(
            self.state,
            self.operation,
            self.message,
        )

    def services_ready(self) -> bool:
        return (
            self.get_client.service_is_ready()
            and self.set_client.service_is_ready()
        )

    def enable_backup_only(self) -> bool:
        if self.state is Create3SafetyState.ACTIVE:
            self.message = 'Create3 safety request already active'
            return False
        if not self.services_ready():
            self.state = Create3SafetyState.FAILED
            self.operation = 'ENABLE_BACKUP_ONLY'
            self.message = (
                f'Parameter services unavailable on {self.remote_node}'
            )
            return False

        self._generation += 1
        generation = self._generation
        self.state = Create3SafetyState.ACTIVE
        self.operation = 'ENABLE_BACKUP_ONLY'
        self.message = 'reading existing safety_override'
        request = GetParameters.Request()
        request.names = ['safety_override']
        future = self.get_client.call_async(request)
        future.add_done_callback(
            lambda completed: self._existing_mode(completed, generation)
        )
        return True

    def restore(self) -> bool:
        if self.state is Create3SafetyState.ACTIVE:
            self._generation += 1
            if self.original_mode is None:
                self.state = Create3SafetyState.SUCCEEDED
                self.operation = 'RESTORE_SAFETY_OVERRIDE'
                self.message = 'Canceled before safety_override was changed'
                return True
            # A pending set request may still reach the server. Queue the
            # original mode after it on the same service client.
            self.changed_by_us = True
        self.operation = 'RESTORE_SAFETY_OVERRIDE'
        if not self.changed_by_us:
            self.state = Create3SafetyState.SUCCEEDED
            self.message = 'Create3 safety_override was not changed'
            return True
        if not self.set_client.service_is_ready():
            self.state = Create3SafetyState.FAILED
            self.message = (
                f'SetParameters unavailable on {self.remote_node}; '
                f'expected restore mode {self.original_mode}'
            )
            return False

        self._generation += 1
        generation = self._generation
        self.state = Create3SafetyState.ACTIVE
        self.message = f'restoring safety_override={self.original_mode}'
        self._set_mode(
            self.original_mode or 'none',
            generation,
            restoring=True,
        )
        return True

    def _existing_mode(self, future, generation: int) -> None:
        if generation != self._generation:
            return
        try:
            response = future.result()
            value = response.values[0]
        except Exception as error:
            self._fail(f'Could not read safety_override: {error}')
            return
        if value.type != ParameterType.PARAMETER_STRING:
            self._fail('Create3 safety_override is not a string parameter')
            return

        self.original_mode = value.string_value
        if self.original_mode in {'backup_only', 'full'}:
            self.state = Create3SafetyState.SUCCEEDED
            self.message = (
                f'safety_override already permits reverse: '
                f'{self.original_mode}'
            )
            return
        self.message = 'setting safety_override=backup_only'
        self._set_mode('backup_only', generation, restoring=False)

    def _set_mode(
        self,
        mode: str,
        generation: int,
        restoring: bool,
    ) -> None:
        request = SetParameters.Request()
        request.parameters = [
            Parameter('safety_override', value=mode).to_parameter_msg()
        ]
        future = self.set_client.call_async(request)
        future.add_done_callback(
            lambda completed: self._mode_set(
                completed,
                generation,
                mode,
                restoring,
            )
        )

    def _mode_set(
        self,
        future,
        generation: int,
        mode: str,
        restoring: bool,
    ) -> None:
        if generation != self._generation:
            return
        try:
            response = future.result()
            result = response.results[0]
        except Exception as error:
            self._fail(f'Could not set safety_override={mode}: {error}')
            return
        if not result.successful:
            self._fail(
                f'Create3 rejected safety_override={mode}: {result.reason}'
            )
            return

        self.state = Create3SafetyState.SUCCEEDED
        self.message = f'safety_override={mode}'
        if restoring:
            self.changed_by_us = False
        else:
            self.changed_by_us = True

    def _fail(self, message: str) -> None:
        self.state = Create3SafetyState.FAILED
        self.message = message

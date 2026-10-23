"""Task-name registry for the independent autonomy solvers."""

from .task1 import Task1Solver
from .task2 import Task2Solver
from .task3 import Task3Solver
from .task4 import Task4Solver
from .task5 import Task5Solver

SOLVERS = {
    'task1': Task1Solver,
    'task2': Task2Solver,
    'task3': Task3Solver,
    'task4': Task4Solver,
    'task5': Task5Solver,
}


def create_solver(task: str, context):
    try:
        solver_class = SOLVERS[task]
    except KeyError as error:
        raise ValueError(
            f'Unknown autonomy task {task!r}; available={sorted(SOLVERS)}'
        ) from error
    return solver_class(context)

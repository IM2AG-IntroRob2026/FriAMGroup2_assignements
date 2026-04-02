from enum import Enum, auto


class PatrolState(Enum):
    DOCKED = auto()
    UNDOCKING = auto()
    PATROLLING = auto()
    INTRUDER_ALERT = auto()
    AVOIDING = auto()
    RETURNING = auto()
    DOCKING = auto()
    MANUAL_OVERRIDE = auto()


class PatrolFSM:

    def __init__(self, num_laps=3):
        self.state = PatrolState.DOCKED
        self.laps_done = 0
        self.num_laps = num_laps
        self._pre_override_state = None

    def on_start(self):
        if self.state == PatrolState.DOCKED:
            self.state = PatrolState.UNDOCKING
        elif self.state == PatrolState.MANUAL_OVERRIDE:
            self.on_resume_service()
        else:
            raise ValueError(f'on_start() illegal in state {self.state}')

    def on_undock_done(self):
        if self.state != PatrolState.UNDOCKING:
            raise ValueError(f'on_undock_done() illegal in state {self.state}')
        self.state = PatrolState.PATROLLING

    def on_lap_done(self):
        if self.state != PatrolState.PATROLLING:
            raise ValueError(f'on_lap_done() illegal in state {self.state}')
        self.laps_done += 1
        if self.laps_done >= self.num_laps:
            self.state = PatrolState.RETURNING
        # else stay PATROLLING

    def on_hazard(self):
        if self.state != PatrolState.PATROLLING:
            raise ValueError(f'on_hazard() illegal in state {self.state}')
        self.state = PatrolState.INTRUDER_ALERT

    def on_siren_done(self):
        if self.state != PatrolState.INTRUDER_ALERT:
            raise ValueError(f'on_siren_done() illegal in state {self.state}')
        self.state = PatrolState.AVOIDING

    def on_avoidance_done(self):
        if self.state != PatrolState.AVOIDING:
            raise ValueError(f'on_avoidance_done() illegal in state {self.state}')
        self.state = PatrolState.PATROLLING

    def on_near_dock(self):
        if self.state != PatrolState.RETURNING:
            raise ValueError(f'on_near_dock() illegal in state {self.state}')
        self.state = PatrolState.DOCKING

    def on_dock_done(self):
        if self.state != PatrolState.DOCKING:
            raise ValueError(f'on_dock_done() illegal in state {self.state}')
        self.state = PatrolState.DOCKED

    def on_stop_service(self):
        self._pre_override_state = self.state
        self.state = PatrolState.MANUAL_OVERRIDE

    def on_resume_service(self):
        if self.state != PatrolState.MANUAL_OVERRIDE:
            raise ValueError(f'on_resume_service() illegal in state {self.state}')
        if self._pre_override_state is None:
            raise ValueError('No previous state to restore')
        self.state = self._pre_override_state
        self._pre_override_state = None

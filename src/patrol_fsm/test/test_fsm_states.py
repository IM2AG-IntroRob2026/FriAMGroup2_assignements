import pytest
from patrol_fsm.fsm_states import PatrolFSM, PatrolState


def make_fsm(num_laps=3):
    return PatrolFSM(num_laps=num_laps)


# ── Legal transitions ──────────────────────────────────────────────────────────

def test_start_docked_to_undocking():
    fsm = make_fsm()
    assert fsm.state == PatrolState.DOCKED
    fsm.on_start()
    assert fsm.state == PatrolState.UNDOCKING


def test_undock_done_to_patrolling():
    fsm = make_fsm()
    fsm.on_start()
    fsm.on_undock_done()
    assert fsm.state == PatrolState.PATROLLING


def test_lap_done_stays_patrolling_when_laps_remain():
    fsm = make_fsm(num_laps=3)
    fsm.on_start()
    fsm.on_undock_done()
    fsm.on_lap_done()  # laps_done=1 < 3
    assert fsm.state == PatrolState.PATROLLING
    assert fsm.laps_done == 1
    fsm.on_lap_done()  # laps_done=2 < 3
    assert fsm.state == PatrolState.PATROLLING


def test_lap_done_transitions_to_returning_when_all_laps_done():
    fsm = make_fsm(num_laps=2)
    fsm.on_start()
    fsm.on_undock_done()
    fsm.on_lap_done()  # laps_done=1 < 2
    assert fsm.state == PatrolState.PATROLLING
    fsm.on_lap_done()  # laps_done=2 == 2
    assert fsm.state == PatrolState.RETURNING


def test_hazard_patrolling_to_intruder_alert():
    fsm = make_fsm()
    fsm.on_start()
    fsm.on_undock_done()
    fsm.on_hazard()
    assert fsm.state == PatrolState.INTRUDER_ALERT


def test_siren_done_to_avoiding():
    fsm = make_fsm()
    fsm.on_start()
    fsm.on_undock_done()
    fsm.on_hazard()
    fsm.on_siren_done()
    assert fsm.state == PatrolState.AVOIDING


def test_avoidance_done_to_patrolling():
    fsm = make_fsm()
    fsm.on_start()
    fsm.on_undock_done()
    fsm.on_hazard()
    fsm.on_siren_done()
    fsm.on_avoidance_done()
    assert fsm.state == PatrolState.PATROLLING


def test_near_dock_returning_to_docking():
    fsm = make_fsm(num_laps=1)
    fsm.on_start()
    fsm.on_undock_done()
    fsm.on_lap_done()
    assert fsm.state == PatrolState.RETURNING
    fsm.on_near_dock()
    assert fsm.state == PatrolState.DOCKING


def test_dock_done_to_docked():
    fsm = make_fsm(num_laps=1)
    fsm.on_start()
    fsm.on_undock_done()
    fsm.on_lap_done()
    fsm.on_near_dock()
    fsm.on_dock_done()
    assert fsm.state == PatrolState.DOCKED


def test_stop_service_from_any_state_goes_to_manual_override():
    for setup in [
        lambda f: None,                                    # DOCKED
        lambda f: f.on_start(),                            # UNDOCKING
        lambda f: (f.on_start(), f.on_undock_done()),      # PATROLLING
    ]:
        fsm = make_fsm()
        setup(fsm)
        prev = fsm.state
        fsm.on_stop_service()
        assert fsm.state == PatrolState.MANUAL_OVERRIDE
        assert fsm._pre_override_state == prev


def test_resume_service_restores_previous_state():
    fsm = make_fsm()
    fsm.on_start()
    fsm.on_undock_done()
    assert fsm.state == PatrolState.PATROLLING
    fsm.on_stop_service()
    assert fsm.state == PatrolState.MANUAL_OVERRIDE
    fsm.on_resume_service()
    assert fsm.state == PatrolState.PATROLLING


# ── Guard / illegal transitions ────────────────────────────────────────────────

def test_on_start_illegal_when_not_docked_or_override():
    fsm = make_fsm()
    fsm.on_start()  # → UNDOCKING
    with pytest.raises(ValueError):
        fsm.on_start()


def test_on_undock_done_illegal_outside_undocking():
    fsm = make_fsm()
    with pytest.raises(ValueError):
        fsm.on_undock_done()


def test_on_lap_done_illegal_outside_patrolling():
    fsm = make_fsm()
    with pytest.raises(ValueError):
        fsm.on_lap_done()


def test_on_hazard_illegal_outside_patrolling():
    fsm = make_fsm()
    with pytest.raises(ValueError):
        fsm.on_hazard()


def test_on_siren_done_illegal_outside_intruder_alert():
    fsm = make_fsm()
    fsm.on_start()
    fsm.on_undock_done()
    with pytest.raises(ValueError):
        fsm.on_siren_done()


def test_on_avoidance_done_illegal_outside_avoiding():
    fsm = make_fsm()
    with pytest.raises(ValueError):
        fsm.on_avoidance_done()


def test_on_near_dock_illegal_outside_returning():
    fsm = make_fsm()
    with pytest.raises(ValueError):
        fsm.on_near_dock()


def test_on_dock_done_illegal_outside_docking():
    fsm = make_fsm()
    with pytest.raises(ValueError):
        fsm.on_dock_done()


def test_on_resume_service_illegal_outside_manual_override():
    fsm = make_fsm()
    with pytest.raises(ValueError):
        fsm.on_resume_service()


def test_laps_done_counter_increments_correctly():
    fsm = make_fsm(num_laps=5)
    fsm.on_start()
    fsm.on_undock_done()
    for i in range(1, 6):
        if i < 5:
            fsm.on_lap_done()
            assert fsm.laps_done == i
            assert fsm.state == PatrolState.PATROLLING
        else:
            fsm.on_lap_done()
            assert fsm.laps_done == 5
            assert fsm.state == PatrolState.RETURNING

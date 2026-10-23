from imdl_autonomy.safety_monitor import evaluate_safety


def test_safety_watchdog_applies_grace_then_stale_stop():
    common = dict(
        started=0.0,
        last_scan=0.0,
        last_odom=0.0,
        scan_timeout=1.0,
        odom_timeout=0.5,
        startup_grace=2.0,
        hazard_until=-1.0,
        active_hazards=[],
        kidnapped=False,
    )

    assert evaluate_safety(now=1.0, **common) == []
    assert evaluate_safety(now=3.0, **common) == [
        'SCAN_STALE', 'ODOM_STALE'
    ]


def test_safety_watchdog_holds_hazard_and_kidnap():
    reasons = evaluate_safety(
        now=5.0,
        started=0.0,
        last_scan=5.0,
        last_odom=5.0,
        scan_timeout=1.0,
        odom_timeout=1.0,
        startup_grace=2.0,
        hazard_until=6.0,
        active_hazards=['BUMP'],
        kidnapped=True,
    )

    assert reasons == ['BUMP', 'KIDNAPPED']

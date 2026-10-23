from imdl_autonomy.tracking import Detection, MultiObjectTracker


def _detection(x, y, stamp):
    return Detection('red', x, y, 0.0175, stamp, variance=0.0004)


def test_tracker_confirms_consistent_measurements_and_gates_outlier():
    tracker = MultiObjectTracker(confirmations_required=3, lifetime_sec=5.0)
    tracker.update([_detection(0.10, 0.20, 0.0)], 0.0)
    tracker.update([_detection(0.105, 0.198, 0.1)], 0.1)
    tracker.update([_detection(0.102, 0.201, 0.2)], 0.2)

    confirmed = tracker.tracks[0]
    assert tracker.status(confirmed, 0.2) == 'CONFIRMED'
    assert confirmed.confirmations == 3

    tracker.update([_detection(0.80, 0.80, 0.3)], 0.3)
    assert len(tracker.tracks) == 2


def test_tracker_marks_stale_then_expires():
    tracker = MultiObjectTracker(stale_after_sec=1.0, lifetime_sec=2.0)
    tracker.update([_detection(0.0, 0.0, 0.0)], 0.0)
    track = tracker.tracks[0]

    assert tracker.status(track, 1.1) == 'STALE'
    tracker.prune(2.1)
    assert tracker.tracks == ()


def test_duplicate_or_out_of_order_timestamp_does_not_confirm_track():
    tracker = MultiObjectTracker(confirmations_required=2)
    tracker.update([_detection(0.10, 0.20, 1.0)], 1.0)
    tracker.update([_detection(0.11, 0.20, 1.0)], 1.1)

    track = tracker.tracks[0]
    assert track.confirmations == 1
    assert track.rejected_updates == 1
    assert tracker.status(track, 1.1) == 'TENTATIVE'

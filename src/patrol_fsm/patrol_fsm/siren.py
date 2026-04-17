"""
siren.py — Builds an AudioNoteSequence goal for the INTRUDER_ALERT siren.

Pattern: 4 repetitions of 880 Hz (400 ms) then 659 Hz (400 ms)
Total duration: 8 notes × 400 ms = 3.2 s

No ROS imports at module level — caller imports action type and passes it in.
"""

# Siren note definitions
_NOTES = [
    (880, 400),   # A5 — high tone
    (659, 400),   # E5 — low tone
]
_REPETITIONS = 4


def build_siren_goal(AudioNoteSequence, AudioNote, duration_ms_to_ros):
    """
    Build and return an AudioNoteSequence.Goal.

    Parameters
    ----------
    AudioNoteSequence : action class
        irobot_create_msgs.action.AudioNoteSequence
    AudioNote : message class
        irobot_create_msgs.msg.AudioNote
    duration_ms_to_ros : callable
        Converts an integer milliseconds value to a ROS Duration message.
        Typically: ``lambda ms: rclpy.duration.Duration(seconds=ms/1000).to_msg()``

    Returns
    -------
    AudioNoteSequence.Goal
    """
    goal = AudioNoteSequence.Goal()
    goal.iterations = 1  # the note list already contains all repetitions

    notes = []
    for _ in range(_REPETITIONS):
        for freq_hz, dur_ms in _NOTES:
            note = AudioNote()
            note.frequency = freq_hz
            note.max_runtime = duration_ms_to_ros(dur_ms)
            notes.append(note)

    goal.note_sequence.notes = notes
    goal.note_sequence.append = False
    return goal

"""Read-only measured state for the privileged scripted demonstration.

The runner assigns ``latest_sample`` after each measured-state audit, before
the next policy action. Only the demonstration reads this channel; it is not a
Dream observation. The demo may append JSON-serializable ``demo_events`` for
the runner to archive, but never writes the sampled object or scoring state.
"""

latest_sample = None
demo_events = []

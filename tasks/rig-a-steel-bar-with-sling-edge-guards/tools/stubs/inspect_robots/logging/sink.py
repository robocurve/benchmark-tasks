"""Stand-in for Inspect Robots' NullSink, which rdb_sling_guards/dream_audit.py subclasses, so that its pure logic can be
tested where the runtime is not installed. A real Inspect Robots, where installed, comes first on the path."""


class NullSink:
    critical = False

    def log_step(self, t, observation, action, result):
        pass

    def close(self):
        pass

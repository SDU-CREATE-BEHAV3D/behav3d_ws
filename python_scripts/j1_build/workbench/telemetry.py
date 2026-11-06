"""Small append-only operation records, including unfinished operations."""
from datetime import datetime, timezone
import json


def event(output, operation, state, **details):
    record = {"time": datetime.now(timezone.utc).isoformat(),
              "operation": operation, "state": state, **details}
    with (output / "performance.jsonl").open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(record) + "\n")

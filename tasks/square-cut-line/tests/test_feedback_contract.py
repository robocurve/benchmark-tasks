# SPDX-License-Identifier: MIT
"""Every reachable controller phase must have a bounded timeout."""
import ast
from pathlib import Path


def test_all_controller_transitions_have_deadlines():
    source=Path(__file__).parents[1]/"square_cut_line/feedback_policy.py"
    tree=ast.parse(source.read_text())
    limits=next(ast.literal_eval(node.value) for node in tree.body
                if isinstance(node,ast.Assign)
                and any(isinstance(t,ast.Name) and t.id=="PHASE_LIMITS" for t in node.targets))
    phases={node.args[0].value for node in ast.walk(tree)
            if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute)
            and node.func.attr=="_transition" and node.args
            and isinstance(node.args[0],ast.Constant)}
    assert phases <= limits.keys()
    assert all(0<float(seconds)<=30 for seconds in limits.values())

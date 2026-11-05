"""Session replacement tests without a Dream client or network."""
from pathlib import Path
import tempfile
import unittest
from dream_runner import request_with_recovery,guard_exchange
from dream_budget import Ledger,DEFAULTS,BudgetStopped

class Expired(Exception):pass
class Transport:
    def _drop(self):pass

class RecoveryTests(unittest.TestCase):
    def test_preserves_command_and_does_not_reset(self):
        commands=[];command=object();stats={'session_recoveries':0}
        class Client:
            def get_action(self,*,commanded):
                commands.append(commanded)
                if len(commands)==1:raise Expired()
                return {'ok':True}
            def reset_episode(self):raise AssertionError('Must not reset the trial')
        self.assertEqual(request_with_recovery(Client(),command,Expired,stats),{'ok':True})
        self.assertEqual(commands,[command,command]);self.assertEqual(stats['session_recoveries'],1)

    def test_one_recovery_total(self):
        calls=[];stats={'session_recoveries':0}
        class Client:
            def get_action(self,**kw):calls.append(1);raise Expired()
        with self.assertRaises(Expired):request_with_recovery(Client(),None,Expired,stats)
        self.assertEqual(len(calls),2)
        with self.assertRaises(Expired):request_with_recovery(Client(),None,Expired,stats)
        self.assertEqual(len(calls),3)

    def test_recovery_obeys_same_budget(self):
        with tempfile.TemporaryDirectory() as d:
            ledger=Ledger.initialize(Path(d)/'budget',limits=dict(DEFAULTS,task_micro_usd=1500))
            sent=[]
            def wire(*a):sent.append(1);return (200,b'{}',{})
            exchange=guard_exchange(wire,ledger,'task','run')
            class Client:
                def get_action(self,**kw):exchange(Transport(),b'',{});raise Expired()
            with self.assertRaises(BudgetStopped):request_with_recovery(Client(),None,Expired,{'session_recoveries':0})
            self.assertEqual(len(sent),1)
            self.assertEqual(ledger.status('task','run')['reserved']['run_calls'],1)

    def test_other_errors_are_not_retried(self):
        calls=[]
        class Client:
            def get_action(self,**kw):calls.append(1);raise ValueError('invalid reply')
        with self.assertRaises(ValueError):request_with_recovery(Client(),None,Expired,{'session_recoveries':0})
        self.assertEqual(len(calls),1)

if __name__=='__main__':unittest.main()

import pytest
from src.host.high_level_fsm import HighLevelStateMachine

class DummyFSM(HighLevelStateMachine):
    def __init__(self):
        super().__init__()
        self.log = []
    def on_exit_IDLE(self):      self.log.append('exit_IDLE')
    def on_enter_PATROL(self):   self.log.append('enter_PATROL')
    def on_enter_TRACKING(self): self.log.append('enter_TRACKING')
    def on_enter_AIMING(self):   self.log.append('enter_AIMING')
    def on_enter_FIRING(self):   self.log.append('enter_FIRING')
    def on_enter_RETREAT(self):  self.log.append('enter_RETREAT')
    def on_enter_FAILSAFE(self): self.log.append('enter_FAILSAFE')
    def on_enter_IDLE(self):     self.log.append('enter_IDLE')

@pytest.fixture
def fsm():
    return HighLevelStateMachine()


def test_initial_state(fsm):
    assert fsm.state == 'IDLE'
    assert not fsm.is_running()
    # 验证 FSM 初始化时处于 IDLE 状态，且 is_running() 返回 False


def test_start_patrol(fsm):
    assert fsm.post_event('START_PATROL')
    assert fsm.state == 'PATROL'
    assert fsm.is_running()
    # 验证接收到 START_PATROL 事件后，状态从 IDLE 切换到 PATROL，且 FSM 开始运行


def test_invalid_event(fsm):
    assert not fsm.post_event('UNKNOWN')
    assert fsm.state == 'IDLE'
    # 验证未知事件不引起状态变化，仍保持在当前状态


def test_fail_safe_global(fsm):
    fsm.post_event('START_PATROL')
    assert fsm.post_event('FAILSAFE')
    assert fsm.state == 'FAILSAFE'
    # 验证 FAILSAFE 事件可从任意状态切换到 FAILSAFE


def test_recover_from_fail_safe(fsm):
    fsm.post_event('FAILSAFE')
    assert fsm.post_event('RECOVER')
    assert fsm.state == 'IDLE'
    # 验证从 FAILSAFE 状态接收 RECOVER 事件后，恢复到 IDLE 状态


def test_detect_and_track():
    d = DummyFSM()
    d.post_event('START_PATROL')
    d.post_event('TARGET_DETECTED')
    assert d.state == 'TRACKING'
    assert d.log == ['exit_IDLE','enter_PATROL','enter_TRACKING']
    # 验证从 PATROL 状态接收 TARGET_DETECTED 事件后，切换到 TRACKING 状态


def test_full_sequence_to_retreat():
    d = DummyFSM()
    # IDLE -> PATROL -> TRACKING -> AIMING -> FIRING -> RETREAT
    assert d.post_event('START_PATROL')
    assert d.post_event('TARGET_DETECTED')
    assert d.post_event('TARGET_STABLE')
    assert d.post_event('READY_TO_FIRE')
    assert d.post_event('HEAT_ALERT')
    assert d.state == 'RETREAT'
    expected = [
        'exit_IDLE','enter_PATROL',
        'enter_TRACKING',
        'enter_AIMING',
        'enter_FIRING',
        'enter_RETREAT'
    ]
    assert d.log[:6] == expected
    # 验证完整事件序列：START_PATROL -> TARGET_DETECTED -> TARGET_STABLE -> READY_TO_FIRE -> HEAT_ALERT，可达 RETREAT 状态


def test_out_of_ammo_to_retreat():
    d = DummyFSM()
    d.post_event('START_PATROL')
    d.post_event('TARGET_DETECTED')
    d.post_event('TARGET_STABLE')
    d.post_event('READY_TO_FIRE')
    assert d.post_event('OUT_OF_AMMO')
    assert d.state == 'RETREAT'
    # 验证在 FIRING 状态接收 OUT_OF_AMMO 事件后，切换到 RETREAT 状态


def test_target_lost_back_to_patrol():
    d = DummyFSM()
    d.post_event('START_PATROL')
    d.post_event('TARGET_DETECTED')
    assert d.post_event('TARGET_LOST')
    assert d.state == 'PATROL'
    # 验证在 TRACKING 状态接收 TARGET_LOST 事件后，返回 PATROL 状态


def test_retreat_recover_to_idle():
    d = DummyFSM()
    d.post_event('START_PATROL')
    d.post_event('TARGET_DETECTED')
    d.post_event('TARGET_STABLE')
    d.post_event('READY_TO_FIRE')
    d.post_event('OUT_OF_AMMO')
    assert d.post_event('RECOVER')
    assert d.state == 'IDLE'
    # 验证在 RETREAT 状态接收 RECOVER 事件后，恢复到 IDLE 状态

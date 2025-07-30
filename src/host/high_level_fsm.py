"""
High-Level State Machine for Host AI.
States:
  IDLE, PATROL, TRACKING, AIMING, FIRING, RETREAT, FAILSAFE
Events:
  START_PATROL, TARGET_DETECTED, TARGET_STABLE, READY_TO_FIRE,
  OUT_OF_AMMO, HEAT_ALERT, TARGET_LOST, RECOVER, FAILSAFE
"""

class HighLevelStateMachine:
    """
    主状态机：控制 IDLE、PATROL、TRACKING、AIMING、FIRING、RETREAT、FAILSAFE 等状态的转移。
    使用事件驱动：调用 post_event 触发状态变化，然后执行对应的 on_exit/on_enter 回调。
    """
    def __init__(self, navigator=None, vision=None, turret=None, weapon=None, patrol_timer=None):
        """
        初始化状态机，并注入各子模块接口：
        - navigator: 导航模块（start_patrol, hold_position, plan_retreat 等）
        - vision: 视觉模块（enable_scanning, start_tracking 等）
        - turret: 云台模块（set_yaw, set_pitch, stop）
        - weapon: 弹控模块（can_fire, trigger_fire）
        - patrol_timer: 巡逻定时器（start/stop）
        """
        self.state = 'IDLE'  # 初始状态为待机
        # 注入模块接口，可为 None，在测试时不需传入
        self.navigator = navigator
        self.vision = vision
        self.turret = turret
        self.weapon = weapon
        self.patrol_timer = patrol_timer
        # 定义有限状态转换表：{(当前状态, 事件): 新状态}
        self._transitions = {
            ('IDLE', 'START_PATROL'): 'PATROL',
            ('PATROL', 'TARGET_DETECTED'): 'TRACKING',
            ('TRACKING', 'TARGET_STABLE'): 'AIMING',
            ('AIMING', 'READY_TO_FIRE'): 'FIRING',
            ('FIRING', 'OUT_OF_AMMO'): 'RETREAT',
            ('FIRING', 'HEAT_ALERT'): 'RETREAT',
            ('TRACKING', 'TARGET_LOST'): 'PATROL',
            ('AIMING', 'TARGET_LOST'): 'PATROL',
            ('FIRING', 'TARGET_LOST'): 'PATROL',
            ('RETREAT', 'RECOVER'): 'IDLE',
            ('FAILSAFE', 'RECOVER'): 'IDLE',
        }

    def post_event(self, event: str) -> bool:
        """
        接收事件并触发状态转移：
        - 如果事件为 FAILSAFE，则无条件切入 FAILSAFE 状态
        - 否则根据当前 (state, event) 在转换表中查找目标状态
        - 如果有匹配，则调用 _change_state 执行回调并更新状态
        返回 True 表示发生了状态转换，否则 False
        """
        if event == 'FAILSAFE' and self.state != 'FAILSAFE':
            return self._change_state('FAILSAFE')
        key = (self.state, event)
        if key in self._transitions:
            return self._change_state(self._transitions[key])
        return False

    def _change_state(self, new_state: str) -> bool:
        """
        真正执行状态切换的内部方法：
        1. 调用当前状态的 on_exit_<STATE> 回调
        2. 更新 self.state
        3. 调用新状态的 on_enter_<STATE> 回调
        """
        exit_cb = getattr(self, f'on_exit_{self.state}', None)
        if callable(exit_cb):
            exit_cb()
        old_state = self.state
        self.state = new_state
        enter_cb = getattr(self, f'on_enter_{new_state}', None)
        if callable(enter_cb):
            enter_cb()
        return True

    def is_running(self) -> bool:
        """
        检查是否处于运行状态（非 IDLE）
        """
        return self.state != 'IDLE'

    # ---------- 状态回调方法 ----------
    def on_enter_IDLE(self):
        # 进入 IDLE：可执行系统复位、资源清理
        # TODO[FSM_IDLE_ENTER]: implement system reset and resource cleanup
        pass

    def on_exit_IDLE(self):
        # 退出 IDLE：准备启动其他模块
        # TODO[FSM_IDLE_EXIT]: prepare modules for startup
        pass

    def on_enter_PATROL(self):
        # 进入巡逻：启动导航和视觉扫描，并开始巡逻定时器
        if self.navigator:
            self.navigator.start_patrol()
        if self.vision:
            self.vision.enable_scanning()
        if self.patrol_timer:
            self.patrol_timer.start()

    def on_exit_PATROL(self):
        # 退出巡逻：停止扫描和定时器
        if self.patrol_timer:
            self.patrol_timer.stop()
        if self.vision:
            self.vision.disable_scanning()
        # TODO[FSM_PATROL_EXIT]: cleanup patrol resources

    def on_enter_TRACKING(self):
        # 进入追踪：视觉模块开始跟踪，导航保持当前位置
        if self.vision:
            self.vision.start_tracking()
        if self.navigator:
            self.navigator.hold_position()

    def on_exit_TRACKING(self):
        # 退出追踪：清理追踪状态或滤波器
        # TODO[FSM_TRACKING_EXIT]: cleanup tracking state and filters
        pass

    def on_enter_AIMING(self):
        # 进入瞄准：获取目标角度并驱动云台
        if self.vision and self.turret:
            yaw, pitch = self.vision.get_target_angles()
            self.turret.set_yaw(yaw)
            self.turret.set_pitch(pitch)

    def on_exit_AIMING(self):
        # 退出瞄准：可停止云台微调
        # TODO[FSM_AIMING_EXIT]: stop turret fine adjustments
        pass

    def on_enter_FIRING(self):
        # 进入开火：判断弹控状态，触发射击或跳到 RETREAT
        if self.weapon:
            if self.weapon.can_fire():
                self.weapon.trigger_fire()
            else:
                self.post_event('OUT_OF_AMMO')

    def on_exit_FIRING(self):
        # 退出开火：清理射击状态
        # TODO[FSM_FIRING_EXIT]: cleanup firing state and logs
        pass

    def on_enter_RETREAT(self):
        # 进入撤退：规划撤退路径并执行
        if self.navigator:
            waypoints = self.navigator.plan_retreat()
            self.navigator.follow(waypoints)

    def on_exit_RETREAT(self):
        # 退出撤退：可清理路径数据
        # TODO[FSM_RETREAT_EXIT]: cleanup retreat path data
        pass

    def on_enter_FAILSAFE(self):
        # 进入保护模式：停止所有运动和扫描
        if self.navigator:
            self.navigator.stop()
        if self.vision:
            self.vision.disable_all()
        if self.turret:
            self.turret.stop()

    def on_exit_FAILSAFE(self):
        # 退出保护：恢复初始化检查
        # TODO[FSM_FAILSAFE_EXIT]: perform recovery initialization
        pass

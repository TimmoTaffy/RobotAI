class WeaponController:
    def __init__(self, ammo_count=10):
        self.ammo = ammo_count

    def can_fire(self):
        # 判断是否还有弹药
        return self.ammo > 0

    def trigger_fire(self):
        # 触发射击，并减少弹药
        if self.ammo > 0:
            # TODO: 实际射击逻辑
            self.ammo -= 1

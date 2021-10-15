from __future__ import division
import numpy as np

np.random.seed(42)


def idm_accel(dx, vego, vlead):
    v0 = 30
    T = 1
    a = 1
    b = 1.5
    delta = 4
    s0 = 2
    s = s0 + vego * T + vego * (vego - vlead) / (2 * np.sqrt(a * b))
    accel = a * (1 - (vego / v0) ** delta - (s / dx) ** 2)
    cmd_vel = vego + accel * 0.01
    if cmd_vel >= 0:
        return accel
    else:
        return 0


def fs_accel(dx, vego, vlead):
    dx10 = 4.5
    dx20 = 5.25
    dx30 = 6.0
    d1 = 1.5
    d2 = 1.0
    d3 = 0.5
    U = 5
    dv = vlead - vego
    dvm = min(dv, 0)
    v = min(max(vlead, 0), U)
    dx1 = dx10 + dvm ** 2 / (2 * d1)
    dx2 = dx20 + dvm ** 2 / (2 * d2)
    dx3 = dx30 + dvm ** 2 / (2 * d3)
    cmd_vel = (v * (dx - dx1) / (dx2 - dx1)) * (dx1 < dx <= dx2) + \
        (v + (U - v) * (dx - dx2) / (dx3 - dx2)) * (dx2 < dx <= dx3) + \
        U * (dx3 < dx)
    accel = np.clip((cmd_vel - vego) / 0.5, -1.5, 1.0)
    return accel


def add_vehicles_debug(kernel):
    kernel.vehicle.add('ego', 'r0', departPos='0')
    kernel.vehicle.setSpeedMode('ego', 0)
    kernel.vehicle.add('lead', 'r0', departPos='50')
    kernel.vehicle.setSpeedMode('lead', 0)


def get_accel(kernel, mode):
    lead_edge = kernel.vehicle.getRoadID('lead')
    lead_edgepos = kernel.vehicle.getLanePosition('lead')
    dx = kernel.vehicle.getDrivingDistance('ego', lead_edge, lead_edgepos)
    dx = dx - kernel.vehicle.getLength('lead')
    dx = dx - kernel.vehicle.getMinGap('ego')
    vego = kernel.vehicle.getSpeed('ego')
    vlead = kernel.vehicle.getSpeed('lead')
    if mode == 'idm':
        accel = idm_accel(dx, vego, vlead)
    elif mode == 'fs':
        accel = fs_accel(dx, vego, vlead)
    else:
        raise ValueError
    return accel


def get_next_vel(kernel, mode):
    vego = kernel.vehicle.getSpeed('ego')
    accel = get_accel(kernel, mode)
    next_vel = vego + accel * kernel.simulation.getDeltaT()
    return next_vel


def set_speed_debug(kernel, mode):
    next_vel = get_next_vel(kernel, mode)
    kernel.vehicle.setSpeed('ego', next_vel)
    kernel.vehicle.setSpeed('lead', 0)


def get_data_debug(kernel, mode):
    t = kernel.simulation.getTime() - kernel.simulation.getDeltaT()
    ego_pos = kernel.vehicle.getLanePosition('ego')
    ego_vel = kernel.vehicle.getSpeed('ego')
    ego_acc = get_accel(kernel, mode)
    lead_pos = kernel.vehicle.getLanePosition('lead')
    lead_vel = kernel.vehicle.getSpeed('lead')
    lead_acc = 0
    return [t, ego_pos, ego_vel, ego_acc, lead_pos, lead_vel, lead_acc]

from types import SimpleNamespace

from jdamr_cube_navigation.imu_bias_relay import GyroBias, wheels_moving


def test_bias_is_learned_only_after_the_wheels_stay_still():
    """Turning never feeds the bias; 10 s still converges to the offset."""
    bias = GyroBias(still_min_s=1.0, tau_s=5.0)
    t = 0.0
    bias.wheels(t, moving=True)
    for _ in range(100):                      # turning: never learned
        t += 0.02
        assert bias.gyro(t, [0.0, 0.0, 0.3])[2] == 0.3
    bias.wheels(t, moving=False)
    out = None
    for _ in range(500):                      # 10 s still at a -0.002 rad/s offset
        t += 0.02
        out = bias.gyro(t, [0.0, 0.0, -0.002])
    assert abs(bias.bias[2] + 0.002) < 1e-9
    assert abs(out[2]) < 1e-9
    bias.wheels(t, moving=True)
    t += 0.02
    assert abs(bias.gyro(t, [0.0, 0.0, 0.2])[2] - 0.202) < 1e-9


def test_one_encoder_count_is_still_but_a_slow_turn_is_not():
    """Encoder jitter at rest (0.00126 m/s, 0.00495 rad/s) must not block learning."""
    def twist(v, w):
        return SimpleNamespace(linear=SimpleNamespace(x=v), angular=SimpleNamespace(z=w))
    assert not wheels_moving(twist(0.00126, -0.00495))
    assert wheels_moving(twist(0.0, 0.05))
    assert wheels_moving(twist(0.02, 0.0))

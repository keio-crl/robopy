"""Per-model current units, signed decoding, the state block, and raw compatibility."""

from __future__ import annotations

import pytest

from robopy.config.robot_config.rakuda_config import RAKUDA_CONTROLTABLE_VALUES
from robopy.motor.dynamixel_control_table import (
    STATE_BLOCK_FIELDS,
    STATE_BLOCK_NUM_BYTES,
    STATE_BLOCK_START_ADDRESS,
    CurrentSense,
    Dtype,
    OperatingMode,
    XControlTable,
    cast_value,
    encode_value,
    get_motor_capabilities,
)
from robopy.motor.sim_dynamixel_bus import SimulatedDynamixelBus

from .conftest import make_sim_bus


class TestModelCapabilities:
    @pytest.mark.parametrize(
        ("model", "unit"),
        [("xm430-w350", 0.00269), ("xm540-w270", 0.00269), ("xc330-t288", 0.001)],
    )
    def test_current_unit_per_model(self, model: str, unit: float) -> None:
        assert get_motor_capabilities(model).current_unit_a == pytest.approx(unit)

    def test_model_name_is_case_insensitive(self) -> None:
        assert get_motor_capabilities("XM430-W350").model_name == "xm430-w350"

    def test_xc330_current_is_marked_as_supply_side(self) -> None:
        # The XC330 senses on the power-supply input, so its reading is not
        # interchangeable with the XM winding-current torque model.
        assert get_motor_capabilities("xc330-t288").current_sense == CurrentSense.SUPPLY_INPUT
        assert get_motor_capabilities("xm430-w350").current_sense == CurrentSense.MOTOR_WINDING

    def test_an_unlisted_model_is_refused_rather_than_guessed(self) -> None:
        # Sharing XControlTable is not evidence of verified units.
        with pytest.raises(ValueError, match="No verified physical-unit data"):
            get_motor_capabilities("xl330-m077")

    def test_required_modes_are_declared(self) -> None:
        capabilities = get_motor_capabilities("xm540-w270")
        assert capabilities.supports_current_control()
        assert capabilities.supports_current_based_position()
        assert OperatingMode.CURRENT in capabilities.supported_operating_modes


class TestSignedDecoding:
    @pytest.mark.parametrize(
        ("dtype", "value"),
        [
            (Dtype.INT8, -1),
            (Dtype.INT8, -128),
            (Dtype.INT8, 127),
            (Dtype.INT16, -1),
            (Dtype.INT16, -32768),
            (Dtype.INT32, -1),
            (Dtype.INT32, -2147483648),
        ],
    )
    def test_encode_decode_round_trip(self, dtype: Dtype, value: int) -> None:
        assert cast_value(encode_value(value, dtype), dtype) == value

    def test_watchdog_minus_one_decodes_as_int8(self) -> None:
        # BUS_WATCHDOG latches -1 on expiry; as a UINT8 that would read as 255.
        assert cast_value(0xFF, Dtype.INT8) == -1
        assert XControlTable.BUS_WATCHDOG.value.dtype is Dtype.INT8

    def test_out_of_range_values_are_refused(self) -> None:
        with pytest.raises(ValueError, match="INT16"):
            encode_value(40000, Dtype.INT16)


class TestStateBlock:
    def test_block_covers_current_velocity_and_position(self) -> None:
        assert STATE_BLOCK_START_ADDRESS == 126
        assert STATE_BLOCK_NUM_BYTES == 10
        assert [f.address for f in STATE_BLOCK_FIELDS] == [126, 128, 132]

    def test_each_field_keeps_its_own_width_and_sign(self) -> None:
        # The block is read in one transaction, but no single ControlItem is
        # widened to ten bytes: each field decodes at 2 or 4 bytes.
        widths = {f.name: (f.num_bytes, f.dtype) for f in STATE_BLOCK_FIELDS}
        assert widths["present_current"] == (2, Dtype.INT16)
        assert widths["present_velocity"] == (4, Dtype.INT32)
        assert widths["present_position"] == (4, Dtype.INT32)
        assert XControlTable.PRESENT_CURRENT.value.num_bytes == 2

    def test_block_matches_the_individual_control_items(self) -> None:
        assert XControlTable.PRESENT_CURRENT.value.address == STATE_BLOCK_FIELDS[0].address
        assert XControlTable.PRESENT_VELOCITY.value.address == STATE_BLOCK_FIELDS[1].address
        assert XControlTable.PRESENT_POSITION.value.address == STATE_BLOCK_FIELDS[2].address


class TestRawCompatibility:
    def test_gripper_current_constants_stay_raw(self) -> None:
        # These were previously commented "mA" but have always been written as
        # raw counts. The behaviour is what is preserved: 128 stays raw 128.
        assert RAKUDA_CONTROLTABLE_VALUES.FOLLOWER_GRIP_GOAL_CURRENT == 128
        assert RAKUDA_CONTROLTABLE_VALUES.LEADER_GRIP_GOAL_CURRENT == 30

    def test_raw_constants_convert_to_amperes_per_model(self) -> None:
        assert RAKUDA_CONTROLTABLE_VALUES.raw_current_to_a(128, "xm430-w350") == pytest.approx(
            0.34432
        )
        # The same raw value is a different current on an XC330 leader gripper.
        assert RAKUDA_CONTROLTABLE_VALUES.raw_current_to_a(30, "xc330-t288") == pytest.approx(0.03)

    def test_raw_value_reaches_the_motor_unchanged(self) -> None:
        bus = make_sim_bus(("l_arm_grip",))
        bus.sync_write(XControlTable.CURRENT_LIMIT, {"l_arm_grip": 1000})
        bus.sync_write(
            XControlTable.GOAL_CURRENT,
            {"l_arm_grip": RAKUDA_CONTROLTABLE_VALUES.FOLLOWER_GRIP_GOAL_CURRENT},
        )
        assert bus.registers("l_arm_grip").goal_current_raw == 128


class TestBulkReadAndCurrentWrite:
    def test_bulk_read_returns_raw_signed_values(self, sim_bus: SimulatedDynamixelBus) -> None:
        sim_bus.joint("torso_yaw").position_rad = -0.5
        sim_bus.joint("torso_yaw").velocity_rad_s = -1.0
        readings, start_ns, end_ns = sim_bus.read_state_block(["torso_yaw"])
        assert end_ns >= start_ns
        reading = readings["torso_yaw"]
        assert reading.valid
        assert reading.present_position < 2048
        assert reading.present_velocity < 0

    def test_goal_current_in_amperes_quantises_per_model(self) -> None:
        xm = make_sim_bus(("j",), model="xm430-w350")
        xc = make_sim_bus(("j",), model="xc330-t288")
        assert xm.write_goal_current_a({"j": 0.269})["j"] == 100
        assert xc.write_goal_current_a({"j": 0.269})["j"] == 269

    def test_negative_goal_current_survives_the_wire_encoding(
        self, sim_bus: SimulatedDynamixelBus
    ) -> None:
        sim_bus.write_goal_current_raw({"torso_yaw": -250})
        assert sim_bus.registers("torso_yaw").goal_current_raw == -250

    def test_goal_current_is_clamped_to_the_current_limit(
        self, sim_bus: SimulatedDynamixelBus
    ) -> None:
        sim_bus.sync_write(XControlTable.CURRENT_LIMIT, {"torso_yaw": 100})
        sim_bus.write_goal_current_raw({"torso_yaw": -5000})
        assert sim_bus.registers("torso_yaw").goal_current_raw == -100

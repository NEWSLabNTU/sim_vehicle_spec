"""The vehicle spec: loading, validation, what it derives, its fingerprints and the outputs
generated here. Pure: no simulator, no ROS."""
import copy
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
SPEC = ROOT / "vehicles/tesla_model3.yaml"

from sim_vehicle_spec import gen  # noqa: E402
from sim_vehicle_spec import spec as specmod  # noqa: E402


def doc():
    return yaml.safe_load(SPEC.read_text())


class ShippedSpecTest(unittest.TestCase):
    def setUp(self):
        self.spec = specmod.load(SPEC)

    def test_the_assumptions_are_marked(self):
        a = specmod.assumptions(self.spec)
        for key in ("inertia.com_front_fraction", "inertia.com_height", "tires"):
            self.assertIn(key, a)
        self.assertNotIn("inertia.mass", a)

    def test_the_box_is_the_measured_body(self):
        # CARLA's vehicle.tesla.model3 (roadmap 050): 4.792 x 2.163 x 1.488, centre 1.415 ahead
        # of the rear axle in the blueprint table (1.416 here, the wheel base being 3.005)
        (cx, cy, cz), (lx, ly, lz) = specmod.box(self.spec)
        self.assertAlmostEqual(lx, 4.792)
        self.assertAlmostEqual(ly, 2.163)
        self.assertAlmostEqual(lz, 1.488)
        self.assertAlmostEqual(cx, 1.416)
        self.assertEqual((cy, cz), (0.0, 0.744))

    def test_the_axles(self):
        self.assertEqual(specmod.axles(self.spec),
                         {"front_x": 3.005, "rear_x": 0.0, "track": 1.667, "wheel_radius": 0.370})

    def test_sensors_are_in_base_link(self):
        s = specmod.sensors(self.spec)
        (t, _), = [s["top"]["base_link"]]
        for a, b in zip(t, (0.9, 0.0, 2.0)):
            self.assertAlmostEqual(a, b)
        self.assertTrue(s["top"]["enabled"])
        self.assertFalse(s["camera6"]["enabled"])
        self.assertEqual(s["top"]["key"], "velodyne_top_base_link")

    def test_camera_intrinsics(self):
        k = specmod.camera_intrinsics(1280, 720, 90.0)
        self.assertAlmostEqual(k[0], 640.0)
        self.assertEqual((k[2], k[5]), (640.0, 360.0))


class ValidationTest(unittest.TestCase):
    def refused(self, d, text):
        with self.assertRaises(specmod.SpecError) as e:
            specmod.validate(d, "x.yaml")
        self.assertIn(text, str(e.exception))

    def test_the_shipped_one_is_valid(self):
        specmod.validate(doc())

    def test_unknown_keys_are_refused(self):
        d = doc()
        d["inertia"]["masss"] = 1.0
        self.refused(d, "inertia: unknown key(s) ['masss']")
        d = doc()
        d["colour"] = "red"
        self.refused(d, "unknown key(s) ['colour']")

    def test_missing_keys_are_refused(self):
        d = doc()
        del d["geometry"]["wheel_base"]
        self.refused(d, "geometry: missing ['wheel_base']")

    def test_a_powertrain_must_be_known(self):
        d = doc()
        d["powertrain"]["model"] = "steam"
        self.refused(d, "powertrain.model 'steam'")

    def test_an_ev_needs_its_drive_unit(self):
        d = doc()
        d["powertrain"]["model"] = "ev"
        specmod.validate(d)
        del d["powertrain"]["gear_ratio"]
        self.refused(d, "an ev needs ['gear_ratio']")
        d = doc()
        d["powertrain"]["model"] = "ev"
        d["powertrain"]["motor_peak_power"] = 0
        self.refused(d, "motor_peak_power must be positive")

    def test_ev_keys_are_checked(self):
        d = doc()
        d["powertrain"]["drive_axle"] = "middle"
        self.refused(d, "drive_axle 'middle'")
        d = doc()
        d["powertrain"]["regen_fade_speeds"] = [3.0, 1.0]
        self.refused(d, "regen_fade_speeds")
        d = doc()
        d["brakes"]["abs"]["slip"] = 0.1
        self.refused(d, "brakes.abs: unknown key(s) ['slip']")
        d = doc()
        d["powertrain"]["traction_control"] = True
        self.refused(d, "powertrain.traction_control is not a mapping")

    def test_rolling_resistance_moved_to_the_physics(self):
        d = doc()
        d["powertrain"]["rolling_resistance"] = 0.0
        self.refused(d, "resistance.rolling_coefficient")

    def test_a_mount_needs_all_six(self):
        d = doc()
        del d["sensors"]["imu"]["mount"]["yaw"]
        self.refused(d, "sensors.imu.mount")

    def test_sensor_keys_by_type(self):
        d = doc()
        d["sensors"]["imu"]["channels"] = 4
        self.refused(d, "unknown key(s) ['channels'] for a imu")
        d = doc()
        del d["sensors"]["camera6"]["width"]
        self.refused(d, "a camera needs width")

    def test_links_are_unique(self):
        d = doc()
        d["sensors"]["imu2"] = copy.deepcopy(d["sensors"]["imu"])
        self.refused(d, "used twice")

    def test_com_fraction_in_range(self):
        d = doc()
        d["inertia"]["com_front_fraction"] = 1.2
        self.refused(d, "com_front_fraction")

    def test_provenance_names_a_section(self):
        d = doc()
        d["provenance"]["nonsense.x"] = {"basis": "?"}
        self.refused(d, "provenance.nonsense.x")


    def test_version_1_is_refused_with_where_its_keys_went(self):
        d = doc()
        d["spec_version"] = 1
        self.refused(d, "spec_version must be 2")
        d = doc()
        d["autoware"] = {"vehicle_model": "x", "sensor_model": "y"}
        self.refused(d, "autoware is gone")
        d = doc()
        d["physx"] = {}
        self.refused(d, "physx is gone in version 2: isaac.physx")

    def test_a_sensors_carla_block_is_checked(self):
        d = doc()
        d["sensors"]["imu"]["carla"]["colour"] = "red"
        self.refused(d, "sensors.imu.carla: unknown key(s) ['colour']")
        d = doc()
        del d["sensors"]["imu"]["carla"]["blueprint"]
        self.refused(d, "sensors.imu.carla: missing blueprint")


class HashTest(unittest.TestCase):
    def load(self, d):
        return specmod.Spec(SPEC, d)

    def test_words_do_not_change_the_hash(self):
        base = specmod.spec_hash(self.load(doc()))
        d = doc()
        d["description"] = "another sentence"
        d["provenance"]["tires"]["basis"] = "another basis"
        self.assertEqual(specmod.spec_hash(self.load(d)), base)
        # nor does the file's layout: the same values reordered
        d = dict(reversed(list(doc().items())))
        self.assertEqual(specmod.spec_hash(self.load(d)), base)

    def test_a_value_changes_the_hash(self):
        base = specmod.spec_hash(self.load(doc()))
        d = doc()
        d["geometry"]["wheel_base"] = 3.006
        self.assertNotEqual(specmod.spec_hash(self.load(d)), base)

    def test_moving_a_sensor_keeps_the_physics_hash(self):
        base, phys = specmod.spec_hash(self.load(doc())), specmod.physics_hash(self.load(doc()))
        d = doc()
        d["sensors"]["gnss"]["mount"]["x"] = 0.5
        self.assertNotEqual(specmod.spec_hash(self.load(d)), base)
        self.assertEqual(specmod.physics_hash(self.load(d)), phys)
        d = doc()
        d["tires"]["friction"] = 0.9
        self.assertNotEqual(specmod.physics_hash(self.load(d)), phys)


class GeneratedTest(unittest.TestCase):
    def test_the_committed_outputs_are_current(self):
        from sim_vehicle_spec import emit
        self.assertEqual(emit.stale(gen.outputs(ROOT), ROOT), [])

    def test_carla_attributes_derive_from_the_shared_fields(self):
        s = specmod.sensors(specmod.load(SPEC))
        top = gen.carla_attributes(s["top"])
        self.assertEqual((top["channels"], top["range"], top["rotation_frequency"],
                          top["upper_fov"], top["lower_fov"]),
                         ("128", "200.0", "20.0", "15.0", "-25.0"))
        cam = gen.carla_attributes(s["camera6"])
        self.assertEqual((cam["image_size_x"], cam["fov"], cam["sensor_tick"]),
                         ("1280", "74.5", "0.1"))

    def test_every_output_names_the_spec_and_its_hash(self):
        h = specmod.spec_hash(specmod.load(SPEC))
        for path, text in gen.outputs(ROOT).items():
            self.assertIn(h, text.split("\n", 3)[0] + text.split("\n", 3)[1], path)

    def test_the_catalog_entry_is_the_car(self):
        xosc = gen.outputs(ROOT)["catalogs/vehicles.xosc"]
        self.assertIn('<Vehicle name="tesla_model3" vehicleCategory="car" model3d="vehicle.tesla.model3">', xosc)
        self.assertIn('<Center x="1.416" y="0.0" z="0.744" />', xosc)
        self.assertIn('<Dimensions width="2.163" height="1.488" length="4.792" />', xosc)
        self.assertIn('positionX="3.005" wheelDiameter="0.74" trackWidth="1.667"', xosc)

    def test_the_catalog_declares_no_defaults(self):
        # In SSv2 an entry's declared default shadows the reference's assignment.
        xosc = gen.outputs(ROOT)["catalogs/vehicles.xosc"]
        self.assertNotIn("<ParameterDeclaration", xosc)
        for name in gen.ASSIGNED:
            self.assertIn(f'"${name}"', xosc)

    def test_carla_physics_is_the_spec_car(self):
        doc = gen.carla_physics_doc(specmod.load(SPEC))
        v, w = doc["vehicle"], doc["wheels"]
        self.assertEqual((v["mass"], v["final_ratio"], v["max_rpm"]), (1760.0, 10.0, 15200.0))
        # flat to base speed (194 kW / 340 N m), constant power above, zero at max_rpm
        curve = v["torque_curve"]
        self.assertEqual(curve[0], [0.0, 340.0])
        self.assertAlmostEqual(curve[1][0], 194000 / 340 * 60 / (2 * 3.141592653589793), 0)
        self.assertEqual(curve[-1], [15200.0, 0.0])
        for rpm, nm in curve[2:-1]:
            self.assertAlmostEqual(nm * rpm * 2 * 3.141592653589793 / 60, 194000.0, -2)
        # the CoM 1.412 m ahead of the rear axle (47 % of 3.005), 0.46 up, from the body's own
        self.assertAlmostEqual(v["center_of_mass"][0], -1.386 + 0.47 * 3.005 - 0.029, 3)
        self.assertAlmostEqual(v["center_of_mass"][2], 0.46 - 0.744, 3)
        self.assertEqual([x["tire_friction"] for x in w], [1.2] * 4)
        self.assertEqual([x["max_handbrake_torque"] for x in w], [0.0, 0.0, 1500.0, 1500.0])
        self.assertEqual([x["max_steer_angle_deg"] for x in w], [70.0, 70.0, 0.0, 0.0])
        self.assertEqual(doc["command_lags"], {"brake_dead_time": 0.08,
                                               "brake_time_constant": 0.06,
                                               "motor_time_constant": 0.04})

    def test_no_carla_physics_no_file(self):
        d = yaml.safe_load(SPEC.read_text())
        del d["carla"]["physics"]
        self.assertIsNone(gen.carla_physics_doc(specmod.Spec(SPEC, d)))


if __name__ == "__main__":
    unittest.main()

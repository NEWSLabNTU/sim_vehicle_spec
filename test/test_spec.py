"""The vehicle spec: loading, validation, what it derives, its fingerprints and the outputs
generated here. Pure: no simulator, no ROS."""
import copy
import sys
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

    def test_the_spec_is_the_real_car(self):
        self.assertEqual(specmod.axles(self.spec),
                         {"front_x": 2.875, "rear_x": 0.0, "track": 1.58, "wheel_radius": 0.334})
        (_, _, _), (lx, ly, lz) = specmod.box(self.spec)
        self.assertAlmostEqual(lx, 4.693)
        self.assertAlmostEqual(ly, 2.088)
        self.assertEqual(self.spec.doc["powertrain"]["gear_ratio"], 9.03)

    def test_the_box_is_the_measured_body(self):
        # CARLA's vehicle.tesla.model3 (roadmap 050): 4.792 x 2.163 x 1.488, centre 1.415 ahead
        # of the rear axle in the blueprint table (1.416 here, the wheel base being 3.005)
        (cx, cy, cz), (lx, ly, lz) = specmod.box(specmod.effective(self.spec, "carla-0.9.16"))
        self.assertAlmostEqual(lx, 4.792)
        self.assertAlmostEqual(ly, 2.163)
        self.assertAlmostEqual(lz, 1.488)
        self.assertAlmostEqual(cx, 1.416)
        self.assertEqual((cy, cz), (0.0, 0.744))

    def test_the_axles(self):
        self.assertEqual(specmod.axles(specmod.effective(self.spec, "carla-0.9.16")),
                         {"front_x": 3.005, "rear_x": 0.0, "track": 1.667, "wheel_radius": 0.370})
        self.assertEqual(specmod.axles(specmod.effective(self.spec, "carla-1.0")),
                         {"front_x": 2.876, "rear_x": 0.0, "track": 1.58, "wheel_radius": 0.336})

    def test_the_1_0_box_is_its_measured_body(self):
        # CARLA 1.0's vehicle.tesla.model3 (blueprint_boxes_1.0.yaml): 4.694 x 2.088 x 1.443,
        # centred 0.068 behind the actor origin, the rear axle 1.438 behind it
        (cx, _, cz), (lx, ly, lz) = specmod.box(specmod.effective(self.spec, "carla-1.0"))
        self.assertAlmostEqual(lx, 4.694)
        self.assertAlmostEqual(ly, 2.088)
        self.assertAlmostEqual(lz, 1.443)
        self.assertAlmostEqual(cx, 1.438 - 0.068, 2)

    def test_sensors_are_in_base_link(self):
        s = specmod.sensors(self.spec)
        (t, _), = [s["top"]["base_link"]]
        for a, b in zip(t, (0.9, 0.0, 2.0)):
            self.assertAlmostEqual(a, b)
        self.assertTrue(s["top"]["enabled"])
        self.assertFalse(s["camera6"]["enabled"])
        self.assertEqual(s["top"]["key"], "velodyne_top_base_link")

    def test_zed_left_on_the_mustang(self):
        spec = specmod.load(Path(ROOT, "vehicles", "drift_mustang.yaml"))
        z = specmod.sensors(spec)["zed_left"]
        self.assertEqual((z["type"], z["width"], z["height"], z["horizontal_fov"], z["rate_hz"]),
                         ("camera", 960, 600, 110.0, 30.0))
        self.assertTrue(z["enabled"])
        self.assertNotIn("carla", spec.doc["sensors"]["zed_left"])
        self.assertEqual((z["topic"], z["info_topic"], z["link"], z["frame"]),
                         ("/sensing/camera/zed/image_raw", "/sensing/camera/zed/camera_info",
                          "zed_left/camera_link", "zed_left/camera_optical_link"))
        (x, y, h), _ = z["base_link"]
        self.assertEqual((round(x, 3), y, round(h, 3)), (1.45, 0.0, 1.2))
        (cx, _, _), (lx, _, lz) = specmod.box(spec)
        self.assertTrue(cx - lx / 2 < x < cx + lx / 2 and h < lz)
        out = gen.outputs(ROOT)["generated/drift_mustang/carla_sensors.yaml"]
        self.assertNotIn("zed", out)

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

    def test_ground_clearance_sits_under_the_axle(self):
        d = doc()
        self.assertEqual(d["body"]["ground_clearance"], 0.138)
        d["body"]["ground_clearance"] = 0.5
        self.refused(d, "body.ground_clearance must be positive and below the wheel radius")
        del d["body"]["ground_clearance"]
        specmod.validate(d)

    def test_a_powertrain_must_be_known(self):
        d = doc()
        d["powertrain"]["model"] = "steam"
        self.refused(d, "powertrain.model 'steam'")

    def test_a_component_powertrain_names_its_entry_point(self):
        """scensim roadmap 091: the car's control component is the user's, named by entry point;
        it needs no max_drive_torque, and only it may carry component and params."""
        d = doc()
        d["powertrain"] = {"model": "component", "component": "my_racer.vesc_servo",
                           "params": {"erpm_gain": 4614.0}}
        specmod.validate(d)
        del d["powertrain"]["params"]
        specmod.validate(d)
        d["powertrain"]["component"] = " "
        self.refused(d, "a component needs component: <entry point name>")
        d["powertrain"] = {"model": "component", "component": "x", "params": [1]}
        self.refused(d, "powertrain.params is not a mapping")
        d = doc()
        d["powertrain"]["component"] = "x"
        self.refused(d, "powertrain.component: only a component powertrain has it")
        d = doc()
        del d["powertrain"]["max_drive_torque"]
        self.refused(d, "powertrain: missing ['max_drive_torque']")

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

    def test_the_differential_is_checked(self):
        d = doc()
        d["powertrain"]["differential"] = {"type": "lsd", "preload": 100.0, "power_lock": 0.5,
                                           "coast_lock": 0.25}
        specmod.validate(d)
        d["powertrain"]["differential"] = {"type": "viscous"}
        self.refused(d, "differential.type 'viscous'")
        d["powertrain"]["differential"] = {"type": "locked", "preload": 50.0}
        self.refused(d, "differential.preload: only an lsd has it")
        d["powertrain"]["differential"] = {"type": "lsd", "power_lock": 1.5}
        self.refused(d, "power_lock must be a share")
        d["powertrain"]["differential"] = {"type": "lsd", "ramp": 45}
        self.refused(d, "powertrain.differential: unknown key(s) ['ramp']")

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

    def test_a_planar_lidar(self):
        """scensim 091 W19: a UST-10LX class scanner, 1081 beams over 270 deg as LaserScan."""
        from sim_vehicle_spec import gen as specgen
        from sim_vehicle_spec import spec as specmod

        d = doc()
        top = d["sensors"]["top"]
        top.update(channels=1, vertical_fov=[0.0, 0.0], horizontal_fov=270.0, beams=1081,
                   rate_hz=40.0, range=10.0, min_range=0.06, message="laser_scan")
        top["carla"].pop("attributes", None)  # the Tesla's own (an OS1's) would win, as they do
        specmod.validate(d)
        self.assertEqual(specmod.planar_columns(top), 1440)
        attrs = specgen.carla_attributes(top)
        self.assertEqual((attrs["horizontal_fov"], attrs["points_per_second"]), ("270.0", "43240.0"))
        bad = copy.deepcopy(d)
        bad["sensors"]["top"]["beams"] = 1082  # 1081 steps over 270 deg: 1441.3 columns a turn
        self.refused(bad, "does not divide 360 deg into whole columns")
        bad = copy.deepcopy(d)
        bad["sensors"]["top"]["channels"] = 16
        self.refused(bad, "a laser_scan lidar has one ring")
        bad = copy.deepcopy(d)
        del bad["sensors"]["top"]["beams"]
        self.refused(bad, "a laser_scan lidar needs beams")
        bad = copy.deepcopy(d)
        bad["sensors"]["top"]["message"] = "scan"
        self.refused(bad, "message 'scan': one of point_cloud, laser_scan")
        bad = copy.deepcopy(d)
        bad["sensors"]["top"]["min_range"] = 20.0
        self.refused(bad, "min_range 20.0")

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


class BodyTest(unittest.TestCase):
    """Roadmap 083: the spec is the real car, each simulator's body an effective spec."""

    def setUp(self):
        self.spec = specmod.load(SPEC)

    def test_0_9_16_is_the_car_the_pedal_maps_were_measured_on(self):
        # The physics_hash autoware_sim_vehicle's pedal maps record: the 0.9.16 body is the
        # spec of before the bodies, value for value.
        e = specmod.effective(self.spec, "carla-0.9.16")
        self.assertEqual(specmod.physics_hash(e),
                         "33635ad5c8611570ffbb9a00b495df5bdc1be06f8f6227280a4c34033acdc267")
        self.assertEqual(e.doc["powertrain"]["gear_ratio"], 10.0)
        self.assertEqual(e.section("body")["catalogue_model"], "vehicle.tesla.model3")
        self.assertNotIn("bodies", e.doc)

    def test_a_body_keeps_the_files_spec_hash(self):
        h = specmod.spec_hash(self.spec)
        for b in specmod.bodies(self.spec):
            self.assertEqual(specmod.spec_hash(specmod.effective(self.spec, b)), h)

    def test_each_body_has_its_own_physics_hash(self):
        a, b = (specmod.physics_hash(specmod.effective(self.spec, n))
                for n in ("carla-0.9.16", "carla-1.0"))
        self.assertNotEqual(a, b)

    def test_a_spec_with_bodies_has_no_physics_hash_of_its_own(self):
        with self.assertRaises(specmod.SpecError) as e:
            specmod.physics_hash(self.spec)
        self.assertIn("carla-0.9.16", str(e.exception))

    def test_a_body_must_be_named_and_known(self):
        with self.assertRaises(specmod.SpecError):
            specmod.effective(self.spec, None)
        with self.assertRaises(specmod.SpecError) as e:
            specmod.effective(self.spec, "carla-0.9.15")
        self.assertIn("carla-1.0", str(e.exception))
        mustang = specmod.load(Path(ROOT, "vehicles", "drift_mustang.yaml"))
        self.assertIs(specmod.effective(mustang, None), mustang)

    def test_isaac_runs_the_0_9_16_body(self):
        # Isaac's ego USD is CARLA 0.9.16's model3 exported
        self.assertEqual(specmod.simulator_body(self.spec, "isaac"), "carla-0.9.16")

    def test_the_reduction_keeps_the_real_cars_force_at_the_road(self):
        real = self.spec.doc["powertrain"]["gear_ratio"] / self.spec.doc["geometry"]["wheel_radius"]
        for b in specmod.bodies(self.spec):
            e = specmod.effective(self.spec, b)
            self.assertAlmostEqual(
                e.doc["powertrain"]["gear_ratio"] / e.doc["geometry"]["wheel_radius"], real, delta=0.1)

    def test_a_body_is_validated(self):
        d = doc()
        del d["bodies"]["carla-1.0"]["geometry"]["wheel_radius"]
        with self.assertRaises(specmod.SpecError) as e:
            specmod.validate(d, "x.yaml")
        self.assertIn("carla-1.0", str(e.exception))
        d = doc()
        d["carla"] = {"physics": d["bodies"]["carla-0.9.16"]["carla"]["physics"]}
        with self.assertRaises(specmod.SpecError) as e:
            specmod.validate(d, "x.yaml")
        self.assertIn("bodies.<name>.carla.physics", str(e.exception))
        d = doc()
        d["simulators"]["isaac"] = "carla-0.9.15"
        with self.assertRaises(specmod.SpecError):
            specmod.validate(d, "x.yaml")


class HashTest(unittest.TestCase):
    def load(self, d):
        return specmod.effective(specmod.Spec(SPEC, d), "carla-0.9.16")

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
        d["bodies"]["carla-0.9.16"]["geometry"]["wheel_base"] = 3.006
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
        # A car's outputs name its own spec; the catalog, every car's, names every spec
        hashes = {p.stem: specmod.spec_hash(specmod.load(p))
                  for p in sorted(Path(ROOT, "vehicles").glob("*.yaml"))}
        for path, text in gen.outputs(ROOT).items():
            names = ([n for n in hashes if f'<Vehicle name="{n}"' in text]
                     if path.startswith("catalogs/") else [path.split("/")[1]])
            self.assertTrue(names, path)
            head = "\n".join(text.split("\n", len(hashes) + 2)[:len(hashes) + 2])
            for name in names:
                self.assertIn(hashes[name], head, f"{path}: {name}")

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
        doc = gen.carla_physics_doc(specmod.effective(specmod.load(SPEC), "carla-0.9.16"))
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
        del d["bodies"]["carla-0.9.16"]["carla"]["physics"]
        e = specmod.effective(specmod.Spec(SPEC, d), "carla-0.9.16")
        self.assertIsNone(gen.carla_physics_doc(e))
        self.assertIsNone(gen.carla_physics_1_0_doc(e))

    def test_the_1_0_physics_is_its_bodys(self):
        e = specmod.effective(specmod.load(SPEC), "carla-1.0")
        doc = gen.carla_physics_1_0_doc(e)
        self.assertEqual(doc["vehicle"]["final_ratio"], 9.084)
        self.assertEqual({w["radius_m"] for w in doc["wheels"]}, {0.336})
        # the CoM at 47 % of the body's wheel base ahead of its rear axle (actor frame)
        self.assertAlmostEqual(doc["vehicle"]["center_of_mass"][0], -1.438 + 0.47 * 2.876, 3)
        self.assertIsNone(gen.carla_physics_doc(e))

    def test_each_body_has_its_catalog(self):
        out = gen.outputs(ROOT)
        self.assertIn('length="4.792"', out["catalogs/vehicles.xosc"])
        one = out["catalogs/carla-1.0/vehicles.xosc"]
        self.assertIn('<Dimensions width="2.088" height="1.443" length="4.694" />', one)
        self.assertNotIn("drift_mustang", one)
        # one `vehicles` catalog per directory: SSv2 keeps the first of a name it finds
        dirs = {}
        for path in out:
            if path.startswith("catalogs/"):
                d = path.rsplit("/", 1)[0]
                self.assertNotIn(d, dirs, f"{path} beside {dirs.get(d)}")
                dirs[d] = path


if __name__ == "__main__":
    unittest.main()

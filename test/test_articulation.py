"""The articulation generator (scensim 092 K2.2b): the body file against the spec."""
import copy
import math
import unittest
from pathlib import Path

import yaml

from sim_vehicle_spec import articulation as art
from sim_vehicle_spec import spec as specmod

ROOT = Path(__file__).resolve().parents[1]


def model3(body="carla-0.9.16"):
    return specmod.effective(specmod.load(ROOT / "vehicles" / "tesla_model3.yaml"), body)


class ArticulationTest(unittest.TestCase):
    def setUp(self):
        self.s = model3()
        self.d = art.articulation_doc(self.s)

    def test_mass_is_the_spec_mass(self):
        total = sum(l["mass"] for l in self.d["links"].values())
        self.assertAlmostEqual(total, self.s.section("inertia")["mass"], places=9)

    def test_structure(self):
        j = self.d["joints"]
        self.assertEqual(sorted(k for k in j if k.endswith("_steer")),
                         ["front_left_steer", "front_right_steer"])
        self.assertEqual(len([k for k in j if k.endswith("_spin")]), 4)
        self.assertEqual(len([k for k in j if k.endswith("_suspension")]), 4)
        for c in art.CORNERS:
            # parent chain ends at the chassis
            child, seen = f"{c}_wheel", 0
            while child != "chassis":
                (jt,) = [v for v in j.values() if v["child"] == child]
                child, seen = jt["parent"], seen + 1
            self.assertEqual(seen, 3 if c.startswith("front") else 2)

    def test_geometry_from_the_body(self):
        g = self.s.section("geometry")
        hub = self.d["joints"]["front_left_spin"]["origin"]
        self.assertEqual(hub, [g["wheel_base"], g["wheel_tread"] / 2, g["wheel_radius"]])
        self.assertEqual(self.d["joints"]["rear_right_spin"]["origin"],
                         [0.0, -g["wheel_tread"] / 2, g["wheel_radius"]])
        self.assertEqual(self.d["joints"]["front_left_steer"]["limits"],
                         [-g["max_steer_angle"], g["max_steer_angle"]])
        self.assertEqual(self.d["links"]["front_left_wheel"]["shape"]["cylinder"]["radius"],
                         g["wheel_radius"])

    def test_suspension_rests_at_design_height(self):
        susp, inertia = self.s.section("suspension"), self.s.section("inertia")
        sprung = inertia["mass"] - 4 * self.s.section("wheels")["mass"]
        load, moment = 0.0, [0.0, 0.0]
        for c in art.CORNERS:
            j = self.d["joints"][f"{c}_suspension"]
            d = j["drive"]
            f = d["stiffness"] * -d["target"]     # the spring's force at q = 0
            load += f
            moment[0] += f * j["origin"][0]
            moment[1] += f * j["origin"][1]
            m = f / art.G
            self.assertAlmostEqual(d["stiffness"], m * (2 * math.pi * susp["frequency"]) ** 2)
            self.assertAlmostEqual(d["damping"],
                                   2 * susp["damping_ratio"] * math.sqrt(d["stiffness"] * m))
        self.assertAlmostEqual(load, sprung * art.G, places=6)
        # the springs carry the chassis about its own centre of mass: no static pitch or roll
        com = self.d["links"]["chassis"]["com"]
        self.assertAlmostEqual(moment[0], sprung * art.G * com[0], places=6)
        self.assertAlmostEqual(moment[1], sprung * art.G * com[1], places=6)

    def test_centre_of_mass_is_the_spec(self):
        inertia = self.s.section("inertia")
        L = self.s.section("geometry")["wheel_base"]
        want = [inertia["com_front_fraction"] * L, inertia.get("com_lateral", 0.0),
                inertia["com_height"]]
        links = self.d["links"].values()
        total = sum(l["mass"] for l in links)
        for i in range(3):
            self.assertAlmostEqual(sum(l["mass"] * l["com"][i] for l in links) / total, want[i],
                                   places=9)

    def test_every_inertia_is_physical(self):
        for s in (self.s, specmod.load(ROOT / "vehicles" / "drift_mustang.yaml")):
            for name, l in art.articulation_doc(s)["links"].items():
                a, b, c = l["inertia"]
                self.assertTrue(a > 0 and b > 0 and c > 0, name)
                for x, y, z in ((a, b, c), (b, c, a), (c, a, b)):
                    self.assertLessEqual(x, y + z + 1e-12, f"{s.name} {name}: {l['inertia']}")

    def test_spin_inertia_is_the_spec(self):
        w = self.s.section("wheels")["mass"]
        r = self.s.section("geometry")["wheel_radius"]
        self.assertAlmostEqual(self.d["links"]["rear_left_wheel"]["inertia"][1], 0.5 * w * r * r)

    def test_default_drives_and_contact(self):
        self.assertEqual(self.d["joints"]["rear_left_spin"]["drive"]["type"], "effort")
        self.assertTrue(self.d["joints"]["rear_left_spin"]["driven"])
        self.assertFalse(self.d["joints"]["front_left_spin"]["driven"])
        self.assertEqual(self.d["joints"]["front_left_steer"]["drive"]["type"], "position")
        mu = self.s.section("tires")["friction"]
        self.assertEqual(self.d["links"]["rear_left_wheel"]["contact"]["static_friction"], mu)
        self.assertEqual(self.d["spec_hash"], specmod.spec_hash(self.s))
        self.assertEqual(self.d["physics_hash"], specmod.physics_hash(self.s))

    def test_default_steer_holds_against_the_scrub_torque(self):
        d = self.d["joints"]["front_left_steer"]["drive"]
        g, links = self.s.section("geometry"), self.d["links"]
        sus = self.d["joints"]["front_left_suspension"]["drive"]
        load = sus["stiffness"] * -sus["target"] + self.s.section("wheels")["mass"] * art.G
        scrub = art._axle_friction(self.s, "front") * load * g["wheel_width"] / 4
        self.assertAlmostEqual(scrub / d["stiffness"], art.STEER_HOLD_RAD, places=9)
        I_z = links["front_left_knuckle"]["inertia"][2] + links["front_left_wheel"]["inertia"][2]
        self.assertAlmostEqual(d["damping"], 2 * math.sqrt(d["stiffness"] * I_z), places=6)

    def test_the_spec_names_its_drives(self):
        doc = copy.deepcopy(self.s.source)
        doc["articulation"] = {
            "spin_drive": {"type": "velocity", "stiffness": 0.0, "damping": 1000.0,
                           "effort_limit": 0.25, "velocity_limit": 450.0},
            "steer_drive": {"type": "position", "stiffness": 100.0, "damping": 10.0,
                            "effort_limit": 3.2, "velocity_limit": 10.0},
            "wheel_contact": {"static_friction": 0.4, "dynamic_friction": 0.4}}
        specmod.validate(doc)
        s = specmod.effective(specmod.Spec(path=self.s.path, doc=doc), "carla-0.9.16")
        d = art.articulation_doc(s)
        self.assertEqual(d["joints"]["rear_left_spin"]["drive"]["damping"], 1000.0)
        self.assertEqual(d["joints"]["front_right_steer"]["drive"]["stiffness"], 100.0)
        self.assertEqual(d["links"]["front_left_wheel"]["contact"]["static_friction"], 0.4)
        self.assertEqual(d["links"]["front_left_wheel"]["contact"]["combine"], "multiply")

    def mushr_like(self, **art_):
        doc = copy.deepcopy(self.s.source)
        doc["articulation"] = art_
        specmod.validate(doc)
        return art.articulation_doc(specmod.effective(specmod.Spec(path=self.s.path, doc=doc),
                                                      "carla-0.9.16"))

    def test_a_dc_motor_on_the_driven_wheels_and_passive_undriven_ones(self):
        # WheeledLab's MuSHR 2WD (scensim 094 P2): DCMotorCfg on the rear, the fronts passive
        dc = {"type": "dc_motor", "stiffness": 0.0, "damping": 1000.0, "effort_limit": 0.5,
              "saturation_effort": 1.05, "velocity_limit": 450.0, "friction": 0.0}
        free = {"type": "effort", "stiffness": 0.0, "damping": 0.0, "effort_limit": 0.0}
        d = self.mushr_like(spin_drive=dc, undriven_spin_drive=free)
        self.assertEqual(d["joints"]["rear_left_spin"]["drive"], dc)
        self.assertEqual(d["joints"]["front_right_spin"]["drive"], free)

    def test_a_suspension_drive_keeps_its_target_or_targets_the_sag(self):
        rigid = {"type": "position", "stiffness": 1e8, "damping": 0.0, "friction": 0.5,
                 "target": 0.0}
        d = self.mushr_like(suspension_drive=rigid)
        self.assertEqual(d["joints"]["front_left_suspension"]["drive"], rigid)
        sag = -self.d["joints"]["front_left_suspension"]["drive"]["target"]
        d = self.mushr_like(suspension_drive={"type": "position", "stiffness": 1e8,
                                              "damping": 0.0})
        self.assertAlmostEqual(d["joints"]["front_left_suspension"]["drive"]["target"], -sag)

    def test_the_articulation_is_in_the_physics_hash_only_when_named(self):
        h = specmod.physics_hash(self.s)
        self.assertEqual(h, self.d["physics_hash"])
        d = self.mushr_like(steer_law="tan")
        self.assertNotEqual(d["physics_hash"], h)
        self.assertNotEqual(self.mushr_like(steer_law="parallel")["physics_hash"],
                            d["physics_hash"])

    def test_the_steer_law(self):
        self.assertEqual(self.d["actuation"]["steer_law"], "ackermann")
        d = self.mushr_like(steer_law="tan")
        self.assertEqual(d["actuation"]["steer_law"], "tan")
        m = self.s.section("geometry")["max_steer_angle"]
        self.assertAlmostEqual(d["joints"]["front_left_steer"]["limits"][1], math.tan(m))
        self.assertEqual(self.d["joints"]["front_left_steer"]["limits"][1], m)
        with self.assertRaisesRegex(specmod.SpecError, "steer_law 'cubic'"):
            self.mushr_like(steer_law="cubic")

    def test_a_dc_motor_needs_its_saturation_and_drives_a_wheel_only(self):
        with self.assertRaisesRegex(specmod.SpecError, "saturation_effort"):
            self.mushr_like(spin_drive={"type": "dc_motor", "damping": 1.0})
        with self.assertRaisesRegex(specmod.SpecError, "saturation_effort"):
            self.mushr_like(spin_drive={"type": "effort", "saturation_effort": 1.0})
        with self.assertRaisesRegex(specmod.SpecError, "spin joint only"):
            self.mushr_like(steer_drive={"type": "dc_motor", "saturation_effort": 1.0})

    def test_bad_drive_type_refused(self):
        doc = copy.deepcopy(self.s.source)
        doc["articulation"] = {"spin_drive": {"type": "torque"}}
        with self.assertRaisesRegex(specmod.SpecError, "spin_drive.type 'torque'"):
            specmod.validate(doc)

    def test_one_file_per_body(self):
        files = art.files(specmod.load(ROOT / "vehicles" / "tesla_model3.yaml"), "h")
        self.assertEqual(sorted(files), ["articulation_carla-0.9.16.yaml",
                                         "articulation_carla-1.0.yaml"])
        d = yaml.safe_load(files["articulation_carla-1.0.yaml"])
        self.assertEqual(d["body"], "carla-1.0")


if __name__ == "__main__":
    unittest.main()

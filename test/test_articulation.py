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
        load = 0.0
        for c in art.CORNERS:
            d = self.d["joints"][f"{c}_suspension"]["drive"]
            load += d["stiffness"] * -d["target"]     # the spring's force at q = 0
            share = inertia["com_front_fraction"] if c.startswith("front") else \
                1 - inertia["com_front_fraction"]
            m = sprung * share / 2
            self.assertAlmostEqual(d["stiffness"], m * (2 * math.pi * susp["frequency"]) ** 2)
            self.assertAlmostEqual(d["damping"],
                                   2 * susp["damping_ratio"] * math.sqrt(d["stiffness"] * m))
        self.assertAlmostEqual(load, sprung * art.G, places=6)

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

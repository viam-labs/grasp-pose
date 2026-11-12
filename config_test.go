package grasppose

import (
	"testing"

	"go.viam.com/test"
)

func parallelConfig() *Config {
	return &Config{
		Vision: "seg", Camera: "cam",
		Gripper: GripperConfig{Type: "parallel", MaxOpeningMM: 85, FingerDepthMM: 40, FingerWidthMM: 22, FingerThicknessMM: 8},
	}
}

func TestValidateAParallelConfigNamesTheVisionService(t *testing.T) {
	deps, optional, err := parallelConfig().Validate("path")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, deps, test.ShouldResemble, []string{"seg"})
	test.That(t, optional, test.ShouldBeEmpty)
}

func TestValidateRejectsMissingAndBadFields(t *testing.T) {
	for _, tc := range []struct {
		field string
		edit  func(c *Config)
	}{
		{"vision", func(c *Config) { c.Vision = "" }},
		{"camera", func(c *Config) { c.Camera = "" }},
		{"gripper.type", func(c *Config) { c.Gripper.Type = "magnet" }},
		{"gripper.max_opening_mm", func(c *Config) { c.Gripper.MaxOpeningMM = 0 }},
		{"gripper.finger_width_mm", func(c *Config) { c.Gripper.FingerWidthMM = -1 }},
		{"gripper.opening_axis", func(c *Config) { c.Gripper.OpeningAxis = "z" }},
		{"grip_depth_mm", func(c *Config) { c.GripDepthMM = 50 }}, // deeper than the 40 mm fingers
		{"grasps_per_object", func(c *Config) { c.GraspsPerObject = -1 }},
		{"support_clearance_mm", func(c *Config) { v := -1.0; c.SupportClearanceMM = &v }},
	} {
		c := parallelConfig()
		tc.edit(c)
		_, _, err := c.Validate("path")
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, tc.field)
	}
}

func TestValidateSuctionNeedsOnlyTheCup(t *testing.T) {
	c := &Config{Vision: "seg", Camera: "cam", Gripper: GripperConfig{Type: "suction", CupDiameterMM: 45}, GripDepthMM: 99}
	_, _, err := c.Validate("path") // grip_depth_mm is ignored for suction
	test.That(t, err, test.ShouldBeNil)
	c.Gripper.CupDiameterMM = 0
	_, _, err = c.Validate("path")
	test.That(t, err.Error(), test.ShouldContainSubstring, "gripper.cup_diameter_mm")
}

func TestSettingsFillTheDefaults(t *testing.T) {
	s := parallelConfig().settings()
	test.That(t, s.graspsPerObject, test.ShouldEqual, 2)
	test.That(t, s.gripDepth, test.ShouldEqual, 15.0)
	test.That(t, s.supportClearance, test.ShouldEqual, 20.0)
	test.That(t, s.supportMargin, test.ShouldEqual, 5.0)
	test.That(t, s.flatness, test.ShouldEqual, 3.0)
	test.That(t, s.parallel, test.ShouldBeTrue)
	test.That(t, s.axisY, test.ShouldBeFalse)
	suction := (&Config{Gripper: GripperConfig{Type: "suction", CupDiameterMM: 45}}).settings()
	test.That(t, suction.minPatch, test.ShouldAlmostEqual, 36.0)
	test.That(t, suction.cupGap, test.ShouldEqual, 1.0)
}

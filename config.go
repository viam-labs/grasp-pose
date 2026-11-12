// Package grasppose serves top-down grasp poses for the objects a vision
// service's GetDetections3D finds, as a pose tracker.
package grasppose

import "fmt"

// GripperConfig describes the gripper, from its datasheet: Viam's gripper API exposes none of this.
type GripperConfig struct {
	Type              string   `json:"type"`
	MaxOpeningMM      float64  `json:"max_opening_mm,omitempty"`
	FingerDepthMM     float64  `json:"finger_depth_mm,omitempty"`
	FingerWidthMM     float64  `json:"finger_width_mm,omitempty"`
	FingerThicknessMM float64  `json:"finger_thickness_mm,omitempty"`
	OpeningAxis       string   `json:"opening_axis,omitempty"`
	CupDiameterMM     float64  `json:"cup_diameter_mm,omitempty"`
	MinPatchMM        float64  `json:"min_patch_mm,omitempty"`
	CupGapMM          *float64 `json:"cup_gap_mm,omitempty"`
}

// Config is the top-down model's configuration.
type Config struct {
	Vision             string        `json:"vision"`
	Camera             string        `json:"camera"`
	Gripper            GripperConfig `json:"gripper"`
	SupportZMM         *float64      `json:"support_z_mm,omitempty"`
	GraspsPerObject    int           `json:"grasps_per_object,omitempty"`
	GripDepthMM        float64       `json:"grip_depth_mm,omitempty"`
	SupportClearanceMM *float64      `json:"support_clearance_mm,omitempty"`
	SupportMarginMM    float64       `json:"support_margin_mm,omitempty"`
	FlatnessMM         float64       `json:"flatness_mm,omitempty"`
}

// settings is a Config with its defaults filled in.
type settings struct {
	parallel, axisY                                      bool
	maxOpening, fingerWidth, fingerThickness             float64
	cupDiameter, minPatch, cupGap                        float64
	supportZ                                             *float64
	graspsPerObject                                      int
	gripDepth, supportClearance, supportMargin, flatness float64
}

func orDefault(v, d float64) float64 {
	if v == 0 {
		return d
	}
	return v
}

func ptrOr(v *float64, d float64) float64 {
	if v == nil {
		return d
	}
	return *v
}

func (c *Config) settings() settings {
	g := c.Gripper
	s := settings{
		parallel: g.Type == "parallel", axisY: g.OpeningAxis == "y",
		maxOpening:  g.MaxOpeningMM,
		fingerWidth: g.FingerWidthMM, fingerThickness: g.FingerThicknessMM,
		cupDiameter: g.CupDiameterMM, minPatch: orDefault(g.MinPatchMM, 0.8*g.CupDiameterMM),
		cupGap: ptrOr(g.CupGapMM, 1), supportZ: c.SupportZMM, graspsPerObject: c.GraspsPerObject,
		gripDepth: orDefault(c.GripDepthMM, 15), supportClearance: ptrOr(c.SupportClearanceMM, 20),
		supportMargin: orDefault(c.SupportMarginMM, 5), flatness: orDefault(c.FlatnessMM, 3),
	}
	if s.graspsPerObject == 0 {
		s.graspsPerObject = 2
	}
	return s
}

type field struct {
	name  string
	value float64
}

// Validate checks the config and returns the vision service as the one dependency
// (viam-server hands every module the frame system without asking).
func (c *Config) Validate(path string) ([]string, []string, error) {
	fail := func(name, why string) ([]string, []string, error) {
		return nil, nil, fmt.Errorf("%s: %q %s", path, name, why)
	}
	if c.Vision == "" {
		return fail("vision", "is required")
	}
	if c.Camera == "" {
		return fail("camera", "is required")
	}
	g := c.Gripper
	switch g.Type {
	case "parallel":
		for _, f := range []field{
			{"gripper.max_opening_mm", g.MaxOpeningMM}, {"gripper.finger_depth_mm", g.FingerDepthMM},
			{"gripper.finger_width_mm", g.FingerWidthMM}, {"gripper.finger_thickness_mm", g.FingerThicknessMM},
		} {
			if f.value <= 0 {
				return fail(f.name, "must be positive")
			}
		}
		if g.OpeningAxis != "" && g.OpeningAxis != "x" && g.OpeningAxis != "y" {
			return fail("gripper.opening_axis", `must be "x" or "y"`)
		}
		if orDefault(c.GripDepthMM, 15) > g.FingerDepthMM {
			return fail("grip_depth_mm", "must be no more than gripper.finger_depth_mm")
		}
	case "suction":
		if g.CupDiameterMM <= 0 {
			return fail("gripper.cup_diameter_mm", "must be positive")
		}
	default:
		return fail("gripper.type", `must be "parallel" or "suction"`)
	}
	for _, f := range []field{
		{"grasps_per_object", float64(c.GraspsPerObject)}, {"grip_depth_mm", c.GripDepthMM},
		{"support_margin_mm", c.SupportMarginMM}, {"flatness_mm", c.FlatnessMM},
		{"gripper.min_patch_mm", g.MinPatchMM},
		{"support_clearance_mm", ptrOr(c.SupportClearanceMM, 0)}, {"gripper.cup_gap_mm", ptrOr(g.CupGapMM, 0)},
	} {
		if f.value < 0 {
			return fail(f.name, "must not be negative")
		}
	}
	return []string{c.Vision}, nil, nil
}

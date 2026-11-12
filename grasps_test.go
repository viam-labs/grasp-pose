package grasppose

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

// axis is where pose turns the unit vector v (the pose's own axis v, in the world).
func axis(p spatialmath.Pose, v r3.Vector) r3.Vector {
	return spatialmath.Compose(p, spatialmath.NewPoseFromPoint(v)).Point().Sub(p.Point())
}

// parallelTo: |cos| of the angle between two horizontal directions is ~1.
func parallelTo(t *testing.T, got r3.Vector, yawDeg float64) {
	t.Helper()
	a := yawDeg * math.Pi / 180
	test.That(t, math.Abs(got.X*math.Cos(a)+got.Y*math.Sin(a)), test.ShouldAlmostEqual, 1, 0.02)
}

func onTable() settings {
	s := parallelConfig().settings()
	s.supportZ = ptr(5)
	return s
}

func block(cx, cy, yaw, length, width float64, label string) cloud {
	return cloud{label: label, points: blockPoints(cx, cy, yaw, length, width, 5, 40, true, true)}
}

func TestDownPoseMatchesTheScriptsDownHeading(t *testing.T) {
	// vision_pick.py's down(x, y, z, h): EulerAngles(roll=pi, yaw=h); its orientation
	// vector is (0, 0, -1) with theta 180 - h (Python SDK: h=30 -> theta 150).
	for _, h := range []float64{0, 30, -60, 89} {
		p := downPose(1, 2, 3, h)
		parallelTo(t, axis(p, r3.Vector{X: 1}), h)
		x := axis(p, r3.Vector{X: 1})
		test.That(t, x.X, test.ShouldAlmostEqual, math.Cos(h*math.Pi/180), 1e-6) // pointing along h, not -h
		test.That(t, axis(p, r3.Vector{Z: 1}).Z, test.ShouldAlmostEqual, -1, 1e-6)
	}
	for h, theta := range map[float64]float64{30: 150, -60: -120, 60: 120} {
		m := poseMap(downPose(0, 0, 0, h))
		test.That(t, m["o_z"], test.ShouldAlmostEqual, -1, 1e-9)
		test.That(t, m["theta"], test.ShouldAlmostEqual, theta, 1e-6)
	}
}

func TestABarGetsOneGraspAcrossItsShortSide(t *testing.T) {
	grasps, objects := plan([]cloud{block(300, 0, 30, 80, 40, "rectangle-red")}, onTable())
	test.That(t, len(grasps), test.ShouldEqual, 1) // 80 + 10 doesn't fit 85
	g := grasps[0]
	test.That(t, g.Name, test.ShouldEqual, "object-0/grasp-0")
	test.That(t, g.Object, test.ShouldEqual, "object-0")
	test.That(t, g.Label, test.ShouldEqual, "rectangle-red")
	test.That(t, g.WidthMM, test.ShouldAlmostEqual, 40, 3)
	test.That(t, g.Pose.Point().X, test.ShouldAlmostEqual, 300, 1.5)
	test.That(t, g.Pose.Point().Z, test.ShouldAlmostEqual, 30, 1) // top 45 - grip depth 15
	parallelTo(t, axis(g.Pose, r3.Vector{X: 1}), 120)             // across the short side
	test.That(t, objects[0].Name, test.ShouldEqual, "object-0")
	test.That(t, objects[0].YawDeg, test.ShouldAlmostEqual, 30, 1)
	test.That(t, objects[0].DimsMM.X, test.ShouldAlmostEqual, 80, 3)
	test.That(t, objects[0].DimsMM.Z, test.ShouldAlmostEqual, 40, 1)
	test.That(t, objects[0].Center.Point().Z, test.ShouldAlmostEqual, 25, 1)
}

func TestOpeningAxisYTurnsTheGraspAQuarter(t *testing.T) {
	s := onTable()
	s.axisY = true
	grasps, _ := plan([]cloud{block(300, 0, 30, 80, 40, "")}, s)
	parallelTo(t, axis(grasps[0].Pose, r3.Vector{Y: 1}), 120)
}

func TestTheToolPointStaysClearOfAKnownSupport(t *testing.T) {
	low := cloud{points: blockPoints(300, 0, 0, 40, 40, 5, 20, true, true)} // top 25
	grasps, _ := plan([]cloud{low}, onTable())
	test.That(t, grasps[0].Pose.Point().Z, test.ShouldAlmostEqual, 25, 1)     // max(25 - 15, 5 + 20)
	lower := cloud{points: blockPoints(300, 0, 0, 40, 40, 5, 15, true, true)} // top 20 < 25
	grasps, objects := plan([]cloud{lower}, onTable())
	test.That(t, grasps, test.ShouldBeEmpty)
	test.That(t, len(objects), test.ShouldEqual, 1) // still an obstacle
}

func TestAnEstimatedSupportSkipsTheClamp(t *testing.T) {
	// obstacles-pointcloud's shape: the table already gone, sides from z 17 to the top at 45
	sides := cloud{points: blockPoints(300, 0, 0, 40, 40, 17, 28, true, false)}
	grasps, _ := plan([]cloud{sides}, parallelConfig().settings())
	test.That(t, len(grasps), test.ShouldBeGreaterThan, 0)
	test.That(t, grasps[0].Pose.Point().Z, test.ShouldAlmostEqual, 30, 1) // top - 15, no clamp
}

func TestSuctionNeedsATopPatchOfMostOfTheCup(t *testing.T) {
	s := (&Config{Gripper: GripperConfig{Type: "suction", CupDiameterMM: 45}, SupportZMM: ptr(5)}).settings()
	grasps, _ := plan([]cloud{block(300, 0, 30, 80, 40, "")}, s)
	test.That(t, len(grasps), test.ShouldEqual, 1)                        // 40 >= 36
	test.That(t, grasps[0].Pose.Point().Z, test.ShouldAlmostEqual, 46, 1) // 1 mm over the top
	test.That(t, grasps[0].Pose.Point().X, test.ShouldAlmostEqual, 300, 1.5)
	grasps, _ = plan([]cloud{block(300, 0, 0, 30, 30, "")}, s)
	test.That(t, grasps, test.ShouldBeEmpty)
}

func TestAFingerOnANeighbourDropsThatGrasp(t *testing.T) {
	// a 40 mm square with a neighbour 60 mm along x: the x-spanning fingers (at 300 +/- 46.5) land on it
	grasps, _ := plan([]cloud{block(300, 0, 0, 40, 40, ""), block(360, 0, 0, 40, 40, "")}, onTable())
	var mine []Grasp
	for _, g := range grasps {
		if g.Object == "object-0" {
			mine = append(mine, g)
		}
	}
	test.That(t, len(mine), test.ShouldEqual, 1)
	parallelTo(t, axis(mine[0].Pose, r3.Vector{X: 1}), 90)
}

func TestGraspsRankByClearance(t *testing.T) {
	clouds := []cloud{block(300, 0, 0, 40, 40, ""), block(410, 0, 0, 40, 40, ""), block(300, 160, 0, 40, 40, "")}
	grasps, _ := plan(clouds, onTable())
	var mine []Grasp
	for _, g := range grasps {
		if g.Object == "object-0" {
			mine = append(mine, g)
		}
	}
	test.That(t, len(mine), test.ShouldEqual, 2)
	parallelTo(t, axis(mine[0].Pose, r3.Vector{X: 1}), 90) // spanning y: the far neighbour is 110 mm along x
	parallelTo(t, axis(mine[1].Pose, r3.Vector{X: 1}), 0)
	test.That(t, mine[0].Score, test.ShouldBeGreaterThan, mine[1].Score)
}

func TestObjectsAreOrderedNearestTheOriginFirst(t *testing.T) {
	_, objects := plan([]cloud{block(600, 0, 0, 40, 40, "far"), block(300, 0, 0, 40, 40, "near")}, onTable())
	test.That(t, objects[0].Label, test.ShouldEqual, "near")
	test.That(t, objects[1].Name, test.ShouldEqual, "object-1")
}

func TestTooFewPointsIsNoObject(t *testing.T) {
	grasps, objects := plan([]cloud{{points: blockPoints(0, 0, 0, 6, 6, 5, 40, false, false)}}, onTable())
	test.That(t, grasps, test.ShouldBeEmpty)
	test.That(t, objects, test.ShouldBeEmpty)
}

func TestAnEmptyCloudIsNoObject(t *testing.T) {
	grasps, objects := plan([]cloud{{}}, parallelConfig().settings())
	test.That(t, grasps, test.ShouldBeEmpty)
	test.That(t, objects, test.ShouldBeEmpty)
}

func TestASmallDetectionStillCountsForClearance(t *testing.T) {
	var small []r3.Vector
	for i := 0; i < 20; i++ {
		small = append(small, r3.Vector{X: 344 + float64(i%5), Y: -5 + float64(i/5)*3, Z: 45})
	}
	grasps, objects := plan([]cloud{block(300, 0, 0, 40, 40, ""), {points: small}}, onTable())
	test.That(t, len(objects), test.ShouldEqual, 1)
	test.That(t, len(grasps), test.ShouldEqual, 1)
	parallelTo(t, axis(grasps[0].Pose, r3.Vector{X: 1}), 90)
}

func TestTiesGoToTheNarrowerGrip(t *testing.T) {
	grasps, _ := plan([]cloud{{points: blockPoints(300, 0, 0, 44, 40, 5, 40, true, false)}}, onTable())
	test.That(t, len(grasps), test.ShouldEqual, 2)
	test.That(t, grasps[0].WidthMM, test.ShouldAlmostEqual, 40, 3)
	test.That(t, grasps[1].WidthMM, test.ShouldAlmostEqual, 44, 3)
}

func TestATopOnlyBoxIsAsTallAsTheSupportMargin(t *testing.T) {
	s := parallelConfig().settings()
	_, objects := plan([]cloud{{points: blockPoints(300, 0, 0, 40, 40, 5, 40, false, false)}}, s)
	test.That(t, len(objects), test.ShouldEqual, 1)
	test.That(t, objects[0].DimsMM.Z, test.ShouldEqual, s.supportMargin)
}

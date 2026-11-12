package grasppose

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/test"
)

func TestMinAreaRectFindsATurnedRectangle(t *testing.T) {
	r := minAreaRect(blockPoints(100, 50, 30, 80, 40, 5, 40, false, false))
	test.That(t, r.cx, test.ShouldAlmostEqual, 100, 1)
	test.That(t, r.cy, test.ShouldAlmostEqual, 50, 1)
	test.That(t, r.yawDeg, test.ShouldAlmostEqual, 30, 1)
	test.That(t, r.length, test.ShouldAlmostEqual, 80, 3)
	test.That(t, r.width, test.ShouldAlmostEqual, 40, 3)
}

func TestMinAreaRectOnASquareFindsItsTurnModuloAQuarter(t *testing.T) {
	r := minAreaRect(blockPoints(0, 0, 20, 50, 50, 5, 40, false, false))
	test.That(t, r.length, test.ShouldAlmostEqual, 50, 3)
	test.That(t, r.width, test.ShouldAlmostEqual, 50, 3)
	test.That(t, math.Mod(r.yawDeg+90, 90), test.ShouldAlmostEqual, 20, 1)
}

func TestNormDegIsInMinus90To90(t *testing.T) {
	for in, want := range map[float64]float64{120: -60, -90: -90, 90: -90, 30: 30, 270: 90 - 180} {
		test.That(t, normDeg(in), test.ShouldAlmostEqual, want, 1e-9)
	}
}

func TestDropSupportWithAConfiguredLevelDropsTheTable(t *testing.T) {
	s := parallelConfig().settings()
	s.supportZ = ptr(5)
	kept, level, known := dropSupport(blockPoints(0, 0, 0, 40, 40, 5, 40, true, true), s)
	test.That(t, known, test.ShouldBeTrue)
	test.That(t, level, test.ShouldEqual, 5.0)
	for _, p := range kept {
		test.That(t, p.Z, test.ShouldBeGreaterThan, 10)
	}
}

func TestDropSupportEstimatedKeepsATopOnlyObject(t *testing.T) {
	s := parallelConfig().settings()
	top := blockPoints(0, 0, 0, 40, 40, 5, 40, false, false) // all at z 45: a 0 mm span
	kept, _, known := dropSupport(top, s)
	test.That(t, known, test.ShouldBeFalse)
	test.That(t, len(kept), test.ShouldEqual, len(top))
}

func TestOwnClusterDropsANeighboursCorner(t *testing.T) {
	block := blockPoints(0, 0, 0, 40, 40, 5, 40, false, false)
	corner := []r3.Vector{}
	for x := 40.0; x <= 50; x += 2 {
		for y := 40.0; y <= 50; y += 2 {
			corner = append(corner, r3.Vector{X: x, Y: y, Z: 45})
		}
	}
	got := ownCluster(append(append([]r3.Vector{}, block...), corner...))
	test.That(t, len(got), test.ShouldEqual, len(block))
}
